"""Mock-only integration probe; never usable in the real robot domain."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListHardwareComponents

parser=argparse.ArgumentParser()
parser.add_argument('mode',choices=['step','continuous'])
parser.add_argument('log')
args=parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID')=='95', 'Only isolated mock domain 95 is allowed'
p=Path(args.log)
rclpy.init()
n=rclpy.create_node('approval_mock_probe')
q={}; previews=set()
def state(m): q.update(zip(m.name,m.position))
def preview(m): previews.add(hashlib.sha256(json.dumps(message_to_ordereddict(m),sort_keys=True,allow_nan=False,separators=(',', ':')).encode()).hexdigest())
s1=n.create_subscription(JointState,'/joint_states',state,qos_profile_sensor_data)
s2=n.create_subscription(DisplayTrajectory,'/display_planned_path',preview,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def until(predicate,seconds=45):
    end=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>end: raise TimeoutError(p.read_text()[-1000:])
        rclpy.spin_once(n,timeout_sec=.02)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.02)
def call(kind,name,request):
    c=n.create_client(kind,name)
    assert c.wait_for_service(timeout_sec=4),name
    f=c.call_async(request)
    until(f.done,5)
    result=f.result()
    n.destroy_client(c)
    return result
def records(marker):
    return [json.loads(line.split(marker+' ',1)[1]) for line in p.read_text().splitlines() if marker+' ' in line]
reply=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
assert any(c.class_type=='mock_components/GenericSystem' for c in reply.component)
assert not any('URPosition' in c.class_type for c in reply.component)
until(lambda: len(q)>=7 and records('STACK_STEP_READY'))
first=records('STACK_STEP_READY')[0]
until(lambda:first['preview_sha256'] in previews)
initial=dict(q)
spin(2)
assert q==initial and not records('STACK_STEP_EXECUTING'), 'Motion before approval'
response=call(Trigger,first['approve_service'],Trigger.Request())
assert response.success
if args.mode=='step':
    until(lambda:len(records('STACK_STEP_READY'))==2)
    second=records('STACK_STEP_READY')[1]
    assert first['index']==1 and second['index']==2 and first['total']==second['total']==7
    until(lambda:second['preview_sha256'] in previews)
    assert any(abs(q[k]-initial[k])>1e-5 for k in initial), 'Approved first step did not move'
    held=dict(q)
    spin(2)
    assert q==held, 'Next step moved without a second approval'
    assert first['approve_service'] not in dict(n.get_service_names_and_types()), 'Old approval service still active'
    assert call(Trigger,'/ur5e_a3_stack/stop',Trigger.Request()).success
    until(lambda:'STACK_ERROR' in p.read_text() and 'STACK_STOPPED_CONFIRMED' in p.read_text())
    assert [v['index'] for v in records('STACK_STEP_EXECUTING')]==[1]
    result={'mode':'step','preview_hash_checked':True,'held_before_first_OK':True,'held_before_second_OK':True,'one_OK_one_segment':True,'old_approval_service_removed':True,'stop_while_waiting_confirmed':True}
else:
    assert first['index']==first['total']==1
    until(lambda:'STACK_STATUS phase=DONE' in p.read_text(),300)
    assert len(records('STACK_STEP_READY'))==1 and len(records('STACK_STEP_EXECUTING'))==1
    assert len(records('STACK_STEP_COMPLETED'))==1
    result={'mode':'continuous','preview_hash_checked':True,'held_before_OK':True,'one_OK_full_path':True,'completed':'DONE'}
p.with_suffix('.result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
n.destroy_node();rclpy.shutdown()
