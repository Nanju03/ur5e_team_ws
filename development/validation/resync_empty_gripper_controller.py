"""Reseed the idle gripper controller from measured position; no action goals."""
import fcntl,json,math,os,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
from action_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import SwitchController,ListControllers
from ur5e_block_bringup.physical_stack import JOINTS,HAND
assert os.environ.get('ROS_DOMAIN_ID')=='0'
locks=[]
for path in ('/tmp/ur5e_air_task_0.lock','/tmp/ur5e_stack_0.lock'):
    fd=os.open(path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600);fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(fd)
rclpy.init();n=rclpy.create_node('resync_empty_gripper_controller');values={};sensor={};statuses=[];status_seen=False
before_driver=(Path('/proc/14935/cmdline').read_bytes())
assert b'ros2_control_node' in before_driver
report={'action_goals_sent':False,'hardware_initialization':False,'controller':'dh_ag95_gripper_controller','driver_pid':14935}
def joints(m):
    for i,name in enumerate(m.name):
        if i<len(m.position)and i<len(m.velocity):values[name]=(m.position[i],m.velocity[i],time.monotonic())
def dynamic(m):
    for name,v in zip(m.joint_names,m.interface_values):
        if name==HAND:sensor.update(zip(v.interface_names,v.values));sensor['received']=time.monotonic()
def status(m):
    global status_seen
    statuses[:]=[s.status for s in m.status_list];status_seen=True
n.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
n.create_subscription(DynamicJointState,'/gripper/dynamic_joint_states',dynamic,qos_profile_sensor_data)
n.create_subscription(GoalStatusArray,'/gripper/dh_ag95_gripper_controller/gripper_cmd/_action/status',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
def safe():
    now=time.monotonic()
    return status_seen and not any(s in (1,2,3)for s in statuses)and all(j in values and math.isfinite(values[j][0])and abs(values[j][1])<.01 and now-values[j][2]<.5for j in JOINTS)and sensor.get('grip_state')==1 and sensor.get('communication_age',99)<.5 and now-sensor.get('received',0)<.5
began=time.monotonic();quiet_since=None
while True:
    rclpy.spin_once(n,timeout_sec=.02)
    if safe():
        quiet_since=quiet_since or time.monotonic()
        if time.monotonic()-quiet_since>.3:break
    else:quiet_since=None
    assert time.monotonic()-began<10.,'Fresh idle empty gripper + quiet ARM6 were not proven'
report['before']={j:values[j][0]for j in JOINTS};seq=sensor['sample_sequence']
def call(kind,name,request):
    c=n.create_client(kind,name)
    assert c.wait_for_service(timeout_sec=8.),name+' unavailable'
    f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=10.)
    assert f.done()and f.result(),name+' timeout'
    ans=f.result();n.destroy_client(c);return ans
def states():return {c.name:c.state for c in call(ListControllers,'/gripper/controller_manager/list_controllers',ListControllers.Request()).controller}
assert states().get('dh_ag95_gripper_controller')=='active'
assert safe(),'Safety state changed before controller switch'
def switch(enable):
    request=SwitchController.Request();fields=request.get_fields_and_field_types()
    setattr(request,'activate_controllers'if 'activate_controllers'in fields else 'start_controllers',['dh_ag95_gripper_controller']if enable else [])
    setattr(request,'deactivate_controllers'if 'deactivate_controllers'in fields else 'stop_controllers',[]if enable else ['dh_ag95_gripper_controller'])
    request.strictness=2;request.timeout.sec=5
    assert call(SwitchController,'/gripper/controller_manager/switch_controller',request).ok
switch(False)
assert states()['dh_ag95_gripper_controller']=='inactive'
assert safe(),'Safety state changed while controller inactive'
switch(True)
assert states()['dh_ag95_gripper_controller']=='active'
began=time.monotonic();quiet_since=None
while True:
    rclpy.spin_once(n,timeout_sec=.02)
    unchanged=all(abs(values.get(j,(math.inf,))[0]-report['before'][j])<(.003if j==HAND else .001)for j in JOINTS)
    assert unchanged,'Measured arm or finger moved during current-position hold'
    if safe()and sensor.get('sample_sequence',0)>seq+3:
        quiet_since=quiet_since or time.monotonic()
        if time.monotonic()-quiet_since>.4:break
    else:quiet_since=None
    assert time.monotonic()-began<8.,'Fresh stable post-switch feedback missing'
assert Path('/proc/14935/cmdline').read_bytes()==before_driver
report.update(result='passed',after={j:values[j][0]for j in JOINTS},sensor={k:sensor[k]for k in ('position','raw_position','grip_state','force_percent','communication_age','sample_sequence')})
Path('/home/ksn/ur5e_ws/validation/gripper_controller_resync.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
n.destroy_node();rclpy.shutdown()
for fd in locks:os.close(fd)
