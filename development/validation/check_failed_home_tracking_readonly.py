"""Collision-check five wrist3 offsets along the previously aborted home path."""
import json,time,math,os
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPlanningScene,GetStateValidity
from ur5e_block_bringup.air_trajectory import sample_trajectory
from ur5e_block_bringup.physical_stack import scene_fingerprint
assert os.environ.get('ROS_DOMAIN_ID')=='0'
p=Path('/home/ksn/.ros/ur5e_a3_stack/physical_plan_domain0_cube_plate_v1.recipe.json')
r=json.loads(p.read_text());m=json.loads(Path(str(p)+'.meta.json').read_text())
assert m['plan_id']=='293399cdcd2a33e7','The failed plan changed; check context again'
step=next(s for s in r['steps']if s['label']=='Prepare vertical working home')
path=deserialize_message((p.parent/step['file']).read_bytes(),RobotTrajectory)
rclpy.init();n=rclpy.create_node('failed_home_tracking_readonly')
def call(kind,name,request):
    c=n.create_client(kind,name);assert c.wait_for_service(timeout_sec=8.)
    f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=10.)
    assert f.done()and f.result();out=f.result();n.destroy_client(c);return out
request=GetPlanningScene.Request();request.components.components=1023
scene=scene_fingerprint(call(GetPlanningScene,'/get_planning_scene',request).scene)
assert scene==step['scene_before_sha256'],'Live geometry changed since failed home; do not treat this as matching validation'
samples=list(sample_trajectory(path,max_dt=.05,max_joint_step=.01))
c=n.create_client(GetStateValidity,'/check_state_validity');assert c.wait_for_service(timeout_sec=8.)
work=[]
for stamp,values in samples:
    for offset in (-.01,-.005,0.,.005,.01):
        q=list(map(float,values));q[5]+=offset;q.append(.3999);work.append((stamp,offset,q))
pending=[];sent=checked=0;invalid=[];began=time.monotonic()
while sent<len(work)or pending:
    while sent<len(work)and len(pending)<8:
        stamp,offset,q=work[sent];req=GetStateValidity.Request();req.robot_state.is_diff=True;req.robot_state.joint_state.name=r['joint_names'];req.robot_state.joint_state.position=q
        pending.append((time.monotonic(),stamp,offset,c.call_async(req)));sent+=1
    rclpy.spin_once(n,timeout_sec=.005)
    for _ in range(len(pending)):
        t,stamp,offset,f=pending.pop(0)
        if not f.done():
            assert time.monotonic()-t<10.,'Read-only validation timeout';pending.append((t,stamp,offset,f));continue
        ans=f.result();assert ans is not None;checked+=1
        if not ans.valid:
            invalid.append({'time':stamp,'wrist3_offset':offset,'contacts':[{'a':x.contact_body_1,'b':x.contact_body_2}for x in ans.contacts]})
report={'actual_goals_sent':False,'scene_writes_sent':False,'plan_id':m['plan_id'],'step':step['label'],'nominal_arm_samples':len(samples),'wrist3_offsets':[-.01,-.005,0.,.005,.01],'checked':checked,'invalid':invalid,'elapsed_seconds':time.monotonic()-began,'scene_unchanged':scene_fingerprint(call(GetPlanningScene,'/get_planning_scene',request).scene)==scene,'result':'passed'if not invalid else'collision_found'}
Path('/home/ksn/ur5e_ws/validation/failed_home_tracking_readonly.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));n.destroy_node();rclpy.shutdown()
