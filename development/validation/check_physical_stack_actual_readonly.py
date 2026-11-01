import json,time,hashlib
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
from moveit_msgs.srv import GetPlanningScene
from ur5e_block_bringup.physical_stack import JOINTS,scene_fingerprint,SCENE_HASH_POLICY
assert __import__('os').environ.get('ROS_DOMAIN_ID')=='0'
rclpy.init()
n=rclpy.create_node('physical_stack_final_readonly')
js,ds={},{}
def joints(m):
    for k,q in zip(m.name,m.position): js[k]=(q,time.monotonic())
def dynamic(m):
    for k,v in zip(m.joint_names,m.interface_values):
        if k=='left_outer_knuckle_joint': ds.update(zip(v.interface_names,v.values)); ds['received']=time.monotonic()
n.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
n.create_subscription(DynamicJointState,'/gripper/dynamic_joint_states',dynamic,qos_profile_sensor_data)
def call(kind,path,req=None):
    c=n.create_client(kind,path)
    if not c.wait_for_service(timeout_sec=8.): raise RuntimeError(path+' unavailable')
    f=c.call_async(req or kind.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=10.)
    if not f.done() or f.result() is None: raise RuntimeError(path+' timeout')
    ans=f.result(); n.destroy_client(c); return ans
report={'actual_robot_goals_sent':False,'mode':'read_only','hardware':{},'controllers':{}}
for manager in ('/controller_manager','/gripper/controller_manager'):
    report['hardware'][manager]=[{'name':h.name,'type':h.type,'state':h.state.label} for h in call(ListHardwareComponents,manager+'/list_hardware_components').component]
    report['controllers'][manager]=[{'name':c.name,'state':c.state} for c in call(ListControllers,manager+'/list_controllers').controller]
p=Path('/home/ksn/ur5e_ws/validation/physical_stack_actual_preflight.recipe.json')
recipe=json.loads(p.read_text()); meta=json.loads(Path(str(p)+'.meta.json').read_text())
r=GetPlanningScene.Request();r.components.components=1023
live=scene_fingerprint(call(GetPlanningScene,'/get_planning_scene',r).scene)
assert live==meta['scene_sha256'], 'Final live scene differs from restored reviewed initial scene'
assert meta['scene_hash_policy']==SCENE_HASH_POLICY
assert hashlib.sha256(p.read_bytes()).hexdigest()==meta['recipe_sha256']
assert all(a['scene_after_sha256']==b['scene_before_sha256'] for a,b in zip(recipe['steps'],recipe['steps'][1:]))
assert all(s['scene_before_sha256']==s['scene_after_sha256'] for s in recipe['steps'] if s['kind']!='scene')
for _ in range(30): rclpy.spin_once(n,timeout_sec=.05)
assert set(JOINTS)<=set(js)
assert all(time.monotonic()-js[j][1]<.5 for j in JOINTS)
assert ds.get('grip_state')==1 and ds.get('communication_age',99)<.5
report.update(result='passed',joint_positions={j:js[j][0] for j in JOINTS},sensor={k:ds[k] for k in ('position','raw_position','grip_state','force_percent','communication_age','sample_sequence') if k in ds},plan_id=meta['plan_id'],scene_hash=live,scene_chain_valid=True,validated_samples=meta['validated_samples'],steps=len(recipe['steps']),moves=recipe['moves'],layers=recipe['layers'],target_cell=recipe['target_cell'],duration=meta['duration'])
Path('/home/ksn/ur5e_ws/validation/physical_stack_final_actual_readonly.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('result','actual_robot_goals_sent','validated_samples','steps','moves','layers','target_cell','sensor')},ensure_ascii=False))
n.destroy_node();rclpy.shutdown()
