import json,os,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Bool
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import GetParameters
from ur5e_block_bringup.physical_stack import JOINTS,HAND,scene_fingerprint
assert os.environ.get('ROS_DOMAIN_ID')=='0'
rclpy.init(); n=rclpy.create_node('diagnose_physical_stop_readonly')
report={'actual_goals_sent':False,'hardware':{},'controllers':{},'services':{},'joints':{},'sensor_samples':[],'program_running':None}
last_joint={}
def joint(m):
    t=time.monotonic()
    for name,q in zip(m.name,m.position):
        report['joints'][name]=q
        prev=last_joint.get(name,t)
        report['max_joint_message_gap']=max(report.get('max_joint_message_gap',0),t-prev)
        last_joint[name]=t
def sensor(m):
    if HAND not in m.joint_names:return
    v=m.interface_values[m.joint_names.index(HAND)]
    d=dict(zip(v.interface_names,v.values));d['received']=time.monotonic()
    report['sensor_samples'].append(d)
n.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)
n.create_subscription(DynamicJointState,'/gripper/dynamic_joint_states',sensor,qos_profile_sensor_data)
n.create_subscription(Bool,'/io_and_status_controller/robot_program_running',lambda m:report.update(program_running=m.data),qos_profile_sensor_data)
def call(kind,path,req=None):
    c=n.create_client(kind,path);start=time.monotonic()
    if not c.wait_for_service(timeout_sec=8.):raise RuntimeError(path+' unavailable')
    f=c.call_async(req or kind.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=12.)
    report['services'][path]=time.monotonic()-start
    if not f.done() or f.result() is None:raise RuntimeError(path+' timeout')
    ans=f.result();n.destroy_client(c);return ans
for manager in ('/controller_manager','/gripper/controller_manager'):
    report['hardware'][manager]=[{'name':h.name,'plugin':h.class_type,'state':h.state.label}for h in call(ListHardwareComponents,manager+'/list_hardware_components').component]
    report['controllers'][manager]=[{'name':c.name,'state':c.state}for c in call(ListControllers,manager+'/list_controllers').controller]
r=GetPlanningScene.Request();r.components.components=1023
report['scene_hash']=scene_fingerprint(call(GetPlanningScene,'/get_planning_scene',r).scene)
p=Path('/home/ksn/.ros/ur5e_a3_stack/physical_plan_domain0_cube_plate_v1.recipe.json')
recipe=json.loads(p.read_text());report['matching_steps']=[{'index':s['index'],'label':s['label'],'phase':s['phase']}for s in recipe['steps'] if s['scene_before_sha256']==report['scene_hash']]
r=GetParameters.Request();r.names=['robot_description','robot_description_semantic']
report['model_parameter_sizes']=[len(v.string_value)for v in call(GetParameters,'/move_group/get_parameters',r).values]
end=time.monotonic()+3.
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
s=report.pop('sensor_samples');report['sensor_sample_count']=len(s)
if s:
    report['last_sensor']={k:v for k,v in s[-1].items() if k!='received'}
    report['max_communication_age']=max(v.get('communication_age',0)for v in s)
    report['max_sensor_message_gap']=max((b['received']-a['received']for a,b in zip(s,s[1:])),default=0)
report['all7_fresh']=all(time.monotonic()-last_joint.get(j,0)<.5for j in JOINTS)
Path('/home/ksn/ur5e_ws/validation/physical_stop_actual_readonly.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False));n.destroy_node();rclpy.shutdown()
