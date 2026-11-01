import copy,json,os,time
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import PlanningScene,AttachedCollisionObject,CollisionObject
from rosidl_runtime_py.convert import message_to_ordereddict
from ur5e_block_bringup import air_scene
assert os.environ.get('ROS_DOMAIN_ID')=='103'
rclpy.init(); n=rclpy.create_node('scene_roundtrip_mock_only')
def call(kind,name,request):
 c=n.create_client(kind,name);assert c.wait_for_service(timeout_sec=5)
 f=c.call_async(request);end=time.monotonic()+10
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 assert f.done();v=f.result();n.destroy_client(c);return v
hs=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
assert hs.component and all(h.class_type=='mock_components/GenericSystem' for h in hs.component)
b=Path('/home/ksn/ur5e_ws/validation');r=json.loads((b/'physical_stack_one.recipe.json').read_text())
def load(name):return deserialize_message((b/name).read_bytes(),PlanningScene)
def scene():
 q=GetPlanningScene.Request();q.components.components=1023
 return call(GetPlanningScene,'/get_planning_scene',q).scene
def apply(s):
 q=ApplyPlanningScene.Request();q.scene=s
 assert call(ApplyPlanningScene,'/apply_planning_scene',q).success
names=r['joint_names'];values=r['initial_joints']
def ready(s):
 s=copy.deepcopy(s);s.is_diff=False;s.robot_state.is_diff=False
 s.robot_state.joint_state.name=names;s.robot_state.joint_state.position=[float(q) for q in values]
 s.robot_state.joint_state.velocity=[];s.robot_state.joint_state.effort=[]
 return s
def clear():
 live=scene();d=PlanningScene();d.is_diff=True;d.robot_state.is_diff=True
 for body in live.robot_state.attached_collision_objects:
  x=AttachedCollisionObject();x.link_name=body.link_name;x.object.id=body.object.id;x.object.operation=CollisionObject.REMOVE;d.robot_state.attached_collision_objects.append(x)
 if d.robot_state.attached_collision_objects:apply(d)
def canonical(s):
 x=message_to_ordereddict(s)
 return dict(world=air_scene._canonical(x['world']),attached=air_scene._canonical(x['robot_state']['attached_collision_objects']),acm=air_scene._allowed_collisions(x['allowed_collision_matrix']))
def diff(a,z,p=''):
 if isinstance(a,dict) and isinstance(z,dict):
  return sum([diff(a.get(k),z.get(k),p+'/'+k) for k in set(a)|set(z)],[])
 if isinstance(a,list) and isinstance(z,list) and len(a)==len(z):return sum([diff(x,y,p+'/'+str(i)) for i,(x,y) in enumerate(zip(a,z))],[])
 return [] if a==z else [(p,a,z)]
initial=load(r['initial_scene_file']);out={}
try:
 for clearing in [False,True]:
  clear();apply(ready(initial))
  for num in [7,8,14,15]:
   step=r['steps'][num-1];source=load(step['scene_after_file'] if num in [7,14] else step['scene_before_file'])
   if clearing:clear()
   apply(ready(source));live=scene()
   key=f'clear{clearing}_step{num}'
   out[key]=dict(raw=air_scene.scene_fingerprint(source),live=air_scene.scene_fingerprint(live),diff=diff(canonical(source),canonical(live))[:20])
   (b/(key+'.json')).write_text(json.dumps(message_to_ordereddict(live),indent=2))
  if clearing:clear()
  apply(ready(initial));live=scene()
  out[f'clear{clearing}_restore']=dict(raw=air_scene.scene_fingerprint(initial),live=air_scene.scene_fingerprint(live),diff=diff(canonical(initial),canonical(live))[:20])
finally:
 clear();apply(ready(initial));n.destroy_node();rclpy.shutdown()
(b/'physical_scene_roundtrip_result.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
