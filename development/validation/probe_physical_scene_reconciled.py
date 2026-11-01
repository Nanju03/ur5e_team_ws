import json,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.msg import PlanningScene
from ur5e_block_bringup.air_task import AirTask
from ur5e_block_bringup.physical_stack import PhysicalStackTask,scene_fingerprint,SCENE_HASH_POLICY
assert os.environ.get('ROS_DOMAIN_ID')=='103'
base=Path('/home/ksn/ur5e_ws/validation')
recipe=json.loads((base/'physical_stack_one.recipe.json').read_text())
class Bridge(Node):
 spin_until=AirTask.spin_until
 call=AirTask.call
 scene=AirTask.scene
 apply_snapshot=PhysicalStackTask.apply_snapshot
 read_scene_for_reconciliation=PhysicalStackTask.read_scene_for_reconciliation
 apply_scene_message=PhysicalStackTask.apply_scene_message
 def check_stop(self): pass
rclpy.init();node=Bridge('physical_scene_reconciled_mock_only')
node.values=dict(zip(recipe['joint_names'],[float(q) for q in recipe['initial_joints']]))
node.scene_request_pending=False;node.model='<robot/>'
hardware=node.call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
assert hardware.component and all(item.class_type=='mock_components/GenericSystem' for item in hardware.component)
def load(name):return deserialize_message((base/name).read_bytes(),PlanningScene)
initial=load(recipe['initial_scene_file']);result={'policy':SCENE_HASH_POLICY,'steps':[]}
try:
 prior=None
 for step in recipe['steps']:
  node.apply_snapshot(load(step['scene_before_file']))
  before=scene_fingerprint(node.scene())
  node.apply_snapshot(load(step['scene_after_file']))
  after=scene_fingerprint(node.scene())
  entry=dict(index=step['index'],kind=step['kind'],before=before,after=after,chain=(prior is None or prior==before),motion_unchanged=(step['kind']=='scene' or before==after))
  result['steps'].append(entry);prior=after
  print(json.dumps(entry),flush=True)
finally:
 node.apply_snapshot(initial,cancel=False)
 result['initial_restored']=scene_fingerprint(node.scene())
 result['restore_matches_first']=bool(result['steps']) and result['initial_restored']==result['steps'][0]['before']
 node.destroy_node();rclpy.shutdown()
result['pass']=all(step['chain'] and step['motion_unchanged'] for step in result['steps']) and result['restore_matches_first']
(base/'physical_scene_reconciliation_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'pass':result['pass'],'steps':len(result['steps']),'restore_matches_first':result['restore_matches_first']}),flush=True)
assert result['pass']
