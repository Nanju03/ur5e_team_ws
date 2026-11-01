"""Compare both sealed plan artifacts offline; never initialize a ROS node."""
import importlib.util
import json
from pathlib import Path
import sys

from moveit_msgs.msg import RobotTrajectory
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict

base = Path('/home/ksn/ur5e_ws/validation')
spec = importlib.util.spec_from_file_location('ur5e_block_bringup._benchmark_reference',base/'physical_stack_before_cache.py')
reference = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = reference
spec.loader.exec_module(reference)

def check(variant):
    path = base/f'cache_benchmark_{variant}.recipe.json'
    recipe = json.loads(path.read_text())
    metadata = json.loads(path.with_name(path.name+'.meta.json').read_text())
    steps = recipe['steps']
    assert metadata['validated_samples'] == 2170
    assert len(steps) == 18 and metadata['motion_steps'] == 12
    assert metadata['scene_sha256'] == steps[0]['scene_before_sha256']
    trajectories, sequence = [], []
    for index, step in enumerate(steps):
        if index:
            assert steps[index-1]['scene_after_sha256'] == step['scene_before_sha256']
            assert max(abs(a-b) for a,b in zip(steps[index-1]['end_joints'],step['start_joints'])) < 1e-8
        assert len(step['start_joints']) == len(step['end_joints']) == 7
        if step['kind'] != 'scene':
            assert step['scene_before_sha256'] == step['scene_after_sha256']
            trajectory = deserialize_message((base/step['file']).read_bytes(),RobotTrajectory)
            data = message_to_ordereddict(trajectory)
            data['joint_trajectory'].pop('header',None)
            data['multi_dof_joint_trajectory'].pop('header',None)
            trajectories.append(data)
        if step['kind'] == 'arm':
            assert step['velocity_scaling'] == (.05 if step['phase'] == 'approach' else .20)
        sequence.append({key:step[key] for key in ('kind','phase','label','start_joints','end_joints','scene_before_sha256','scene_after_sha256')})
    assert [step['hand_target'] for step in steps if step['kind']=='hand'] == [.4,.73,.70,.60]
    return dict(metadata=metadata,recipe=recipe,trajectories=trajectories,sequence=sequence)

before, after = check('before'),check('after')
assert before['trajectories'] == after['trajectories']
assert before['sequence'] == after['sequence']
assert before['metadata']['duration'] == after['metadata']['duration']
old_timing=json.loads((base/'physical_stack_cache_benchmark_before_cache.timing.json').read_text())
new_timing=json.loads((base/'physical_stack_cache_benchmark_after_cache.timing.json').read_text())
report=dict(result='passed',domain=103,mode='physical_plan',hardware='mock_components/GenericSystem',
            movement_goals_sent=0,steps=18,motion_steps=12,validated_samples_each=2170,
            identical_all_motion_trajectory_points=True,identical_all_step_joint_vectors=True,
            identical_initial_and_all_scene_hashes=True,all_adjacent_scene_hashes_continuous=True,
            every_motion_keeps_collision_geometry=True,gripper_targets=[.4,.73,.70,.60],
            approach_scaling=.05,other_arm_scaling=.20,
            old_seconds=old_timing['elapsed_seconds'],new_seconds=new_timing['elapsed_seconds'],
            measured_speedup=old_timing['elapsed_seconds']/new_timing['elapsed_seconds'],
            old_collision_clients_created=sum(old_timing['clients_created'][name] for name in ('/get_planning_scene','/apply_planning_scene','/check_state_validity')),
            new_collision_clients_created=sum(new_timing['clients_created'][name] for name in ('/get_planning_scene','/apply_planning_scene','/check_state_validity')),
            note='Isolated domain103 did not reproduce actual-domain cold discovery stalls; the measured timing is not a forecast of actual-domain planning time.')
(base/'physical_stack_cache_benchmark_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
