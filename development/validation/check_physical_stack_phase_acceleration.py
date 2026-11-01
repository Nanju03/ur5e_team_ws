"""Inspect a sealed phase-acceleration plan offline before any mock replay."""
import json
import os
from pathlib import Path
from types import SimpleNamespace, MethodType
import xml.etree.ElementTree as ET

from moveit_msgs.msg import RobotTrajectory
from rclpy.serialization import deserialize_message
from ur5e_block_bringup.physical_stack import validate_recipe, RECIPE_POLICY, arm_execution_profile, PhysicalStackTask, JOINTS
from ur5e_block_bringup.model import load_moveit_settings

base=Path('/home/ksn/ur5e_ws/validation')
path=Path(os.environ.get('PHYSICAL_STACK_CHECK_RECIPE',str(base/'physical_stack_phase_acceleration_one.recipe.json')))
recipe=json.loads(path.read_text())
metadata=json.loads(path.with_name(path.name+'.meta.json').read_text())
validate_recipe(recipe)
assert metadata['recipe_policy']==RECIPE_POLICY
assert metadata['arm_execution_profile']==recipe['arm_execution_profile']==arm_execution_profile()
settings=load_moveit_settings('/home/ksn/ur5e_ws/robot_calibration/ur5e_20261031.yaml','/home/ksn/ur5e_ws/physical_setup.yaml')
tree=ET.fromstring(settings['robot_description'])
bounds={name:(float(tree.find(f"joint[@name='{name}']/limit").get('lower')),
              float(tree.find(f"joint[@name='{name}']/limit").get('upper'))) for name in JOINTS}
sampler=SimpleNamespace(bounds=bounds)
sampler.motion_samples=MethodType(PhysicalStackTask.motion_samples,sampler)
assert recipe['approach_acceleration_scaling']==.05 and recipe['transfer_acceleration_scaling']==.20
assert recipe['approach_velocity_scaling']==.05 and recipe['transfer_velocity_scaling']==.20
assert len(recipe['steps'])==18 and metadata['motion_steps']==12
assert metadata['scene_sha256']==recipe['steps'][0]['scene_before_sha256']
phases, geometric_paths, hand_targets = {}, [], []
expected_checked,transfer_steps=0,0
for index,step in enumerate(recipe['steps']):
    assert len(step['start_joints'])==len(step['end_joints'])==7
    if index:
        previous=recipe['steps'][index-1]
        assert previous['scene_after_sha256']==step['scene_before_sha256']
        assert max(abs(a-b) for a,b in zip(previous['end_joints'],step['start_joints']))<1e-8
    if step['kind']=='scene':
        continue
    assert step['scene_before_sha256']==step['scene_after_sha256']
    trajectory=deserialize_message((base/step['file']).read_bytes(),RobotTrajectory)
    points=trajectory.joint_trajectory.points
    for point in (points[0],points[-1]):
        assert point.velocities and point.accelerations
        assert max(abs(v) for v in point.velocities+point.accelerations)<1e-8
    if step['kind']=='arm':
        expected=.05 if step['phase']=='approach' else .20
        assert step['velocity_scaling']==step['acceleration_scaling']==expected
        phases[step['phase']]=dict(velocity=expected,acceleration=expected)
        geometric_paths.append([list(point.positions) for point in points])
        nominal=sampler.motion_samples(step,trajectory)
        expanded=PhysicalStackTask.tracking_motion_samples(sampler,step,trajectory)
        assert len(expanded)==len(nominal)*(5 if step['phase']=='transfer' else 1)
        if step['phase']=='transfer':
            transfer_steps+=1
            offsets=[sample[1][5]-nominal[0][1][5] for sample in expanded[:5]]
            assert max(abs(a-b) for a,b in zip(offsets,[-.01,-.005,0.,.005,.01]))<1e-9
        expected_checked+=len(expanded)
    else:
        hand_targets.append(step['hand_target'])
        expected_checked+=len(sampler.motion_samples(step,trajectory))
assert hand_targets==[.4,.73,.70,.60]
assert expected_checked==metadata['validated_samples']
old=json.loads((base/'cache_benchmark_before.recipe.json').read_text())
old_paths=[]
for step in old['steps']:
    if step['kind']=='arm':
        trajectory=deserialize_message((base/step['file']).read_bytes(),RobotTrajectory)
        old_paths.append([list(point.positions) for point in trajectory.joint_trajectory.points])
same_geometric_paths=old_paths==geometric_paths
assert same_geometric_paths
old_duration=97.298032529
new_duration=metadata['duration']
assert 0<new_duration<old_duration
report=dict(result='passed',domain=103,recipe_policy=RECIPE_POLICY,
            phases=phases,steps=18,motions=12,validated_samples=metadata['validated_samples'],
            old_duration_seconds=old_duration,new_duration_seconds=new_duration,
            time_reduction_percent=100*(old_duration-new_duration)/old_duration,
            unchanged_arm_geometric_path_points=same_geometric_paths,
            all_motion_endpoints_at_rest=True,all_seven_joint_vectors_continuous=True,
            all_scene_hashes_continuous=True,hand_targets=hand_targets,
            arm_execution_profile=arm_execution_profile(),transfer_steps_checked=transfer_steps,
            wrist3_transfer_offsets_rad=[-.01,-.005,0.,.005,.01],
            all_expanded_tracking_samples_accounted_for=expected_checked==metadata['validated_samples'],
            actual_robot_goals_sent=False)
output=Path(os.environ.get('PHYSICAL_STACK_CHECK_OUTPUT',str(base/'physical_stack_phase_acceleration_artifact_check.json')))
output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
