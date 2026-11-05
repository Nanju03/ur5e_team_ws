"""Sensor freshness and mutation-sensitive scene digest regressions."""
import copy
import math
from types import SimpleNamespace

import pytest
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict
from shape_msgs.msg import Mesh, MeshTriangle

from ur5e_block_bringup import physical_stack as physical


def feedback(sequence, age=.02):
    return DynamicJointState(joint_names=[physical.HAND], interface_values=[InterfaceValue(
        interface_names=['position','raw_position','grip_state','force_percent','communication_age','sample_sequence'],
        values=[.72261,223.,2.,20.,age,float(sequence)])])


def sensor_task():
    return SimpleNamespace(sensor={},sensor_sequence=-1,sensor_received=-math.inf,mock=False)


def test_duplicate_feedback_never_refreshes_an_old_hardware_sample(monkeypatch):
    clock=[1.]
    monkeypatch.setattr(physical.time,'monotonic',lambda:clock[0])
    task=sensor_task()
    physical.PhysicalStackTask.sensor_feedback(task,feedback(10))
    assert physical.PhysicalStackTask.sensor_fresh(task)
    original=task.sensor_received
    clock[0]=1.51
    physical.PhysicalStackTask.sensor_feedback(task,feedback(10,age=.01))
    assert task.sensor_received==original
    assert not physical.PhysicalStackTask.sensor_fresh(task)
    details=physical.PhysicalStackTask.sensor_stale_details(task)
    assert details['accepted_sample_age_seconds']>.5
    assert details['callback_gap_seconds']==0.
    assert details['last_rejection']=='sequence_not_advanced'
    physical.PhysicalStackTask.sensor_feedback(task,feedback(11))
    assert physical.PhysicalStackTask.sensor_fresh(task,after=10)


def test_late_or_invalid_device_age_is_not_accepted(monkeypatch):
    monkeypatch.setattr(physical.time,'monotonic',lambda:10.)
    task=sensor_task()
    physical.PhysicalStackTask.sensor_feedback(task,feedback(10,age=.501))
    assert task.sensor_sequence==-1
    assert not physical.PhysicalStackTask.sensor_fresh(task)
    assert task.sensor_rejection=='invalid_fields_or_reported_age'


def mesh_scene():
    scene=PlanningScene()
    obj=CollisionObject(id='plate')
    obj.header.frame_id='world'
    obj.pose.orientation.w=1.
    obj.meshes=[Mesh(vertices=[Point(x=0.,y=0.,z=0.),Point(x=1.,y=0.,z=0.),Point(x=0.,y=1.,z=0.)],
                     triangles=[MeshTriangle(vertex_indices=[0,1,2])])]
    pose=Pose();pose.orientation.w=1.
    obj.mesh_poses=[pose]
    scene.world.collision_objects=[obj]
    return scene


def test_mesh_work_cache_does_not_ignore_geometry_or_pose_changes():
    scene=mesh_scene()
    physical.normalized_mesh_digest.cache_clear()
    original=physical.scene_fingerprint(scene)
    assert physical.scene_fingerprint(copy.deepcopy(scene))==original
    assert physical.normalized_mesh_digest.cache_info().hits>=1
    assert physical.scene_fingerprint(message_to_ordereddict(scene))==original
    scene.world.collision_objects[0].meshes[0].vertices[1].x+=.000001
    assert physical.scene_fingerprint(scene)!=original
    scene.world.collision_objects[0].meshes[0].vertices[1].x=1.
    assert physical.scene_fingerprint(scene)==original
    scene.world.collision_objects[0].pose.position.z+=.000001
    assert physical.scene_fingerprint(scene)!=original


def test_mesh_cache_retains_round12_equivalence():
    scene=mesh_scene()
    original=physical.scene_fingerprint(scene)
    scene.world.collision_objects[0].meshes[0].vertices[1].x+=1e-14
    assert physical.scene_fingerprint(scene)==original
    scene.world.collision_objects[0].meshes[0].triangles[0].vertex_indices=[0,2,1]
    assert physical.scene_fingerprint(scene)!=original
