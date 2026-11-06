"""Protocol faults and actual offline MoveIt geometry; never command a robot."""
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_prefix
from moveit_msgs.msg import AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from ur5e_block_bringup.collision_worker import CollisionWorker, MAX_STATES
from ur5e_block_bringup.physical_stack import JOINTS
from ur5e_block_bringup.model import _protect_workbench


def fake_worker(tmp_path, reply, poll=lambda: None, timeout=2.):
    script = tmp_path/'fake.py'
    script.write_text('''import sys,struct,json
print('{"ready":true,"protocol":1}',flush=True)
while True:
 h=sys.stdin.buffer.read(8)
 if not h:break
 size,count=struct.unpack('<II',h)
 data=sys.stdin.buffer.read(size+count*7*8)
 if len(data)!=size+count*7*8:break
 '''+reply+'\n')
    return CollisionWorker({}, tmp_path, poll, timeout=timeout, command=[sys.executable, str(script)])


def test_every_sample_is_acknowledged_across_bounded_batches(tmp_path):
    with fake_worker(tmp_path, 'print(json.dumps(dict(ok=True,checked=count)),flush=True)') as worker:
        assert worker.validate(PlanningScene(), [(0., [0.]*7)]*(MAX_STATES+3)) == MAX_STATES+3
    assert worker.process.poll() == 0


def test_concurrent_workers_do_not_share_parameter_or_error_files(tmp_path):
    reply='print(json.dumps(dict(ok=True,checked=count)),flush=True)'
    with fake_worker(tmp_path, reply) as first, fake_worker(tmp_path, reply) as second:
        assert first.parameters_path != second.parameters_path
        assert first.stderr_path != second.stderr_path
        assert first.validate(PlanningScene(), [(0., [0.]*7)]) == 1
        assert second.validate(PlanningScene(), [(0., [0.]*7)]) == 1


@pytest.mark.parametrize('reply,reason', [
    ('print(json.dumps(dict(ok=False,invalid_index=0)),flush=True)', '충돌 검사 실패'),
    ('print(json.dumps(dict(ok=True,checked=count-1)),flush=True)', 'every sample'),
    ('print("invalid-json",flush=True)', 'Malformed'),
    ('sys.exit(2)', 'exited'),
])
def test_worker_failure_never_becomes_a_success(tmp_path, reply, reason):
    with fake_worker(tmp_path, reply) as worker:
        with pytest.raises(RuntimeError, match=reason):
            worker.validate(PlanningScene(), [(0., [0.]*7)])


def test_stop_terminates_pending_local_work(tmp_path):
    stopped = False
    def poll():
        if stopped:
            raise RuntimeError('operator stop')
    with fake_worker(tmp_path, 'import time;time.sleep(10)', poll) as worker:
        stopped = True
        with pytest.raises(RuntimeError, match='operator stop'):
            worker.validate(PlanningScene(), [(0., [0.]*7)])
    assert worker.process.poll() is not None


def test_timeout_cannot_block_the_next_plan(tmp_path):
    with fake_worker(tmp_path, 'import time;time.sleep(10)') as worker:
        worker.timeout = .1
        with pytest.raises(TimeoutError):
            worker.validate(PlanningScene(), [(0., [0.]*7)])
    assert worker.process.poll() is not None


@pytest.fixture
def actual_model():
    root = Path(__file__).resolve().parents[2]
    urdf = (root/'ur5e_block_description/urdf/ur5e_on_box.urdf').read_text()
    semantic = (root/'ur5e_ag95_moveit_config/config/ur5e_on_box.srdf').read_text()
    semantic = _protect_workbench(urdf, semantic)
    home = {j.get('name'): float(j.get('value'))
            for j in ET.fromstring(semantic).find("group_state[@name='home']").findall('joint')}
    values = [home.get(j, .4) for j in JOINTS]
    scene = PlanningScene()
    scene.robot_state.joint_state.name = JOINTS
    scene.robot_state.joint_state.position = values
    # A full message replaces the ACM, so include the actual SRDF exclusions.
    links = [link.get('name') for link in ET.fromstring(urdf).findall('link')]
    disabled = {frozenset((c.get('link1'), c.get('link2')))
                for c in ET.fromstring(semantic).findall('disable_collisions')}
    scene.allowed_collision_matrix.entry_names = links
    scene.allowed_collision_matrix.entry_values = [AllowedCollisionEntry(
        enabled=[a == b or frozenset((a, b)) in disabled for b in links]) for a in links]
    return dict(robot_description=urdf, robot_description_semantic=semantic), scene, values


def giant_box():
    obj = CollisionObject()
    obj.header.frame_id = 'world'
    obj.id = 'offline_test_obstacle'
    obj.pose.orientation.w = 1.
    shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[10., 10., 10.])
    obj.primitives = [shape]
    return obj


def test_actual_checker_checks_world_attached_geometry_acm_and_bounds(tmp_path, actual_model):
    executable = Path(get_package_prefix('ur5e_a3_grid'))/'lib/ur5e_a3_grid/physical_collision_checker'
    assert executable.is_file(), 'Build ur5e_a3_grid before running the offline checker integration test'
    parameters, scene, values = actual_model
    with CollisionWorker(parameters, tmp_path, lambda: None) as worker:
        assert worker.validate(scene, [(0., values)]) == 1
        world = copy.deepcopy(scene)
        world.world.collision_objects = [giant_box()]
        with pytest.raises(RuntimeError, match='충돌 검사 실패'):
            worker.validate(world, [(0., values)])
        # Change the full snapshot after a failure: no stale scene/ACM reuse.
        world.allowed_collision_matrix.default_entry_names = ['offline_test_obstacle']
        world.allowed_collision_matrix.default_entry_values = [True]
        assert worker.validate(world, [(0., values)]) == 1
        assert worker.validate(scene, [(0., values)]) == 1
        attached = copy.deepcopy(scene)
        body = AttachedCollisionObject(link_name='tool0', object=giant_box())
        body.object.header.frame_id = 'tool0'
        attached.robot_state.attached_collision_objects = [body]
        with pytest.raises(RuntimeError, match='충돌 검사 실패'):
            worker.validate(attached, [(0., values)])
        bad = list(values);bad[0] = 100.
        with pytest.raises(RuntimeError, match='충돌 검사 실패'):
            worker.validate(scene, [(0., bad)])


def test_bounded_carried_surface_contact_rejects_penetration_hidden_by_acm(tmp_path, actual_model):
    parameters, scene, values = actual_model
    # Isolate two 25mm cubes well above the robot. Their allowed support pair
    # must allow exact contact and reject 1mm penetration in continuous mode.
    carried = giant_box()
    carried.id = 'ur5e_stack/block_99'
    carried.primitives[0].dimensions = [.025]*3
    carried.pose.position.z = 5.
    support = copy.deepcopy(carried)
    support.id = 'offline_support'
    support.pose.position.z = 4.975
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(link_name='base_link', object=carried, touch_links=['base_link'])]
    scene.world.collision_objects = [support]
    matrix = scene.allowed_collision_matrix
    for name in (carried.id, support.id):
        for row in matrix.entry_values:
            row.enabled.append(False)
        matrix.entry_names.append(name)
        matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
    a, b = [matrix.entry_names.index(name) for name in (carried.id,support.id)]
    matrix.entry_values[a].enabled[b] = matrix.entry_values[b].enabled[a] = True
    penetrating = copy.deepcopy(scene)
    penetrating.world.collision_objects[0].pose.position.z += .001
    with CollisionWorker(parameters,tmp_path,lambda:None) as unbounded:
        assert unbounded.validate(penetrating,[(0.,values)]) == 1
    with CollisionWorker(dict(parameters,bounded_surface_contacts=True),tmp_path,lambda:None) as bounded:
        assert bounded.validate(scene,[(0.,values)]) == 1
        with pytest.raises(RuntimeError,match='충돌 검사 실패'):
            bounded.validate(penetrating,[(0.,values)])
        assert bounded.validate(scene,[(0.,values)]) == 1


def test_snapshot_and_sample_shape_are_rejected_before_ipc(tmp_path):
    with fake_worker(tmp_path, 'print(json.dumps(dict(ok=True,checked=count)),flush=True)') as worker:
        diff = PlanningScene(is_diff=True)
        with pytest.raises(ValueError, match='full snapshot'):
            worker.validate(diff, [(0., [0.]*7)])
        with pytest.raises(ValueError, match='seven finite'):
            worker.validate(PlanningScene(), [(0., [float('nan')]*7)])
