"""Read-only empty acknowledgement must never move hardware or erase geometry."""
import copy
import json
from types import SimpleNamespace, MethodType
from unittest.mock import Mock

import pytest
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from ur5e_block_bringup import physical_recovery as recovery
from ur5e_block_bringup import physical_held as held
from test_stack_gui_recovery import launch_gui, ui


def task(tmp_path, monkeypatch):
    monkeypatch.setattr(held, 'held_fault_path', lambda domain=None: tmp_path/'held.json')
    held.write_held_fault('Physical grasp stopped while holding a block', recovery.digest(b'model'))
    scene = PlanningScene()
    params = dict(recovery_empty_confirmed=True, preparation_id='unlock_test')
    obj = SimpleNamespace(path=tmp_path/'unused.recipe.json', model='model', sensor=dict(grip_state=1,position=0.),
        values=dict.fromkeys(recovery.JOINTS,0.), has_parameter=lambda n:n in params,param=params.get,
        spin_until=Mock(),fresh=lambda:True,sensor_fresh=lambda:True,measured_stop=lambda:True,
        check_stop=Mock(),check_model=Mock(),scene=Mock(return_value=scene),phase=Mock(),
        run_motion=Mock(side_effect=AssertionError('No robot motion')),
        apply_snapshot=Mock(side_effect=AssertionError('No scene writes')),recovery_finished=False)
    obj.require_empty_unlock=MethodType(recovery.PhysicalRecoveryTask.require_empty_unlock,obj)
    return obj,params,scene


def test_verified_empty_acknowledgement_clears_only_fault_and_keeps_audit(tmp_path,monkeypatch):
    obj,_,scene=task(tmp_path,monkeypatch)
    before=copy.deepcopy(scene)
    recovery.PhysicalRecoveryTask.acknowledge_empty(obj)
    assert held.read_held_fault() is None
    assert obj.recovery_finished and scene==before
    obj.run_motion.assert_not_called();obj.apply_snapshot.assert_not_called()
    record=json.loads((tmp_path/'empty_unlock_unlock_test.json').read_text())
    assert record['previous_fault']['cause']=='Physical grasp stopped while holding a block'
    assert record['arm_motion'] is record['gripper_motion'] is False
    assert obj.phase.call_args.args==('STACK_EMPTY_UNLOCK_DONE',)


@pytest.mark.parametrize('failure',['unconfirmed','stale_joints','stale_sensor','held','drop','moving','joint_mismatch','attachment','foreign_model','pending_scene','stop','changing_scene','changing_fault'])
def test_uncertain_state_never_clears_held_fault(tmp_path,monkeypatch,failure):
    obj,params,scene=task(tmp_path,monkeypatch)
    if failure=='unconfirmed':params['recovery_empty_confirmed']=False
    elif failure=='stale_joints':obj.fresh=lambda:False
    elif failure=='stale_sensor':obj.sensor_fresh=lambda:False
    elif failure in ('held','drop'):obj.sensor['grip_state']=2 if failure=='held' else 3
    elif failure=='moving':obj.measured_stop=lambda:False
    elif failure=='joint_mismatch':obj.values[recovery.HAND]=.1
    elif failure=='attachment':
        a=AttachedCollisionObject();a.object.id='ur5e_stack/block_3'
        scene.robot_state.attached_collision_objects=[a]
    elif failure=='foreign_model':obj.model='different'
    elif failure=='pending_scene':held.write_held_fault('SCENE_RPC_PENDING: test',recovery.digest(b'model'))
    elif failure=='stop':obj.check_stop.side_effect=RuntimeError('stop')
    elif failure=='changing_scene':
        other=PlanningScene();other.world.collision_objects=[CollisionObject(id='changed')]
        obj.scene.side_effect=[scene,other]
    elif failure=='changing_fault':
        def changed():
            held.write_held_fault('Newer failure',recovery.digest(b'model'))
            return scene
        obj.scene.side_effect=changed
    with pytest.raises(RuntimeError):recovery.PhysicalRecoveryTask.acknowledge_empty(obj)
    assert held.read_held_fault() is not None
    obj.run_motion.assert_not_called();obj.apply_snapshot.assert_not_called()


def test_unlock_button_requires_confirmation_and_never_requests_arm_execution(monkeypatch,tmp_path):
    obj=launch_gui(tmp_path)
    start=Mock(return_value=SimpleNamespace(stdout=[]))
    monkeypatch.setattr(ui.subprocess,'Popen',start)
    monkeypatch.setattr(ui.threading,'Thread',Mock())
    monkeypatch.setattr(ui.messagebox,'askokcancel',Mock(return_value=False))
    obj.launch_task('unlock_empty');start.assert_not_called()
    monkeypatch.setattr(ui.messagebox,'askokcancel',Mock(return_value=True))
    obj.launch_task('unlock_empty')
    command=start.call_args.args[0]
    assert 'stack_mode:=physical_unlock' in command
    assert 'recovery_empty_confirmed:=true' in command
    assert not any(s.startswith('execution_mode:=') for s in command)
    assert f'preparation_id:={obj.job_plan_id}' in command
    assert obj.job_plan_id.startswith('unlock_')
    assert 'unlock_empty' not in obj.MOTION_ACTIONS


@pytest.mark.parametrize('change',[{}, {'plan_id':'old'}, {'arm_motion':True},{'scene_preserved':False}, {'policy':'old'}])
def test_gui_requires_its_exact_read_only_receipt(tmp_path,change):
    obj=launch_gui(tmp_path)
    obj.job_action='unlock_empty';obj.job=object();obj.job_plan_id='unlock_current';obj.recovery_verified=False
    payload=dict(plan_id='unlock_current',policy=held.EMPTY_UNLOCK_POLICY,gripper_empty=True,
                 fault_cleared=True,arm_motion=False,gripper_motion=False,scene_preserved=True)
    payload.update(change)
    obj.receive_empty_unlock_done(payload)
    assert obj.recovery_verified is (not change)
    assert obj.job_failed is bool(change)
