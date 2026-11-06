"""Measured grasp coverage and opening direction, with no hardware goals."""
from types import SimpleNamespace
from unittest.mock import Mock
from concurrent.futures import Future
import json
import pytest
from moveit_msgs.msg import PlanningScene
from ur5e_block_bringup import physical_stack as physical
from ur5e_block_bringup import physical_replay as replay
from test_physical_replay import package,held_task,trajectory,bind,SCENE

@pytest.mark.parametrize('angle',[.70401,.69936,.70587,.72075])
def test_observed_25mm_grasps_use_the_prevalidated_range_without_extra_checks(package,angle):
    task=held_task(package);task.holding_reference=angle;task.values[physical.HAND]=angle
    task.validation_worker=Mock(side_effect=AssertionError('No extra checks for covered grasps'))
    step=package[1]['steps'][4]
    task.bind_certified_motion(step,trajectory(step),SCENE)
    assert task.holding_context_verified(step,trajectory(step),SCENE)
    task.validation_worker.assert_not_called()

@pytest.mark.parametrize('angle,target',[(.72075,.70),(.70401,.69401),(.69936,.68936)])
def test_first_release_is_always_an_opening_for_the_measured_grasp(angle,target):
    task=SimpleNamespace(holding_reference=angle)
    task.holding_interval=lambda:physical.PhysicalStackTask.holding_interval(task)
    result=physical.PhysicalStackTask.first_release_target(task)
    assert result==pytest.approx(target)
    assert .60<=result<task.holding_interval()[0]
    step=dict(kind='hand',phase='release_first',start_joints=[0.]*6+[.73],hand_target=.70)
    lo,hi=replay.finger_range(step,True,(0.,.93))
    assert lo<=result-.003<=result+.003<=hi


def extended_task(package,monkeypatch):
    task=held_task(package);step=package[1]['steps'][4];motion=trajectory(step)
    task.holding_reference=.694;task.values=dict(zip(physical.JOINTS,step['start_joints']));task.values[physical.HAND]=.694
    task.check_stop=Mock();task.fresh=lambda:True;task.sensor_fresh=lambda:True
    task.read_message=lambda *args:PlanningScene();task.replay_queries=None
    worker=SimpleNamespace(poll=lambda:None)
    def validate(scene,samples):
        worker.poll();return len(samples)
    worker.validate=Mock(side_effect=validate)
    task.validation_worker=Mock(return_value=worker);task.release_validation_worker=Mock()
    monkeypatch.setattr(physical,'scene_fingerprint',lambda scene:SCENE)
    return task,step,motion,worker


def test_only_uncovered_fingers_are_checked_locally_and_coverage_is_reused(package,monkeypatch):
    task,step,motion,worker=extended_task(package,monkeypatch)
    task.bind_certified_motion(step,motion,SCENE)
    assert task.holding_context_verified(step,motion,SCENE)
    samples=worker.validate.call_args.args[1]
    assert min(q[-1] for _,q in samples)==pytest.approx(.689)
    assert all(q[-1]<replay.HOLDING_LOWER for _,q in samples)
    assert task.verified_holding_context['source']=='local_cpp_finger_extension'
    physical.PhysicalStackTask.extend_measured_fingers(task,step,motion,SCENE)
    assert worker.validate.call_count==1
    task.validate_samples.assert_not_called()

@pytest.mark.parametrize('failure',['collision','arm_changed','finger_changed','stale','stop','partial_reply'])
def test_extension_failure_never_marks_unchecked_path_as_executable(package,monkeypatch,failure):
    task,step,motion,worker=extended_task(package,monkeypatch)
    def validate(scene,samples):
        if failure=='collision':raise RuntimeError('collision')
        if failure=='arm_changed':task.values[physical.ARM[0]]+=.01
        if failure=='finger_changed':task.holding_reference-=.01
        if failure=='stale':task.sensor_fresh=lambda:False
        if failure=='stop':task.check_stop.side_effect=RuntimeError('stop')
        worker.poll()
        return len(samples)-1 if failure=='partial_reply' else len(samples)
    worker.validate.side_effect=validate
    with pytest.raises(RuntimeError):task.bind_certified_motion(step,motion,SCENE)
    assert task.verified_holding_context is None
    task.release_validation_worker.assert_called_once()


def test_release_boundary_accepts_only_the_verified_actual_opening_target():
    step=dict(phase='place')
    task=SimpleNamespace(release_stage=1,last_first_release_target=.68936,values={physical.HAND:.68936})
    assert physical.PhysicalStackTask.released_start_matches(task,step)
    task.values[physical.HAND]=.68
    assert not physical.PhysicalStackTask.released_start_matches(task,step)


@pytest.mark.parametrize('angle',[.70401,.69936,.72075])
def test_release_dispatch_opens_from_the_measured_grasp_and_accepts_its_endpoint(monkeypatch,angle):
    finished,accepted=Future(),Future()
    accepted.set_result(SimpleNamespace(accepted=True,get_result_async=lambda:finished))
    sent=[]
    def send(goal,**options):
        sent.append(goal.command.position)
        return accepted
    client=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=send,destroy=Mock())
    monkeypatch.setattr(physical,'ActionClient',lambda *args:client)
    monkeypatch.setattr(physical,'file_digest',lambda path:'recipe')
    task=SimpleNamespace(mock=False,prevalidated_replay=True,motion_clients={},replay_queries=[],
        replay_checkpoint=Mock(),spin_until=Mock(),sensor_sequence=1,sensor={},
        values=dict(zip(physical.JOINTS,[0.]*6+[angle])),holding_reference=angle,
        carrying=True,release_stage=0,verified_holding_context={'key':'reviewed'},
        holding_context_verified=lambda *args:True,verified_holding_ok=lambda *args:True,
        holding_ok=lambda:True,prewarm_flight_queries=Mock(),measured_stop=lambda **kwargs:True,
        input_hashes=lambda:{},path=physical.Path('/not/read'),check_stop=Mock(),phase=Mock(),
        stopped=False,poll_flight_queries=Mock(),fresh=lambda:True,sensor_fresh=lambda **kwargs:True,
        cancel_remaining_goal=Mock(),execution_mode='continuous')
    task.holding_interval=lambda:physical.PhysicalStackTask.holding_interval(task)
    target=physical.PhysicalStackTask.first_release_target(task)
    def spin(*args,**kwargs):
        task.sensor_sequence+=1;task.values[physical.HAND]=sent[0]
        finished.set_result(SimpleNamespace(status=4,result=SimpleNamespace(
            reached_goal=True,stalled=False,position=sent[0])))
    monkeypatch.setattr(physical.rclpy,'spin_once',spin)
    step=dict(index=1,kind='hand',phase='release_first',hand_target=.70,label='release',
              collision_certificate={'finger_range':[.597,.733]})
    physical.PhysicalStackTask.run_motion(task,step,physical.RobotTrajectory(),
        dict(input_sha256={},recipe_sha256='recipe'),'scene',1,1)
    assert sent==[target]
    assert task.last_first_release_target==target<angle


@pytest.mark.parametrize('needs_extension',[False,True])
def test_motion_timing_reports_only_its_added_finger_check_including_worker_startup(
        package,monkeypatch,capsys,needs_extension):
    task,step,motion,worker=extended_task(package,monkeypatch)
    clock=[10.]
    monkeypatch.setattr(physical.time,'monotonic',lambda:clock[0])
    def create_worker(bounded_contacts):
        clock[0]+=2.
        return worker
    def validate(scene,samples):
        clock[0]+=.75
        worker.poll()
        return len(samples)
    task.validation_worker.side_effect=create_worker
    worker.validate.side_effect=validate
    if not needs_extension:
        task.holding_reference=.70401;task.values[physical.HAND]=.70401
    task.bind_certified_motion(step,motion,SCENE)
    # Step-mode approval binds the same motion again; it must reuse this pass.
    task.bind_certified_motion(step,motion,SCENE)
    expected_samples=len(worker.validate.call_args.args[1]) if needs_extension else 0
    assert worker.validate.call_count==int(needs_extension)

    finished,accepted=Future(),Future()
    accepted.set_result(SimpleNamespace(accepted=True,get_result_async=lambda:finished))
    client=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=lambda *args,**kwargs:accepted)
    monkeypatch.setattr(physical,'ActionClient',lambda *args:client)
    task.mock=False;task.prevalidated_replay=True;task.motion_clients={};task.replay_queries=[]
    task.replay_checkpoint=Mock();task.spin_until=Mock();task.sensor_sequence=1;task.sensor={}
    task.input_hashes=lambda:package[2]['input_sha256']
    task.phase=Mock();task.stopped=False;task.poll_flight_queries=Mock()
    task.cancel_remaining_goal=Mock();task.execution_mode='continuous'
    def prewarm(queries):clock[0]+=.1
    def measured_stop(**kwargs):
        clock[0]+=.3
        return True
    task.prewarm_flight_queries=prewarm;task.measured_stop=measured_stop
    def spin(*args,**kwargs):
        clock[0]+=.2
        task.values.update(dict(zip(physical.ARM,step['end_joints'][:6])))
        finished.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0,error_string='')))
    monkeypatch.setattr(physical.rclpy,'spin_once',spin)
    physical.PhysicalStackTask.run_motion(task,step,motion,package[2],SCENE,1,1)
    records=capsys.readouterr().out.splitlines()
    timing=json.loads(next(line.split(' ',1)[1] for line in records if line.startswith('STACK_REPLAY_STEP_TIMING ')))
    assert timing['collision_rechecks']==int(needs_extension)
    assert timing['additional_finger_samples']==expected_samples
    assert timing['additional_finger_validation_seconds']==(2.75 if needs_extension else 0.)
    assert timing['prepare_seconds']==pytest.approx(.4)
    assert timing['action_seconds']==pytest.approx(.2)
    assert timing['settle_seconds']==pytest.approx(.3)
    if needs_extension:
        extension=json.loads(next(line.split(' ',1)[1] for line in records if line.startswith('STACK_FINGER_EXTENSION_VALIDATED ')))
        assert extension['seconds']==timing['additional_finger_validation_seconds']
