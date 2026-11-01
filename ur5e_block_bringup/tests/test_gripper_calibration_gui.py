"""Calibration callbacks with fake widgets/backend; no Tk display or ROS Node."""
import ast
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import MagicMock, patch
from ur5e_block_bringup.plate_geometry import ALIGNMENT_GRIPPER_POSITION

BASE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('calibration_ui', BASE/'ur5e_block_bringup/ur5e_block_bringup/gripper_calibration_gui.py')
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)

class Var:
    def __init__(self, value=''): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value

class Widget:
    def __init__(self): self.options = {}
    def configure(self, **kwargs): self.options.update(kwargs)

class Window:
    def __init__(self): self.destroyed = False; self.scheduled = []
    def after(self, delay, callback): self.scheduled.append((delay, callback))
    def destroy(self): self.destroyed = True

READY = dict(phase='IDLE', ready=True, busy=False, controller_active=True,
             arm_stationary=True, feedback_valid=True, age=.05, can_record=False,
             arm_required=True, gripper_only=False, arm_feedback_present=True,
             can_close=False, stop_confirmed=False, force_supported=True,
             q_rad=.5, raw_position=462, force_percent=20, model_gap_mm=50,
             grip_label='진행 중', recommendation=None)

class Backend:
    def __init__(self): self.state = copy.deepcopy(READY); self.calls = []; self.close_result = False
    def poll(self): return copy.deepcopy(self.state)
    def configure(self, **kwargs): self.calls.append(('configure', kwargs))
    def start(self, mode, workspace_clear): self.calls.append(('start', mode, workspace_clear)); self.state['busy'] = True
    def record_result(self, success): self.calls.append(('record', success)); self.state['can_record'] = False
    def apply_recommendation(self): self.calls.append(('apply',)); return '/tmp/calibration.yaml'
    def request_stop(self): self.calls.append(('stop',))
    def approve_next(self, request_id):
        self.calls.append(('approve', request_id))
        self.state.update(pending_approval=None, phase='GOAL')
    def close(self): self.calls.append(('close',)); return self.close_result

def make_ui():
    obj = ui.GripperCalibrationWindow.__new__(ui.GripperCalibrationWindow)
    obj.backend = Backend(); obj.state = copy.deepcopy(READY)
    obj.gripper_only = False
    obj.execution_mode = 'continuous'
    obj.closed = obj.close_requested = obj.confirming = obj.command_pending = obj.stop_pending = obj.poll_failed = False
    obj.configuration_loaded = False; obj.last_error = ''
    obj.window = Window(); obj.invalidations = []; obj.statuses = []; obj.closes = []
    obj.invalidate_plan = lambda: obj.invalidations.append(True)
    obj.on_status = obj.statuses.append; obj.on_close = lambda: obj.closes.append(True)
    for name, value in [('open_q', '.50'), ('max_q', '.78'), ('step_q', '.01'), ('force_percent', '20'), ('settle_s', '.4')]:
        setattr(obj, name, Var(value))
    for name in ['phase_text', 'feedback_text', 'readiness_text', 'csv_text', 'recommendation_text', 'notice']:
        setattr(obj, name, Var())
    obj.entries = [Widget(), Widget()]; obj.force_entry = obj.entries[-1]
    obj.motion_buttons = {key: Widget() for key in ['baseline', 'grasp', 'step_close', 'open']}
    for name in ['success_button', 'failure_button', 'apply_button', 'stop_button', 'close_button', 'approve_button']:
        setattr(obj, name, Widget())
    return obj

class CalibrationUiTests(unittest.TestCase):
    def waiting_step(self):
        obj=make_ui(); obj.execution_mode='step'
        preview=dict(request_id='current-token',index=2,total=30,operation='baseline',
                     start_q=.5,target_q=.51,start_gap_mm=50.,target_gap_mm=48.)
        obj.state.update(phase='WAIT_APPROVAL',busy=True,ready=False,pending_approval=preview)
        obj.backend.state=copy.deepcopy(obj.state)
        return obj

    def test_step_target_preview_is_visible_and_only_current_ok_approves(self):
        obj=self.waiting_step(); obj._render()
        self.assertIn('0.500 → 0.510rad',obj.feedback_text.get())
        self.assertIn('50.0 → 48.0mm (모델 추정)',obj.feedback_text.get())
        self.assertEqual(obj.approve_button.options['state'],'normal')
        with patch.object(ui.messagebox,'askokcancel',return_value=True) as modal:
            obj.approve_next(); obj.approve_next()
        self.assertEqual(obj.backend.calls,[('approve','current-token')])
        self.assertIn('팔의 이동 경로가 아닌',modal.call_args.args[1])

    def test_cancelled_step_confirmation_never_approves(self):
        obj=self.waiting_step()
        with patch.object(ui.messagebox,'askokcancel',return_value=False):obj.approve_next()
        self.assertEqual(obj.backend.calls,[])

    def test_step_changed_during_confirmation_is_not_approved(self):
        obj=self.waiting_step()
        def changed(*args,**kwargs):
            obj.backend.state['pending_approval']['request_id']='new-token'
            return True
        with patch.object(ui.messagebox,'askokcancel',side_effect=changed):obj.approve_next()
        self.assertEqual(obj.backend.calls,[])
        self.assertIn('바뀌었습니다',obj.last_error)

    def test_step_ok_disabled_after_stop_or_stale_feedback(self):
        obj=self.waiting_step();obj.stop_pending=True
        self.assertFalse(obj._can_approve())
        obj.stop_pending=False;obj.state['age']=.6
        self.assertFalse(obj._can_approve())

    def test_continuous_confirmation_explains_recipe_runs_automatically(self):
        obj=make_ui()
        with patch.object(ui.messagebox,'askokcancel',return_value=False) as modal:obj.start('baseline')
        self.assertIn('전체 연속 실행',modal.call_args.args[1])
        self.assertIn('개폐 단계를 자동으로 진행',modal.call_args.args[1])

    def test_constructor_preserves_legacy_factory_and_explicit_single_gripper(self):
        for gripper_only in (False, True):
            fake_window = MagicMock()
            factory = MagicMock(return_value=Backend())
            with patch.object(ui.tk, 'Toplevel', return_value=fake_window), \
                    patch.object(ui.tk, 'StringVar', side_effect=lambda *args, **kwargs: Var(kwargs.get('value', ''))), \
                    patch.object(ui.GripperCalibrationWindow, '_build'):
                obj = ui.GripperCalibrationWindow(object(), 'calibration', 'physical',
                    on_close=lambda: None, invalidate_plan=lambda: None,
                    backend_factory=factory, gripper_only=gripper_only)
            expected = {'allow_mock': False, 'execution_mode':'step'}
            if gripper_only: expected['gripper_only'] = True
            factory.assert_called_once_with('calibration', 'physical', **expected)
            if gripper_only:
                self.assertIn('그리퍼 단독 모드', obj.notice.get())
                self.assertNotIn('팔을 수동', obj.notice.get())

    def test_gripper_only_ready_without_arm_and_records_sensor_grasp(self):
        obj = make_ui(); obj.gripper_only = True
        obj.state.update(gripper_only=True, arm_required=False, arm_feedback_present=False,
                         arm_stationary=False, can_record=True)
        obj.backend.state = copy.deepcopy(obj.state)
        self.assertTrue(obj._ready_for_motion()); self.assertTrue(obj._can_record())
        obj._render(); self.assertIn('그리퍼 단독 모드 · 팔 피드백 불필요', obj.readiness_text.get())
        with patch.object(ui.messagebox, 'askyesno', return_value=True): obj.record_result(True)
        self.assertEqual(obj.backend.calls, [('record', True)])

    def test_gripper_only_does_not_bypass_backend_ready_or_sensor_checks(self):
        obj = make_ui(); obj.gripper_only = True
        for change in [dict(ready=False), dict(age=.6), dict(feedback_valid=False), dict(controller_active=False)]:
            obj.state = dict(READY, gripper_only=True, arm_required=False, arm_stationary=False, **change)
            self.assertFalse(obj._ready_for_motion())

    def test_mode_mismatch_never_bypasses_default_arm_condition(self):
        obj = make_ui(); obj.state.update(gripper_only=True, arm_required=False, arm_stationary=False, can_record=True)
        self.assertFalse(obj._ready_for_motion()); self.assertFalse(obj._can_record())
        obj.gripper_only = True; obj.state.update(gripper_only=False)
        self.assertFalse(obj._ready_for_motion())

    def test_gripper_only_motion_confirmation_mentions_fixing_and_power_cutoff(self):
        obj = make_ui(); obj.gripper_only = True
        obj.state.update(gripper_only=True, arm_required=False, arm_stationary=False)
        with patch.object(ui.messagebox, 'askokcancel', return_value=False) as modal: obj.start('grasp')
        text = modal.call_args.args[1]
        self.assertIn('그리퍼를 단단히 고정', text)
        self.assertIn('비상 전원 차단', text)
        self.assertNotIn('팔이 멈춰', text)
        self.assertEqual(obj.backend.calls, [])

    def test_fresh_ready_is_required(self):
        obj = make_ui(); self.assertTrue(obj._ready_for_motion())
        for change in [dict(age=.6), dict(age=float('nan')), dict(arm_stationary=False), dict(feedback_valid=False), dict(controller_active=False), dict(ready=False), dict(busy=True)]:
            obj.state = dict(READY, **change); self.assertFalse(obj._ready_for_motion(), change)

    def test_force_locked_until_supported(self):
        obj = make_ui(); obj.state['force_supported'] = False; obj._render()
        self.assertEqual(obj.force_entry.options['state'], 'disabled')
        self.assertEqual(obj.stop_button.options['state'], 'normal')

    def test_invalid_force_rejected(self):
        obj = make_ui()
        for value in ['19', '101', '20.5', 'nan']:
            obj.force_percent.set(value)
            with self.assertRaises(ValueError): obj._configuration()

    def test_gui_limits_match_calibration_backend(self):
        for name, value in [('max_q', '.86'), ('step_q', '.04'), ('step_q', '.0005'), ('settle_s', '.25'), ('settle_s', '2.1')]:
            obj = make_ui(); getattr(obj, name).set(value)
            with self.assertRaises(ValueError, msg=f'{name}={value}'): obj._configuration()

    def test_cancelled_modal_never_dispatches(self):
        obj = make_ui()
        with patch.object(ui.messagebox, 'askokcancel', return_value=False): obj.start('grasp')
        self.assertEqual(obj.backend.calls, [])

    def test_confirmed_motion_has_configuration_and_preparation_flag(self):
        obj = make_ui()
        with patch.object(ui.messagebox, 'askokcancel', return_value=True) as modal: obj.start('grasp')
        self.assertEqual(obj.backend.calls[-1], ('start', 'grasp', True))
        self.assertEqual(obj.backend.calls[0][1]['force_percent'], 20)
        self.assertIn('자동으로 열지 않습니다', modal.call_args.args[1])
        self.assertTrue(obj.invalidations)

    def test_modal_has_baseline_block_absence_instruction(self):
        obj = make_ui()
        with patch.object(ui.messagebox, 'askokcancel', return_value=False) as modal: obj.start('baseline')
        self.assertIn('블록이 없는 빈 공간', modal.call_args.args[1])

    def test_state_change_during_modal_blocks_motion(self):
        obj = make_ui()
        def changed(*args, **kwargs): obj.backend.state['arm_stationary'] = False; return True
        with patch.object(ui.messagebox, 'askokcancel', side_effect=changed): obj.start('grasp')
        self.assertFalse(any(call[0] == 'start' for call in obj.backend.calls))

    def test_configuration_change_during_modal_blocks_motion(self):
        obj = make_ui()
        def changed(*args, **kwargs): obj.max_q.set('.80'); return True
        with patch.object(ui.messagebox, 'askokcancel', side_effect=changed): obj.start('grasp')
        self.assertFalse(any(call[0] == 'start' for call in obj.backend.calls))

    def test_failed_poll_blocks_new_commands(self):
        obj = make_ui(); obj.backend.poll = lambda: (_ for _ in ()).throw(RuntimeError('lost feedback'))
        with patch.object(ui.messagebox, 'askokcancel', return_value=True): obj.start('open')
        self.assertEqual(obj.backend.calls, [])
        self.assertFalse(obj._ready_for_motion())

    def test_exception_after_dispatch_does_not_reenable_stale_readiness(self):
        obj = make_ui()
        def failed_start(*args, **kwargs): obj.backend.calls.append(('dispatched',)); raise RuntimeError('late action error')
        obj.backend.start = failed_start
        with patch.object(ui.messagebox, 'askokcancel', return_value=True): obj.start('step_close')
        self.assertTrue(obj.poll_failed); self.assertFalse(obj._ready_for_motion())

    def test_record_only_when_backend_allows_it(self):
        obj = make_ui()
        with patch.object(ui.messagebox, 'askyesno', return_value=True) as modal: obj.record_result(True)
        modal.assert_not_called(); self.assertEqual(obj.backend.calls, [])
        obj.state['can_record'] = obj.backend.state['can_record'] = True
        with patch.object(ui.messagebox, 'askyesno', return_value=True): obj.record_result(True)
        self.assertEqual(obj.backend.calls, [('record', True)])

    def test_record_rechecks_backend_after_modal(self):
        obj = make_ui(); obj.state['can_record'] = obj.backend.state['can_record'] = True
        def changed(*args, **kwargs): obj.backend.state['can_record'] = False; return True
        with patch.object(ui.messagebox, 'askyesno', side_effect=changed): obj.record_result(False)
        self.assertEqual(obj.backend.calls, [])

    def test_apply_detects_in_place_recommendation_change(self):
        obj = make_ui(); recommendation = dict(commanded_q=.75, measured_q=.72, force_percent=20, trials=3)
        obj.state['recommendation'] = obj.backend.state['recommendation'] = recommendation
        def changed(*args, **kwargs): recommendation['commanded_q'] = .79; return True
        with patch.object(ui.messagebox, 'askokcancel', side_effect=changed): obj.apply_recommendation()
        self.assertEqual(obj.backend.calls, [])

    def test_apply_only_writes_backend_profile_and_invalidates_plan(self):
        obj = make_ui(); obj.state['recommendation'] = obj.backend.state['recommendation'] = dict(commanded_q=.75, measured_q=.72, force_percent=20, trials=3)
        with patch.object(ui.messagebox, 'askokcancel', return_value=True): obj.apply_recommendation()
        self.assertEqual(obj.backend.calls, [('apply',)]); self.assertTrue(obj.invalidations)

    def test_close_requests_stop_and_keeps_window(self):
        obj = make_ui(); obj.request_close()
        self.assertEqual(obj.backend.calls, [('stop',)]); self.assertFalse(obj.window.destroyed)
        self.assertFalse(obj._ready_for_motion())

    def test_can_close_false_keeps_polling(self):
        obj = make_ui(); obj.close_requested = True; obj.backend.close_result = True; obj.poll()
        self.assertFalse(obj.window.destroyed); self.assertTrue(obj.window.scheduled)
        self.assertNotIn(('close',), obj.backend.calls)

    def test_backend_close_false_is_authoritative(self):
        obj = make_ui(); obj.close_requested = True; obj.backend.state['can_close'] = True; obj.poll()
        self.assertIn(('close',), obj.backend.calls); self.assertFalse(obj.window.destroyed)

    def test_backend_close_true_finishes_once(self):
        obj = make_ui(); obj.close_requested = True; obj.backend.state['can_close'] = True; obj.backend.close_result = True; obj.poll(); obj.poll()
        self.assertTrue(obj.window.destroyed); self.assertEqual(obj.closes, [True])

    def test_modal_window_is_not_destroyed_mid_confirmation(self):
        obj = make_ui(); obj.close_requested = obj.confirming = True; obj.backend.state['can_close'] = True; obj.backend.close_result = True; obj.poll()
        self.assertFalse(obj.window.destroyed)

    def test_stop_confirmation_blocks_reentry(self):
        obj = make_ui(); obj.stop(); self.assertFalse(obj._ready_for_motion())
        obj._read_state(); self.assertTrue(obj.stop_pending)
        obj.backend.state['stop_confirmed'] = True; obj._read_state(); self.assertFalse(obj.stop_pending)

    def test_failed_poll_never_destroys_window(self):
        obj = make_ui(); obj.close_requested = True
        obj.backend.poll = lambda: (_ for _ in ()).throw(RuntimeError('lost response'))
        obj.poll(); self.assertFalse(obj.window.destroyed); self.assertTrue(obj.window.scheduled)

    def test_configuration_is_loaded_once(self):
        obj = make_ui(); obj.backend.state['configuration_values'] = {'open_q': .4}; obj._read_state()
        self.assertEqual(obj.open_q.get(), '0.4')
        obj.open_q.set('.45'); obj._read_state(); self.assertEqual(obj.open_q.get(), '.45')

# Isolate the real StackGui class without importing/creating any ROS Node.
source = ast.parse((BASE/'ur5e_a3_grid/scripts/stack_gui.py').read_text())
class_node = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == 'StackGui')
namespace = {'ttk': SimpleNamespace(Combobox=type('Combobox', (), {})),
             'ALIGNMENT_GRIPPER_POSITION': ALIGNMENT_GRIPPER_POSITION}
exec(compile(ast.Module(body=[class_node], type_ignores=[]), '<StackGui AST>', 'exec'), namespace)
StackGui = namespace['StackGui']

class MainUiTests(unittest.TestCase):
    def test_stop_routes_to_calibration(self):
        calls = []; app = StackGui.__new__(StackGui); app.calibration_window = SimpleNamespace(stop=lambda: calls.append('calibration'))
        app.stop(); self.assertEqual(calls, ['calibration'])

    def test_main_close_waits_for_calibration(self):
        calls = []; app = StackGui.__new__(StackGui)
        app.closing = app.close_requested = False; app.real_backend = True
        app.calibration_window = SimpleNamespace(request_close=lambda: calls.append('close_child'))
        app.status = Var(); app.busy = lambda active: calls.append(('busy', active)); app.finish_close = lambda: calls.append('destroy_main')
        app.close(); self.assertEqual(calls, ['close_child', ('busy', True)]); self.assertTrue(app.close_requested)

    def test_arm_launch_rejects_calibration_owner(self):
        app = StackGui.__new__(StackGui); app.job = None; app.stop_pending = app.close_requested = False; app.calibration_window = object()
        app.launch_task('run')  # Any access beyond the ownership guard fails.

    def test_grid_edit_rejects_calibration_owner(self):
        app = StackGui.__new__(StackGui); app.job = None; app.stop_pending = False; app.calibration_window = object()
        app.select_cell(1)

    def test_ready_plan_does_not_enable_run_during_calibration(self):
        app = StackGui.__new__(StackGui); app.real_backend = app.plan_valid = True
        app.trajectory_file = SimpleNamespace(is_file=lambda: True); app.job = None
        app.stop_pending = app.close_requested = False; app.calibration_window = object(); app.run_button = Widget()
        app.execution_ready = lambda: True; app.refresh_run_button()
        self.assertEqual(app.run_button.options['state'], 'disabled')

    def test_busy_false_cannot_unlock_calibration_controls(self):
        app = StackGui.__new__(StackGui); app.calibration_window = object(); app.close_requested = False
        app.controls = [Widget(), Widget()]; app.stop_button = Widget(); app.refresh_run_button = lambda: None
        app.busy(False)
        self.assertTrue(all(widget.options['state'] == 'disabled' for widget in app.controls))
        self.assertEqual(app.stop_button.options['state'], 'normal')


class StandaloneUiTests(unittest.TestCase):
    def test_lost_display_keeps_context_until_backend_confirms_close(self):
        events = []
        ros = SimpleNamespace(init=lambda **kwargs: events.append('init'), ok=lambda: True,
                              shutdown=lambda: events.append('shutdown'))
        root = MagicMock()
        root.mainloop.side_effect = RuntimeError('display disconnected')
        root.update.side_effect = ui.tk.TclError('no display')
        outcomes = iter([False, True])
        def close_backend():
            answer = next(outcomes)
            events.append(('backend_close', answer))
            return answer
        window = SimpleNamespace(closed=False, window=MagicMock(),
                                 request_close=MagicMock(side_effect=ui.tk.TclError('window gone')),
                                 backend=SimpleNamespace(close=close_backend))
        with patch.dict(sys.modules, {'rclpy': ros, 'rclpy.signals': SimpleNamespace(
                SignalHandlerOptions=SimpleNamespace(NO=0))}), \
                patch.object(ui.tk, 'Tk', return_value=root), \
                patch.object(ui, 'GripperCalibrationWindow', return_value=window), \
                patch.object(ui.signal, 'signal', return_value='previous'), \
                patch.object(ui.time, 'sleep'):
            with self.assertRaisesRegex(RuntimeError, 'display disconnected'): ui.main([])
        self.assertEqual(events, ['init', ('backend_close', False), ('backend_close', True), 'shutdown'])

    def test_signal_routes_to_safe_close_before_ros_shutdown(self):
        events = []; handlers = {}
        ros = SimpleNamespace(init=lambda **kwargs: events.append('init'), ok=lambda: True,
                              shutdown=lambda: events.append('shutdown'))
        root = MagicMock()
        root.after.side_effect = lambda delay, callback: callback()
        def factory(*args, **kwargs):
            obj = SimpleNamespace(closed=False, window=MagicMock())
            def close():
                events.append('request_close')
                obj.closed = True
                kwargs['on_close']()
                events.append('safe_close')
            obj.request_close = close
            root.mainloop.side_effect = lambda: handlers[ui.signal.SIGTERM](ui.signal.SIGTERM, None)
            return obj
        def handler(signum, callback):
            previous = handlers.get(signum, 'previous')
            handlers[signum] = callback
            return previous
        with patch.dict(sys.modules, {'rclpy': ros, 'rclpy.signals': SimpleNamespace(
                SignalHandlerOptions=SimpleNamespace(NO=0))}), \
                patch.object(ui.tk, 'Tk', return_value=root), \
                patch.object(ui, 'GripperCalibrationWindow', side_effect=factory) as created, \
                patch.object(ui.signal, 'signal', side_effect=handler):
            ui.main([])
        self.assertFalse(created.call_args.kwargs['gripper_only'])
        self.assertEqual(events, ['init', 'request_close', 'safe_close', 'shutdown'])

    def test_main_owns_ros_context_and_forwards_unknown_ros_args(self):
        events = []; ros = SimpleNamespace()
        ros.init = lambda **kwargs: events.append(('init', kwargs))
        ros.ok = lambda: True
        ros.shutdown = lambda: events.append('shutdown')
        root = MagicMock()
        factory_calls = []
        def factory(*args, **kwargs):
            obj = SimpleNamespace(closed=False, window=MagicMock())
            factory_calls.append((args, kwargs))
            def loop():
                events.append('mainloop')
                obj.closed = True
                kwargs['on_close']()
                events.append('closed')
            root.mainloop.side_effect = loop
            return obj
        no_handlers = object()
        with patch.dict(sys.modules, {'rclpy': ros, 'rclpy.signals': SimpleNamespace(
                SignalHandlerOptions=SimpleNamespace(NO=no_handlers))}), \
                patch.object(ui.tk, 'Tk', return_value=root), \
                patch.object(ui, 'GripperCalibrationWindow', side_effect=factory), \
                patch.object(ui.signal, 'signal', return_value='previous') as signals:
            ui.main(['--gripper-only', '--calibration-file', '/tmp/nominal.yaml',
                     '--physical-profile', '/tmp/physical.yaml', '--ros-args', '-r', '__ns:=/preview'])
        self.assertEqual(events[0], ('init', {'args':['--ros-args', '-r', '__ns:=/preview'],
                                            'signal_handler_options':no_handlers}))
        self.assertTrue(factory_calls[0][1]['gripper_only'])
        self.assertEqual(factory_calls[0][0][1:], ('/tmp/nominal.yaml', '/tmp/physical.yaml'))
        self.assertEqual(events[-2:], ['closed', 'shutdown'])
        self.assertEqual(signals.call_count, 4)

if __name__ == '__main__': unittest.main(verbosity=2)
