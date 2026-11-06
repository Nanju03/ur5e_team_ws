"""Pane placement and control callbacks with fake Tk widgets and no ROS node."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[2] / 'ur5e_a3_grid/scripts/stack_gui.py'
SPEC = importlib.util.spec_from_file_location('stack_layout_ui', SOURCE)
ui = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ui)


class Var:
    def __init__(self, value=''):
        self.value = value

    def get(self):
        return self.value


class Widget:
    def __init__(self, kind, master=None, **options):
        self.kind, self.master, self.options = kind, master, options
        self.children, self.bindings, self.panes = [], {}, []
        self.packed = self.gridded = None
        if master is not None:
            master.children.append(self)

    def configure(self, **options):
        self.options.update(options)

    def pack(self, **options):
        self.packed = options

    def grid(self, **options):
        self.gridded = options

    def bind(self, name, callback):
        self.bindings[name] = callback

    def unbind(self, name):
        self.bindings.pop(name)

    def add(self, widget, **options):
        self.panes.append((widget, options))

    def sash_place(self, *position):
        self.sash = position

    def winfo_width(self):
        return self.master.width

    def create_window(self, *args, **kwargs):
        return 1

    def bbox(self, *args):
        return (0, 0, 300, 1000)

    rowconfigure = columnconfigure = pack_propagate = lambda *args, **kwargs: None
    yview = xview = set = itemconfigure = yview_scroll = lambda *args, **kwargs: None


class Root(Widget):
    def __init__(self, screen):
        super().__init__('root')
        self.screen, self.idle = screen, []

    def winfo_screenwidth(self):
        return self.screen[0]

    def winfo_screenheight(self):
        return self.screen[1]

    def geometry(self, size):
        self.width, self.height = map(int, size.split('x'))

    def minsize(self, *size):
        self.minimum = size

    def after_idle(self, callback):
        self.idle.append(callback)

    title = lambda *args: None


def build_gui(monkeypatch, real, screen):
    for module, names in ((ui.tk, ('PanedWindow', 'Label', 'Button', 'Canvas', 'Text')),
                          (ui.ttk, ('Frame', 'Label', 'Button', 'LabelFrame', 'Scrollbar',
                                    'Combobox', 'Radiobutton', 'Checkbutton', 'Entry'))):
        for name in names:
            monkeypatch.setattr(module, name, lambda master=None, kind=name, **options: Widget(kind, master, **options))
    monkeypatch.setattr(ui.tk, 'StringVar', Var)
    obj = ui.StackGui.__new__(ui.StackGui)
    obj.root = Root(screen)
    obj.real_backend, obj.plate_enabled, obj.domain = real, True, '0'
    obj.columns, obj.rows, obj.controls = 4, 6, []
    for name in ('connection', 'hardware_status', 'execution_mode', 'step_status', 'status', 'mode',
                 'summary', 'center_x', 'center_y', 'obstacle_height', 'speed', 'empty_hand_clearance',
                 'placement_mode', 'corner_text', 'incremental_mode'):
        setattr(obj, name, Var(name))
    obj.air_path_note = lambda: '공중 경로 안내'
    obj.launch_task, obj.approve_next_step, obj.stop = Mock(), Mock(), Mock()
    obj.build()
    return obj


def is_within(widget, container):
    while widget is not None:
        if widget is container:
            return True
        widget = widget.master
    return False


def descendants(widget):
    for child in widget.children:
        yield child
        yield from descendants(child)


@pytest.mark.parametrize('real', [False, True])
@pytest.mark.parametrize('screen', [(1920, 1080), (1366, 768)])
def test_half_width_terminal_and_pinned_controls_keep_their_existing_callbacks(monkeypatch, real, screen):
    obj = build_gui(monkeypatch, real, screen)
    left, right = [pane for pane, _ in obj.layout_panes.panes]
    assert obj.layout_panes.options['orient'] == 'horizontal'
    assert obj.layout_panes.packed == dict(fill='both', expand=True)
    obj.layout_panes.bindings['<Map>'](None)
    obj.root.idle[0]()
    assert '<Map>' not in obj.layout_panes.bindings
    assert obj.layout_panes.sash == (0, obj.root.width//2, 0)
    assert obj.root.width <= screen[0] and obj.root.height <= screen[1]
    assert all(a <= b for a, b in zip(obj.root.minimum, (obj.root.width, obj.root.height)))

    assert is_within(obj.log, right) and not is_within(obj.log, left)
    assert obj.log.options['wrap'] == 'none' and obj.log.options['font'][1] == 11
    assert obj.log.gridded['sticky'] == 'nsew'
    assert obj.log.master.packed == dict(fill='both', expand=True)
    scrollbars = [item for item in descendants(right) if item.kind == 'Scrollbar']
    assert {item.options['orient'] for item in scrollbars} == {'horizontal', 'vertical'}
    assert {item.options['command'] for item in scrollbars} == {obj.log.xview, obj.log.yview}

    assert len(obj.cells) == 24 and all(is_within(cell, left) for cell in obj.cells.values())
    for control in (obj.run_button, obj.approve_button, obj.stop_button):
        assert is_within(control, left)
        assert not is_within(control, obj.settings_canvas)
    assert obj.approve_button.options['state'] == obj.stop_button.options['state'] == 'disabled'
    obj.run_button.options['command']()
    obj.approve_button.options['command']()
    obj.stop_button.options['command']()
    obj.launch_task.assert_called_once_with('run')
    obj.approve_next_step.assert_called_once_with()
    obj.stop.assert_called_once_with()


def test_control_labels_rewrap_when_the_left_pane_is_resized(monkeypatch):
    obj = build_gui(monkeypatch, True, (1920, 1080))
    left = obj.layout_panes.panes[0][0]
    status = next(item for item in descendants(left) if item.options.get('textvariable') is obj.status)
    status.bindings['<Configure>'](SimpleNamespace(width=480))
    assert status.options['wraplength'] == 480


def copy_fixture(monkeypatch):
    obj = build_gui(monkeypatch, True, (1920, 1080))
    obj.root.clipboard_clear = Mock()
    obj.root.clipboard_append = Mock()
    obj.root.update_idletasks = Mock()
    obj.log.get = Mock(return_value='화면에 남은 마지막 로그\n')
    obj.log_copy_status = SimpleNamespace(set=Mock())
    obj.status = SimpleNamespace(set=Mock())
    return obj


def test_log_copy_uses_the_full_current_file_even_after_the_view_trims(monkeypatch, tmp_path):
    obj = copy_fixture(monkeypatch)
    contents = ''.join(f'{i}: 한글 실행 로그\n' for i in range(1200))+'STACK_ERROR 원인\n'
    obj.job_logfile = tmp_path/'running.log'
    obj.job_logfile.write_text(contents)
    _, terminal = [pane for pane, _ in obj.layout_panes.panes]
    assert is_within(obj.copy_log_button, terminal)
    assert obj.copy_log_button not in obj.controls
    assert obj.copy_log_button.options.get('state') != 'disabled'
    obj.copy_log_button.options['command']()
    obj.root.clipboard_clear.assert_called_once_with()
    obj.root.clipboard_append.assert_called_once_with(contents)
    obj.log.get.assert_not_called()
    obj.status.set.assert_not_called()  # Preserve the actual task error/status.
    obj.launch_task.assert_not_called()
    obj.approve_next_step.assert_not_called()


def test_copy_reads_the_latest_flushed_output_while_a_job_is_running(monkeypatch, tmp_path):
    obj = copy_fixture(monkeypatch)
    obj.job_logfile = tmp_path/'running.log'
    obj.job_logfile.write_text('첫 줄\n')
    obj.copy_log()
    with obj.job_logfile.open('a') as stream:
        stream.write('추가 오류\n')
    obj.copy_log()
    assert obj.root.clipboard_append.call_args.args == ('첫 줄\n추가 오류\n',)


@pytest.mark.parametrize('missing_file', [False, True])
def test_copy_before_a_job_or_with_a_missing_file_uses_visible_logs(monkeypatch, tmp_path, missing_file):
    obj = copy_fixture(monkeypatch)
    if missing_file:
        obj.job_logfile = tmp_path/'missing.log'
    obj.copy_log()
    obj.root.clipboard_append.assert_called_once_with('화면에 남은 마지막 로그\n')
    if missing_file:
        assert '표시 로그만' in obj.log_copy_status.set.call_args.args[0]


def test_empty_logs_leave_the_clipboard_untouched(monkeypatch):
    obj = copy_fixture(monkeypatch)
    obj.log.get.return_value = '\n'
    obj.copy_log()
    obj.root.clipboard_clear.assert_not_called()
    obj.log_copy_status.set.assert_called_once_with('복사할 로그가 없습니다.')


def test_clipboard_error_is_local_and_does_not_overwrite_the_task_failure(monkeypatch):
    obj = copy_fixture(monkeypatch)
    obj.root.clipboard_append.side_effect = ui.tk.TclError('clipboard unavailable')
    obj.copy_log()
    assert '로그 복사 실패' in obj.log_copy_status.set.call_args.args[0]
    obj.status.set.assert_not_called()
