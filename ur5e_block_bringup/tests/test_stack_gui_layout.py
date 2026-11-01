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
                                    'Combobox', 'Radiobutton', 'Entry'))):
        for name in names:
            monkeypatch.setattr(module, name, lambda master=None, kind=name, **options: Widget(kind, master, **options))
    monkeypatch.setattr(ui.tk, 'StringVar', Var)
    obj = ui.StackGui.__new__(ui.StackGui)
    obj.root = Root(screen)
    obj.real_backend, obj.plate_enabled, obj.domain = real, True, '0'
    obj.columns, obj.rows, obj.controls = 4, 6, []
    for name in ('connection', 'hardware_status', 'execution_mode', 'step_status', 'status', 'mode',
                 'summary', 'center_x', 'center_y', 'obstacle_height', 'speed', 'empty_hand_clearance',
                 'placement_mode', 'corner_text'):
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
