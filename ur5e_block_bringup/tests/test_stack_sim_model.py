"""Runtime-only simulation initialization, without starting ROS or controllers."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


SOURCE = Path(__file__).resolve().parents[2] / 'ur5e_a3_grid/launch/stack_model.py'
SPEC = importlib.util.spec_from_file_location('standalone_stack_sim_model', SOURCE)
helper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(helper)


@pytest.mark.parametrize('height', [0., .016])
def test_mock_starts_at_named_home_with_default_point_four_fingers(height):
    model = helper.make_model(height)
    root = ET.fromstring(model.robot_description['robot_description'])
    semantic = ET.fromstring(model.robot_description_semantic['robot_description_semantic'])
    expected = {joint.get('name'): float(joint.get('value'))
                for joint in semantic.find("group_state[@name='home'][@group='ur5e_arm']")}
    expected['left_outer_knuckle_joint'] = .4
    controls = root.findall('ros2_control')
    assert controls
    for control in controls:
        assert control.findtext('hardware/plugin') == 'mock_components/GenericSystem'
        for joint in control.findall('joint'):
            value = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            assert float(value.text) == expected[joint.get('name')]
    # The workbench frame is 400mm above world; its top/base joint is another
    # 400mm above that frame, giving the 800mm world support plane.
    assert float(root.find("joint[@name='base_joint']/origin").get('xyz').split()[2]) == pytest.approx(.4+height)
    riser = root.find("link[@name='ur5e_stack_riser']")
    if height:
        for kind in ('visual', 'collision'):
            assert list(map(float, riser.find(f'{kind}/geometry/box').get('size').split())) == [.2, .2, height]
    else:
        assert riser is None


def test_simulation_refuses_a_physical_control_plugin(monkeypatch):
    model = helper.make_model()
    root = ET.fromstring(model.robot_description['robot_description'])
    root.find('ros2_control/hardware/plugin').text = 'ur_robot_driver/URPositionHardwareInterface'
    model.robot_description['robot_description'] = ET.tostring(root, encoding='unicode')
    monkeypatch.setattr(helper, 'MoveItConfigsBuilder', lambda *a, **kw: type('Builder', (), {'to_moveit_configs': lambda self: model})())
    with pytest.raises(ValueError, match='GenericSystem'):
        helper.make_model(.016)
