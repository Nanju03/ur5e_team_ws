"""Gazebo-only model and free-body layout checks without a ROS node."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

GRID = Path(__file__).resolve().parents[2]/'ur5e_a3_grid'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


model = module('gazebo_model_test', GRID/'launch/stack_model.py')
layout = module('gazebo_layout_test', GRID/'scripts/gazebo_layout.py')


def test_gazebo_uses_only_simulated_actuators_and_compliant_finger_followers():
    nominal = model.make_model(.016).robot_description['robot_description']
    actual = model.make_model(.016, simulation_backend='gazebo')
    root = ET.fromstring(actual.robot_description['robot_description'])
    controls = root.findall('ros2_control')
    assert len(controls) == 1
    assert controls[0].findtext('hardware/plugin') == 'gazebo_ros2_control/GazeboSystem'
    joints = {j.get('name'): j for j in controls[0].findall('joint')}
    assert joints['left_outer_knuckle_joint'].find('command_interface').get('name') == 'effort'
    assert sum(j.find('command_interface') is None for j in joints.values()) == 5
    assert sum(j.find("command_interface[@name='position_pid']") is not None for j in joints.values()) == 6
    assert root.find("gazebo/plugin[@name='ag95_mimic_coupling']") is not None
    assert root.find("link[@name='workbench']/inertial/mass") is not None
    assert root.find("link[@name='ur5e_stack_riser']/inertial/mass") is not None
    assert model.make_model(.016).robot_description['robot_description'] == nominal
    assert ET.fromstring(nominal).findtext('ros2_control/hardware/plugin') == 'mock_components/GenericSystem'


def settings():
    result = yaml.safe_load((GRID/'config/a3_grid.yaml').read_text())['/**']['ros__parameters']
    result.update(yaml.safe_load((GRID/'config/stack_blocks.yaml').read_text())['/**']['ros__parameters'])
    result.update(target_cell=14, obstacle_cells=[11],
                  empty_cells=[i for i in range(1, 25) if i not in (10, 11, 14, 15)])
    return result


def test_gazebo_bodies_match_the_current_plate_pockets_and_support_heights():
    objects = {item['name']: item for item in layout.layout_models(settings(), 'file:///tmp/plate.stl')}
    assert len(objects) == 5
    assert objects['ur5e_stack_plate']['position'] == pytest.approx([.2515, .21, .8])
    assert objects['ur5e_stack_block_10']['position'] == pytest.approx([.3703, .03, .8175])
    assert objects['ur5e_stack_block_14']['role'] == 'foundation'
    assert objects['ur5e_stack_obstacle_11']['position'] == pytest.approx([.4297, .03, .88])
    physics = yaml.safe_load((GRID/'config/gazebo_physics.yaml').read_text())
    body = ET.fromstring(layout.model_sdf(objects['ur5e_stack_block_10'], physics['objects'], 10))
    assert body.findtext('model/static') == 'false'
    assert body.findtext('model/link/gravity') == 'true'
    assert float(body.findtext('model/link/inertial/mass')) == .04
    assert float(body.findtext('model/link/collision/surface/friction/ode/mu')) == .4
    assert body.findall('.//joint') == []  # No fixed attachment substitutes for a grasp.


@pytest.mark.parametrize('key,value', [('target_cell', 11), ('target_cell', 10), ('empty_cells', [11])])
def test_overlapping_cell_roles_are_rejected(key, value):
    data = settings()
    if key == 'target_cell' and value == 10:
        data['empty_cells'].append(10)
    data[key] = value
    with pytest.raises(ValueError):
        layout.layout_models(data, 'file:///tmp/plate.stl')
