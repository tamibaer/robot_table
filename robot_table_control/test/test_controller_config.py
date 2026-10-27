import os

import pytest
from robot_table_control.controller_config import (
    base_controllers_file, deep_merge, load_yaml, merge_files)

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')
OVERLAYS = ['ros2_controllers.yaml', 'admittance_controllers.yaml']


def overlay_path(name):
    return os.path.join(CONFIG_DIR, name)


def controller_types(config):
    return config.get('controller_manager', {}).get('ros__parameters', {})


def test_deep_merge_merges_dicts_and_replaces_lists():
    base = {'a': {'x': 1, 'y': [1, 2]}, 'b': 2}
    overlay = {'a': {'y': [3], 'z': 4}}
    assert deep_merge(base, overlay) == {'a': {'x': 1, 'y': [3], 'z': 4}, 'b': 2}
    assert base == {'a': {'x': 1, 'y': [1, 2]}, 'b': 2}


@pytest.mark.parametrize('overlay', OVERLAYS)
def test_overlay_only_configures_known_controllers(overlay):
    base = load_yaml(base_controllers_file())
    config = load_yaml(overlay_path(overlay))
    known = set(controller_types(base)) | set(controller_types(config))
    unknown = [name for name in config if name != 'controller_manager' and name not in known]
    assert not unknown, f'{overlay}: Controller ohne Typ in Basis oder Overlay: {unknown}'


@pytest.mark.parametrize('overlay', OVERLAYS)
def test_new_controllers_have_type(overlay):
    for name, entry in controller_types(load_yaml(overlay_path(overlay))).items():
        if isinstance(entry, dict):
            assert 'type' in entry, f'{overlay}: {name} ohne type'


def test_default_config_keeps_robot_table_controllers():
    merged = merge_files(base_controllers_file(), [overlay_path('ros2_controllers.yaml')])
    types = controller_types(merged)
    assert 'robotiq_gripper_controller' in types
    assert 'robotiq_activation_controller' in types
    assert 'admittance_controller' not in types
    jtc = merged['scaled_joint_trajectory_controller']['ros__parameters']
    assert 'command_joints' not in jtc


def test_admittance_chain_matches_base_joints():
    base_joints = load_yaml(base_controllers_file())['scaled_joint_trajectory_controller']['ros__parameters']['joints']
    merged = merge_files(base_controllers_file(), [overlay_path(name) for name in OVERLAYS])
    chained = ['admittance_controller/' + joint for joint in base_joints]

    assert merged['admittance_controller']['ros__parameters']['joints'] == base_joints
    assert merged['scaled_joint_trajectory_controller']['ros__parameters']['joints'] == base_joints
    assert merged['scaled_joint_trajectory_controller']['ros__parameters']['command_joints'] == chained
    assert merged['forward_position_controller']['ros__parameters']['joints'] == chained
