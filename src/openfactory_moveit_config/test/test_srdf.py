"""Checks that the SRDF matches the cell URDF it is loaded with."""

import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory


def _process(package, *parts, **mappings):
    path = Path(get_package_share_directory(package)).joinpath(*parts)
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


URDF = _process('openfactory_description', 'urdf', 'openfactory_cell.urdf.xacro',
                include_fixtures='true')
SRDF = _process('openfactory_moveit_config', 'srdf', 'openfactory_cell.srdf.xacro')
LINKS = {link.get('name') for link in URDF.findall('link')}
JOINTS = {joint.get('name') for joint in URDF.findall('joint')}
GRIPPER = yaml.safe_load(
    (Path(get_package_share_directory('openfactory_description')) / 'config' / 'gripper.yaml')
    .read_text())


def test_robot_names_match():
    assert SRDF.get('name') == URDF.get('name')


def test_srdf_references_existing_links_and_joints():
    for pair in SRDF.findall('disable_collisions'):
        assert pair.get('link1') in LINKS
        assert pair.get('link2') in LINKS
    for chain in SRDF.findall('group/chain'):
        assert chain.get('base_link') in LINKS
        assert chain.get('tip_link') in LINKS
    for joint in SRDF.findall('group_state/joint'):
        assert joint.get('name') in JOINTS
    for joint in SRDF.findall('group/joint'):
        assert joint.get('name') in JOINTS
    for effector in SRDF.findall('end_effector'):
        assert effector.get('parent_link') in LINKS


def test_table_collides_with_moving_arm_links():
    disabled = {
        frozenset((p.get('link1'), p.get('link2'))) for p in SRDF.findall('disable_collisions')}
    moving = ['upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link',
              'gripper_palm', 'left_finger', 'right_finger']
    for fixture in ('table', 'floor', 'input_bin', 'output_bin', 'camera_link'):
        for link in moving:
            assert frozenset((fixture, link)) not in disabled


def test_gripper_states_match_finger_stroke():
    states = {s.get('name'): s for s in SRDF.findall("group_state[@group='gripper']")}
    open_value = float(states['open'].find("joint[@name='left_finger_joint']").get('value'))
    closed_value = float(states['closed'].find("joint[@name='left_finger_joint']").get('value'))
    assert open_value == GRIPPER['finger']['stroke']
    assert closed_value == 0.0
