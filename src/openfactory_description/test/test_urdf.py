"""Checks the generated cell URDFs against config/cell.yaml."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

PKG = Path(get_package_share_directory('openfactory_description'))
CELL = yaml.safe_load((PKG / 'config' / 'cell.yaml').read_text())
GRIPPER = yaml.safe_load((PKG / 'config' / 'gripper.yaml').read_text())

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def _urdf(name, **mappings):
    doc = xacro.process_file(str(PKG / 'urdf' / name), mappings=mappings)
    return ET.fromstring(doc.toxml())


def _joint(root, name):
    joint = root.find(f"joint[@name='{name}']")
    assert joint is not None, f'missing joint {name}'
    return joint


def _xyz(element):
    return [float(v) for v in element.find('origin').get('xyz').split()]


@pytest.mark.parametrize('include_fixtures', ['true', 'false'])
def test_cell_robot(include_fixtures):
    root = _urdf('openfactory_cell.urdf.xacro', include_fixtures=include_fixtures)
    links = {link.get('name') for link in root.findall('link')}

    for fixture in ('floor', 'table'):
        assert (fixture in links) == (include_fixtures == 'true')
    for name in ARM_JOINTS:
        assert _joint(root, name).get('type') == 'revolute'

    # ros2_control interfaces exist for every arm joint (gz_ros2_control needs them)
    control_joints = {j.get('name') for j in root.findall('ros2_control/joint')}
    assert set(ARM_JOINTS) <= control_joints

    base = _joint(root, 'base_link-base_link_inertia').find('parent').get('link')
    assert base == 'base_link'
    mount = root.find("joint/child[@link='base_link']/..")
    assert mount.find('parent').get('link') == 'world'

    table = CELL['table']
    expected = [
        table['xyz'][0] + CELL['robot_mount']['xyz'][0],
        table['xyz'][1] + CELL['robot_mount']['xyz'][1],
        table['xyz'][2] + table['size'][2] + CELL['robot_mount']['xyz'][2],
    ]
    assert _xyz(mount) == pytest.approx(expected)


def test_table_model_matches_cell_table():
    standalone = _urdf('table.urdf.xacro')
    in_cell = _urdf('openfactory_cell.urdf.xacro', include_fixtures='true')

    for root in (standalone, in_cell):
        joint = _joint(root, 'table_joint')
        assert joint.find('parent').get('link') == 'world'
        table = CELL['table']
        assert _xyz(joint) == pytest.approx(
            [table['xyz'][0], table['xyz'][1], table['xyz'][2] + table['size'][2]])
        size = root.find("link[@name='table']/collision/geometry/box").get('size')
        assert [float(v) for v in size.split()] == pytest.approx(table['size'])


def test_floor_top_is_world_z0():
    root = _urdf('openfactory_cell.urdf.xacro', include_fixtures='true')
    assert _joint(root, 'floor_joint').find('parent').get('link') == 'world'
    collision = root.find("link[@name='floor']/collision")
    thickness = float(collision.find('geometry/box').get('size').split()[2])
    assert _xyz(collision)[2] + thickness / 2 == pytest.approx(0.0)


@pytest.mark.parametrize('include_fixtures', ['true', 'false'])
def test_gripper(include_fixtures):
    root = _urdf('openfactory_cell.urdf.xacro', include_fixtures=include_fixtures)
    finger = GRIPPER['finger']

    assert _joint(root, 'gripper_mount_joint').find('parent').get('link') == 'tool0'
    tcp = _joint(root, 'gripper_tcp_joint')
    assert tcp.find('parent').get('link') == 'tool0'
    assert _xyz(tcp) == pytest.approx([0.0, 0.0, GRIPPER['tcp_offset']])
    # the TCP lies between the pads, above the fingertips
    tips = GRIPPER['palm']['size'][2] + finger['size'][2]
    assert GRIPPER['palm']['size'][2] < GRIPPER['tcp_offset'] < tips

    for name, axis in (('left_finger_joint', [0, 1, 0]), ('right_finger_joint', [0, -1, 0])):
        joint = _joint(root, name)
        assert joint.get('type') == 'prismatic'
        assert [float(v) for v in joint.find('axis').get('xyz').split()] == axis
        limit = joint.find('limit')
        # commanded range [0, stroke] strictly inside the joint limits (gz_ros2_control#165)
        assert finger['limit_margin'] > 0
        assert float(limit.get('lower')) == pytest.approx(-finger['limit_margin'])
        assert float(limit.get('upper')) == pytest.approx(finger['stroke'] + finger['limit_margin'])
        assert float(limit.get('effort')) == pytest.approx(finger['effort'])

    mimic = _joint(root, 'right_finger_joint').find('mimic')
    assert mimic is not None and mimic.get('joint') == 'left_finger_joint'
    assert float(mimic.get('multiplier')) == 1.0

    # left finger commanded by position; the mimic joint must not have command interfaces
    control = root.find("ros2_control[@name='gripper']")
    assert control.find('hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
    left = control.find("joint[@name='left_finger_joint']")
    right = control.find("joint[@name='right_finger_joint']")
    assert [c.get('name') for c in left.findall('command_interface')] == ['position']
    assert right.findall('command_interface') == []
