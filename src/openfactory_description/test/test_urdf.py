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

    for fixture in ('floor', 'table', 'input_bin', 'output_bin'):
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


def test_fixtures_model_matches_cell_table():
    standalone = _urdf('fixtures.urdf.xacro')
    in_cell = _urdf('openfactory_cell.urdf.xacro', include_fixtures='true')

    for root in (standalone, in_cell):
        joint = _joint(root, 'table_joint')
        assert joint.find('parent').get('link') == 'world'
        table = CELL['table']
        assert _xyz(joint) == pytest.approx(
            [table['xyz'][0], table['xyz'][1], table['xyz'][2] + table['size'][2]])
        size = root.find("link[@name='table']/collision/geometry/box").get('size')
        assert [float(v) for v in size.split()] == pytest.approx(table['size'])


def _boxes(root, link):
    """(centre xyz, size) of every collision box of a link, in the link frame."""
    boxes = []
    for collision in root.findall(f"link[@name='{link}']/collision"):
        size = [float(v) for v in collision.find('geometry/box').get('size').split()]
        boxes.append((_xyz(collision), size))
    return boxes


@pytest.mark.parametrize('urdf, mappings', [
    ('fixtures.urdf.xacro', {}),
    ('openfactory_cell.urdf.xacro', {'include_fixtures': 'true'}),
])
@pytest.mark.parametrize('name', ['input', 'output'])
def test_bin_matches_cell_yaml(urdf, mappings, name):
    root = _urdf(urdf, **mappings)
    spec, wall = CELL['bins'][name], CELL['bin_wall']
    ix, iy, h = spec['inner_size']

    joint = _joint(root, f'{name}_bin_joint')
    assert joint.find('parent').get('link') == 'table'
    # the bin frame is the inner floor surface: one wall thickness above the table top
    assert _xyz(joint) == pytest.approx([spec['xy'][0], spec['xy'][1], wall])

    boxes = _boxes(root, f'{name}_bin')
    assert len(boxes) == 5
    lows = [[c[i] - s[i] / 2 for i in range(3)] for c, s in boxes]
    highs = [[c[i] + s[i] / 2 for i in range(3)] for c, s in boxes]
    # outer envelope: floor under the frame, walls up to the inner height
    assert [min(v[i] for v in lows) for i in range(3)] == pytest.approx(
        [-ix / 2 - wall, -iy / 2 - wall, -wall])
    assert [max(v[i] for v in highs) for i in range(3)] == pytest.approx(
        [ix / 2 + wall, iy / 2 + wall, h])
    # the inner volume is free: no box reaches inside it above the floor
    for low, high in zip(lows, highs):
        eps = 1e-9  # boxes may touch the inner faces, not overlap them
        inside = all(low[i] < bound - eps and high[i] > -bound + eps
                     for i, bound in ((0, ix / 2), (1, iy / 2)))
        assert not (inside and high[2] > eps), f'box {low}..{high} intrudes into the bin'


def test_bins_on_table_and_apart():
    table = CELL['table']['size']
    wall = CELL['bin_wall']
    footprints = []
    for spec in CELL['bins'].values():
        ix, iy, _ = spec['inner_size']
        x, y = spec['xy']
        footprint = (x - ix / 2 - wall, x + ix / 2 + wall, y - iy / 2 - wall, y + iy / 2 + wall)
        assert -table[0] / 2 <= footprint[0] and footprint[1] <= table[0] / 2
        assert -table[1] / 2 <= footprint[2] and footprint[3] <= table[1] / 2
        footprints.append(footprint)
    a, b = footprints
    assert a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or b[3] < a[2]


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
