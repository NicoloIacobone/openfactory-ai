"""Checks the generated URDFs of the Rubik's cube cell against its layout files."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

PKG = Path(get_package_share_directory('openfactory_description'))
CELL = yaml.safe_load((PKG / 'config' / 'cell.yaml').read_text())
LAYOUT = yaml.safe_load((PKG / 'config' / 'rubik_cell.yaml').read_text())
CUBE = yaml.safe_load((PKG / 'config' / 'rubik.yaml').read_text())
GRIPPER = yaml.safe_load((PKG / 'config' / 'gripper.yaml').read_text())
SIDE = 3 * CUBE['cube']['pitch']

URDFS = [
    ('rubik_fixtures.urdf.xacro', {}),
    ('rubik_cell.urdf.xacro', {'include_fixtures': 'true'}),
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


def _boxes(root, link):
    boxes = []
    for collision in root.findall(f"link[@name='{link}']/collision"):
        size = [float(v) for v in collision.find('geometry/box').get('size').split()]
        boxes.append((_xyz(collision), size))
    return boxes


@pytest.mark.parametrize('include_fixtures', ['true', 'false'])
def test_robot_is_the_cells(include_fixtures):
    """Same arm, gripper, joint names and mount as the cell: its controllers and MoveIt
    configuration apply unchanged."""
    cell = _urdf('openfactory_cell.urdf.xacro', include_fixtures=include_fixtures)
    rubik = _urdf('rubik_cell.urdf.xacro', include_fixtures=include_fixtures)
    assert rubik.get('name') == cell.get('name')

    def moving(root):
        return {j.get('name'): (j.get('type'), j.find('parent').get('link'),
                                j.find('child').get('link'), j.find('origin').get('xyz'))
                for j in root.findall('joint') if j.get('type') != 'fixed'}

    assert moving(rubik) == moving(cell)
    assert ({j.get('name') for j in rubik.findall('ros2_control/joint')}
            == {j.get('name') for j in cell.findall('ros2_control/joint')})
    links = {link.get('name') for link in rubik.findall('link')}
    for fixture in ('floor', 'table', 'pedestal', 'nest', 'camera_link', 'camera_optical_frame'):
        assert (fixture in links) == (include_fixtures == 'true')
    assert 'input_bin' not in links and 'output_bin' not in links
    assert rubik.find('gazebo/sensor') is None
    mount = rubik.find("joint/child[@link='base_link']/..")
    assert _xyz(mount) == pytest.approx(_xyz(cell.find("joint/child[@link='base_link']/..")))


@pytest.mark.parametrize('urdf, mappings', URDFS)
def test_nest_matches_the_layout(urdf, mappings):
    root = _urdf(urdf, **mappings)
    pedestal, nest = LAYOUT['pedestal'], LAYOUT['nest']
    joint = _joint(root, 'pedestal_joint')
    assert joint.find('parent').get('link') == 'table'
    assert _xyz(joint) == pytest.approx([pedestal['xy'][0], pedestal['xy'][1], 0.0])
    joint = _joint(root, 'nest_joint')
    assert joint.find('parent').get('link') == 'pedestal'
    # the nest frame is its inner floor, `height` above the table top
    assert _xyz(joint) == pytest.approx([0.0, 0.0, pedestal['height']])

    inner = SIDE + 2 * nest['clearance']
    boxes = _boxes(root, 'nest')
    assert len(boxes) == 5
    lows = [[c[i] - s[i] / 2 for i in range(3)] for c, s in boxes]
    highs = [[c[i] + s[i] / 2 for i in range(3)] for c, s in boxes]
    outer = inner / 2 + nest['wall']
    assert [min(v[i] for v in lows) for i in range(3)] == pytest.approx(
        [-outer, -outer, -nest['wall']])
    assert [max(v[i] for v in highs) for i in range(3)] == pytest.approx(
        [outer, outer, nest['wall_height']])
    for low, high in zip(lows, highs):
        eps = 1e-9
        inside = all(low[i] < inner / 2 - eps and high[i] > -inner / 2 + eps for i in (0, 1))
        assert not (inside and high[2] > eps), f'box {low}..{high} intrudes into the nest'

    # the pedestal fills the space between the table top and the nest floor
    (centre, size), = _boxes(root, 'pedestal')
    assert centre[2] + size[2] / 2 == pytest.approx(pedestal['height'] - nest['wall'])
    assert centre[2] - size[2] / 2 == pytest.approx(0.0)
    assert size[:2] == pytest.approx(pedestal['size'])


def test_nest_holds_one_layer_and_leaves_room_for_the_fingers():
    pedestal, nest = LAYOUT['pedestal'], LAYOUT['nest']
    pitch = CUBE['cube']['pitch']
    finger, palm = GRIPPER['finger'], GRIPPER['palm']
    # only the bottom layer is inside the walls, with some play but less than a degree and a half
    assert 0.5 * pitch < nest['wall_height'] < pitch - 0.003
    assert 0.0003 <= nest['clearance'] <= 0.001
    assert math.degrees(2 * nest['clearance'] / SIDE) < 1.6
    # tool horizontal, its axis through the cube centre: the fingers (their width is then
    # vertical) pass over the walls, the palm hangs beside the pedestal and above the table
    finger_bottom = 1.5 * pitch - finger['size'][0] / 2
    assert finger_bottom - nest['wall_height'] >= 0.005
    palm_bottom = 1.5 * pitch - palm['size'][0] / 2
    assert pedestal['height'] + palm_bottom >= 0.05
    fingertip_to_palm = finger['size'][2]
    assert fingertip_to_palm - (1.5 * pitch + 0.02) > max(pedestal['size']) / 2
    # the nest stands on the pedestal, the pedestal on the table
    outer = SIDE + 2 * nest['clearance'] + 2 * nest['wall']
    assert outer <= min(pedestal['size'])
    table = CELL['table']['size']
    for i in (0, 1):
        assert abs(pedestal['xy'][i]) + pedestal['size'][i] / 2 <= table[i] / 2
    # the open gripper goes round the cube
    assert SIDE + 0.02 <= 2 * finger['stroke']


@pytest.mark.parametrize('urdf, mappings', URDFS)
def test_camera_is_above_the_nest(urdf, mappings):
    root = _urdf(urdf, **mappings)
    joint = _joint(root, 'camera_joint')
    assert joint.find('parent').get('link') == 'nest'
    assert _xyz(joint) == pytest.approx([0.0, 0.0, LAYOUT['camera_mount']['height']])
    rpy = [float(v) for v in joint.find('origin').get('rpy').split()]
    assert rpy == pytest.approx(LAYOUT['camera_mount']['rpy'])
    sensor = root.find('gazebo/sensor')
    assert (sensor is not None) == (urdf == 'rubik_fixtures.urdf.xacro')
