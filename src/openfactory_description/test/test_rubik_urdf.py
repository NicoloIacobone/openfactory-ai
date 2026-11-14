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
GRIPPER = yaml.safe_load((PKG / 'config' / 'rubik_gripper.yaml').read_text())
CELL_GRIPPER = yaml.safe_load((PKG / 'config' / 'gripper.yaml').read_text())
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

    def boxes(root, link):
        return [c.find('geometry/box').get('size')
                for c in root.findall(f"link[@name='{link}']/collision")]

    assert boxes(rubik, 'gripper_palm') == boxes(cell, 'gripper_palm')
    pad = GRIPPER['finger']['pad']
    for finger, side, offsets in (('left_finger', 1, [pad['spacing'] / 2, -pad['spacing'] / 2]),
                                  ('right_finger', -1, [0.0])):
        body, *ribs = boxes(rubik, finger)
        assert [body] == boxes(cell, finger)
        assert len(ribs) == len(offsets)
        centres = [_xyz(c) for c in rubik.findall(f"link[@name='{finger}']/collision")[1:]]
        for rib, centre, offset in zip(ribs, centres, offsets):
            assert [float(v) for v in rib.split()] == pytest.approx(pad['size'])
            # a rib stands out of the inner face, flush with the fingertip
            assert centre == pytest.approx(
                [offset, -side * pad['size'][1] / 2,
                 GRIPPER['finger']['size'][2] - pad['size'][2] / 2])
    links = {link.get('name') for link in rubik.findall('link')}
    for fixture in ('floor', 'table', 'pedestal', 'nest', 'camera_link', 'camera_optical_frame'):
        assert (fixture in links) == (include_fixtures == 'true')
    assert 'input_bin' not in links and 'output_bin' not in links
    assert rubik.find('gazebo/sensor') is None
    mount = rubik.find("joint/child[@link='base_link']/..")
    assert _xyz(mount) == pytest.approx(_xyz(cell.find("joint/child[@link='base_link']/..")))


def test_gripper_is_the_cells_with_ribs():
    ours = {**GRIPPER, 'finger': dict(GRIPPER['finger'])}
    pad = ours['finger'].pop('pad')
    assert ours == CELL_GRIPPER
    size, pitch, nest = pad['size'], CUBE['cube']['pitch'], LAYOUT['nest']
    # the three ribs bear on one cubie only, whatever play the nest leaves the cube
    assert pad['spacing'] + size[0] + 2 * (nest['clearance'] + 0.001) <= pitch
    assert size[2] + 0.004 <= pitch
    # the two left ribs are far enough apart to stop the cube from turning between them
    assert pad['spacing'] >= 0.01
    # with the fingertips 2 mm above the bottom layer the ribs are centred on the cube centre
    assert pitch + 0.002 + size[2] / 2 == pytest.approx(1.5 * pitch)
    # the open fingers go round the cube
    assert SIDE + 0.01 <= 2 * (GRIPPER['finger']['stroke'] - size[1])


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
    collisions = root.findall("link[@name='nest']/collision")
    upright = [c for c in collisions if c.find('origin').get('rpy') == '0 0 0']
    rim = [c for c in collisions if c not in upright]
    assert len(upright) == 5 and len(rim) == 4
    boxes = [(_xyz(c), [float(v) for v in c.find('geometry/box').get('size').split()])
             for c in upright]
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

    # lead-in: the inner face of each plate runs from the top of a wall's inner face to
    # `width` further out and `height` higher
    lead = nest['lead_in']
    slope = math.atan2(lead['width'], lead['height'])
    for plate in rim:
        centre = _xyz(plate)
        roll, pitch, _ = (float(v) for v in plate.find('origin').get('rpy').split())
        axis = 0 if abs(pitch) > 1e-9 else 1
        side = math.copysign(1.0, centre[axis])
        assert abs(pitch if axis == 0 else roll) == pytest.approx(slope)
        # outward normal of the plate and the direction of its slope, in the nest frame
        normal = [0.0, 0.0, -math.sin(slope)]
        normal[axis] = side * math.cos(slope)
        along = [0.0, 0.0, math.cos(slope)]
        along[axis] = side * math.sin(slope)
        face = [c - lead['thickness'] / 2 * n for c, n in zip(centre, normal)]
        slant = math.hypot(lead['width'], lead['height'])
        low = [f - slant / 2 * a for f, a in zip(face, along)]
        high = [f + slant / 2 * a for f, a in zip(face, along)]
        assert (low[axis], low[2]) == pytest.approx((side * inner / 2, nest['wall_height']))
        assert (high[axis], high[2]) == pytest.approx(
            (side * (inner / 2 + lead['width']), nest['wall_height'] + lead['height']))
    friction = root.find("gazebo[@reference='nest']/mu1")
    assert float(friction.text) == pytest.approx(nest['friction'])

    # the pedestal fills the space between the table top and the nest floor
    (centre, size), = _boxes(root, 'pedestal')
    assert centre[2] + size[2] / 2 == pytest.approx(pedestal['height'] - nest['wall'])
    assert centre[2] - size[2] / 2 == pytest.approx(0.0)
    assert size[:2] == pytest.approx(pedestal['size'])


def test_nest_holds_one_layer_and_leaves_room_for_the_fingers():
    pedestal, nest = LAYOUT['pedestal'], LAYOUT['nest']
    pitch = CUBE['cube']['pitch']
    finger, palm = GRIPPER['finger'], GRIPPER['palm']
    rim_top = nest['wall_height'] + nest['lead_in']['height']
    # only the bottom layer is inside the walls, with some play but less than a degree and a half
    assert 0.4 * pitch < nest['wall_height'] and rim_top < pitch - 0.003
    assert 0.0003 <= nest['clearance'] <= 0.001
    assert math.degrees(2 * nest['clearance'] / SIDE) < 1.6
    # tool horizontal, its axis through the cube centre: the fingers (their width is then
    # vertical) pass over the walls, the palm hangs beside the pedestal and above the table
    finger_bottom = 1.5 * pitch - finger['size'][0] / 2
    assert finger_bottom - rim_top >= 0.005
    # the rim is steep enough for the cube to slide down it, and stays on the walls
    assert nest['friction'] < nest['lead_in']['height'] / nest['lead_in']['width'] / 2
    assert nest['lead_in']['width'] + nest['lead_in']['thickness'] <= nest['wall']
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
