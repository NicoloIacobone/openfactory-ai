"""Checks the generated cell URDFs against config/cell.yaml."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

PKG = Path(get_package_share_directory('openfactory_description'))
CELL = yaml.safe_load((PKG / 'config' / 'cell.yaml').read_text())
GRIPPER = yaml.safe_load((PKG / 'config' / 'gripper.yaml').read_text())
CAMERA = yaml.safe_load((PKG / 'config' / 'camera.yaml').read_text())
OBJECTS = yaml.safe_load((PKG / 'config' / 'objects.yaml').read_text())['objects']

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

    for fixture in ('floor', 'table', 'input_bin', 'output_bin', 'camera_link',
                    'camera_optical_frame'):
        assert (fixture in links) == (include_fixtures == 'true')
    # the Gazebo camera sensor belongs to the fixtures model only
    assert root.find('gazebo/sensor') is None
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


def _rpy(element):
    return [float(v) for v in element.find('origin').get('rpy').split()]


def _rotation(roll, pitch, yaw):
    """Rows of R = Rz(yaw) Ry(pitch) Rx(roll), the URDF convention."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


@pytest.mark.parametrize('urdf, mappings', [
    ('fixtures.urdf.xacro', {}),
    ('openfactory_cell.urdf.xacro', {'include_fixtures': 'true'}),
])
def test_camera_frames_match_cell_yaml(urdf, mappings):
    root = _urdf(urdf, **mappings)
    mount = CELL['camera_mount']

    joint = _joint(root, 'camera_joint')
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'table'
    assert _xyz(joint) == pytest.approx(mount['xyz'])
    assert _rpy(joint) == pytest.approx(mount['rpy'])

    optical = _joint(root, 'camera_optical_joint')
    assert optical.find('parent').get('link') == 'camera_link'
    assert optical.find('child').get('link') == 'camera_optical_frame'
    assert _xyz(optical) == pytest.approx([0.0, 0.0, 0.0])

    # optical frame in the table frame: z (view) straight down, x (image right) along table +x,
    # y (image down) along table -y
    rotation = _matmul(_rotation(*mount['rpy']), _rotation(*_rpy(optical)))
    columns = [[rotation[i][j] for i in range(3)] for j in range(3)]
    assert columns[0] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)
    assert columns[1] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)
    assert columns[2] == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)

    (centre, size), = _boxes(root, 'camera_link')
    assert size == pytest.approx(CAMERA['body']['size'])
    # the body lies behind the optical centre
    assert centre[0] + size[0] / 2 == pytest.approx(0.0)


def test_camera_sees_the_whole_input_bin_within_its_depth_range():
    mount, image, depth = CELL['camera_mount'], CAMERA['image'], CAMERA['depth']
    spec, wall = CELL['bins']['input'], CELL['bin_wall']
    ix, iy, wall_height = spec['inner_size']
    height = mount['xyz'][2] - wall                 # optical centre above the bin floor

    # straight above the bin centre
    assert mount['xyz'][:2] == pytest.approx(spec['xy'])
    # image x spans table x, image y spans table y (see the frame test); square pixels
    half_x = height * math.tan(math.radians(image['hfov_deg']) / 2)
    half_y = half_x * image['height'] / image['width']
    assert half_x > ix / 2 + wall and half_y > iy / 2 + wall
    # the rim of the walls is in view too (closer to the camera, so it spreads out more)
    rim = height - wall_height
    assert (ix / 2 + wall) * height / rim < half_x and (iy / 2 + wall) * height / rim < half_y

    tallest = max(
        s['size'][2] if s['shape'] == 'box' else
        s['length'] if s['shape'] == 'cylinder' else s['side'] * math.sqrt(3) / 2
        for s in OBJECTS.values())
    assert height - tallest > depth['min_z']
    assert height < depth['max_z']


@pytest.mark.parametrize('camera_sensor', ['true', 'false'])
def test_camera_sensor_in_the_fixtures_model(camera_sensor):
    root = _urdf('fixtures.urdf.xacro', camera_sensor=camera_sensor)
    sensor = root.find("gazebo[@reference='camera_link']/sensor")
    if camera_sensor == 'false':
        assert sensor is None
        return
    image, sim = CAMERA['image'], CAMERA['simulator']
    assert sensor.get('type') == 'rgbd_camera'
    assert float(sensor.find('update_rate').text) == pytest.approx(image['rate_hz'])
    assert sensor.find('topic').text == sim['topic']
    assert sensor.find('gz_frame_id').text == 'camera_optical_frame'
    camera = sensor.find('camera')
    assert float(camera.find('horizontal_fov').text) == pytest.approx(
        math.radians(image['hfov_deg']))
    assert int(camera.find('image/width').text) == image['width']
    assert int(camera.find('image/height').text) == image['height']
    assert [float(camera.find('clip/near').text), float(camera.find('clip/far').text)] == \
        pytest.approx(sim['clip'])
    # the render range covers the valid depth range
    assert sim['clip'][0] < CAMERA['depth']['min_z'] and CAMERA['depth']['max_z'] < sim['clip'][1]
