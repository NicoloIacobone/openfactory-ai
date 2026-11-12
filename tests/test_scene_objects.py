import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

import scene_objects as so

CONFIG = Path(__file__).resolve().parents[1] / 'src' / 'openfactory_description' / 'config'
CATALOG = yaml.safe_load((CONFIG / 'objects.yaml').read_text())
CELL = yaml.safe_load((CONFIG / 'cell.yaml').read_text())
GRIPPER = yaml.safe_load((CONFIG / 'gripper.yaml').read_text())
OBJECTS = CATALOG['objects']


def test_catalog_has_the_three_mvp_shapes():
    assert {spec['shape'] for spec in OBJECTS.values()} == set(so.SHAPES)


@pytest.mark.parametrize('name', sorted(OBJECTS))
def test_objects_fit_the_open_gripper(name):
    # phase 2 grasps: cube/cylinder across x, prism across its triangular end faces (x)
    width = so.bounding_box(OBJECTS[name])[0]
    assert width < 2 * GRIPPER['finger']['stroke']


def test_bounding_boxes():
    assert so.bounding_box(OBJECTS['cube']) == [0.05, 0.05, 0.05]
    assert so.bounding_box(OBJECTS['cylinder']) == pytest.approx([0.05, 0.05, 0.08])
    assert so.bounding_box(OBJECTS['prism']) == pytest.approx(
        [0.06, 0.05, 0.05 * math.sqrt(3) / 2])
    with pytest.raises(ValueError):
        so.bounding_box({'shape': 'sphere'})


def test_inertia_matches_closed_forms():
    _, (ixx, iyy, izz) = so.inertia({'shape': 'box', 'size': [1.0, 2.0, 3.0], 'mass': 12.0})
    assert (ixx, iyy, izz) == pytest.approx((13.0, 10.0, 5.0))
    _, (ixx, _, izz) = so.inertia({'shape': 'cylinder', 'radius': 1.0, 'length': 2.0, 'mass': 12.0})
    assert (ixx, izz) == pytest.approx((7.0, 6.0))
    com, (ixx, iyy, izz) = so.inertia(
        {'shape': 'triangular_prism', 'side': 1.0, 'length': 2.0, 'mass': 24.0})
    assert ixx == pytest.approx(2.0) and iyy == izz == pytest.approx(9.0)
    assert com == pytest.approx([0.0, 0.0, -math.sqrt(3) / 12])


@pytest.mark.parametrize('name', sorted(OBJECTS))
def test_model_sdf_is_well_formed(name):
    spec = OBJECTS[name]
    root = ET.fromstring(so.model_sdf(f'{name}_0', spec, CATALOG['friction']))
    assert root.tag == 'sdf' and root.find('model').get('name') == f'{name}_0'
    link = root.find('model/link')
    assert float(link.find('inertial/mass').text) == spec['mass']
    for tag in ('ixx', 'iyy', 'izz'):
        assert float(link.find(f'inertial/inertia/{tag}').text) > 0
    collision, visual = link.find('collision'), link.find('visual')
    assert ET.tostring(collision.find('geometry')) == ET.tostring(visual.find('geometry'))
    assert float(collision.find('surface/friction/ode/mu').text) == CATALOG['friction']


def _rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def test_prism_polyline_fills_its_bounding_box():
    spec = OBJECTS['prism']
    pose, geometry = so.geometry_sdf(spec)
    x, y, z, roll, pitch, yaw = (float(v) for v in pose.split())
    rot = _rpy_matrix(roll, pitch, yaw)
    poly = ET.fromstring(geometry)
    height = float(poly.find('height').text)
    corners = []
    for point in poly.findall('point'):
        px, py = (float(v) for v in point.text.split())
        for pz in (0.0, height):          # SDF extrudes the polyline from 0 to height
            local = (px, py, pz)
            corners.append([sum(rot[i][j] * local[j] for j in range(3)) + (x, y, z)[i]
                            for i in range(3)])
    lows = [min(c[i] for c in corners) for i in range(3)]
    highs = [max(c[i] for c in corners) for i in range(3)]
    half = [v / 2 for v in so.bounding_box(spec)]
    assert lows == pytest.approx([-v for v in half], abs=1e-6)
    assert highs == pytest.approx(half, abs=1e-6)
    # flat rectangular face down: two triangle corners on the bottom plane
    assert sum(abs(c[2] + half[2]) < 1e-6 for c in corners) == 4


def test_bin_origin_and_spawn_pose():
    assert so.bin_origin(CELL, 'input') == pytest.approx([0.40, -0.30, 0.76])
    assert so.bin_origin(CELL, 'output') == pytest.approx([0.40, 0.30, 0.76])
    xyz, yaw = so.spawn_pose(CELL, 'input', OBJECTS['cube'], 0.02, -0.01, 0.3)
    assert xyz == pytest.approx([0.42, -0.31, 0.76 + 0.025 + 0.002]) and yaw == 0.3
    with pytest.raises(ValueError):
        so.spawn_pose(CELL, 'input', OBJECTS['cube'], 0.2, 0.0, 0.0)


@pytest.mark.parametrize('rpy', [(0.0, 0.0, 0.0), (0.3, -0.7, 2.0), (math.pi / 2, 0.0, math.pi / 2)])
def test_quaternion_from_rpy_matches_rotation_matrix(rpy):
    x, y, z, w = so.quaternion_from_rpy(*rpy)
    assert x * x + y * y + z * z + w * w == pytest.approx(1.0)
    from_quat = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    expected = _rpy_matrix(*rpy)
    for row, expected_row in zip(from_quat, expected):
        assert row == pytest.approx(expected_row)


def test_rest_states():
    assert so.rest_states(OBJECTS['cube']) == ('catalog',)
    assert so.rest_states(OBJECTS['cylinder']) == so.rest_states(OBJECTS['prism']) == (
        'catalog', 'tumbled')
    with pytest.raises(ValueError):
        so.rest_rpy(OBJECTS['cube'], 'tumbled')
    assert so.resting_size(OBJECTS['cylinder'], 'tumbled') == pytest.approx([0.05, 0.08, 0.05])
    assert so.resting_size(OBJECTS['prism'], 'tumbled') == pytest.approx(
        [0.05 * math.sqrt(3) / 2, 0.05, 0.06])
    xyz, _ = so.spawn_pose(CELL, 'input', OBJECTS['prism'], 0.0, 0.0, 0.0, rest='tumbled')
    assert xyz[2] == pytest.approx(0.76 + 0.03 + 0.002)


@pytest.mark.parametrize('name', ['cylinder', 'prism'])
def test_resting_size_is_the_rotated_bounding_box(name):
    """resting_size equals the extent of the bounding box turned by rest_rpy."""
    spec = OBJECTS[name]
    rot = _rpy_matrix(*so.rest_rpy(spec, 'tumbled'))
    half = [v / 2 for v in so.bounding_box(spec)]
    corners = [[sum(rot[i][j] * c[j] for j in range(3)) for i in range(3)]
               for c in ((sx * half[0], sy * half[1], sz * half[2])
                         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))]
    extent = [max(c[i] for c in corners) - min(c[i] for c in corners) for i in range(3)]
    assert extent == pytest.approx(so.resting_size(spec, 'tumbled'))


def test_rest_error():
    cylinder, prism = OBJECTS['cylinder'], OBJECTS['prism']
    lying = so.quaternion_from_rpy(*so.rest_rpy(cylinder, 'tumbled', 1.0))
    assert so.rest_error(cylinder, 'tumbled', lying) == pytest.approx(0.0, abs=1e-6)
    # rolled about its own axis: still lying
    rolled = so.quat_multiply(lying, so.quaternion_from_rpy(0.0, 0.0, 0.8))
    assert so.rest_error(cylinder, 'tumbled', rolled) == pytest.approx(0.0, abs=1e-6)
    assert so.rest_error(cylinder, 'tumbled', (0.0, 0.0, 0.0, 1.0)) == pytest.approx(math.pi / 2)
    standing = so.quaternion_from_rpy(*so.rest_rpy(prism, 'tumbled', -2.0))
    assert so.rest_error(prism, 'tumbled', standing) == pytest.approx(0.0, abs=1e-6)
    upside = so.quaternion_from_rpy(0.0, -math.pi / 2, 0.3)
    assert so.rest_error(prism, 'tumbled', upside) == pytest.approx(0.0, abs=1e-6)
    assert so.rest_error(prism, 'tumbled', (0.0, 0.0, 0.0, 1.0)) == pytest.approx(math.pi / 2)
    assert so.rest_error(prism, 'catalog', (0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.0)
