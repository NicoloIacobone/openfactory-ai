"""Checks the generated ring URDFs against the layout they are built from."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from openfactory_description import ring

PKG = Path(get_package_share_directory('openfactory_description'))
CAMERA = yaml.safe_load((PKG / 'config' / 'camera.yaml').read_text())
SIZES = [2, 3]


@pytest.fixture(scope='module', params=SIZES)
def built(request, tmp_path_factory):
    """(n, layout, {variant: URDF root}) for one ring size."""
    n = request.param
    layout_file = ring.write_layout(n, tmp_path_factory.mktemp(f'ring_n{n}'))

    def urdf(name, **mappings):
        doc = xacro.process_file(str(PKG / 'urdf' / name),
                                 mappings={'layout_file': str(layout_file), **mappings})
        return ET.fromstring(doc.toxml())

    roots = {
        'cell': urdf('ring.urdf.xacro', include_fixtures='true'),
        'arms': urdf('ring.urdf.xacro', include_fixtures='false'),
        'fixtures': urdf('ring_fixtures.urdf.xacro'),
        'fixtures_no_sensor': urdf('ring_fixtures.urdf.xacro', camera_sensor='false'),
    }
    return n, ring.load_layout(n), roots


def _joint(root, name):
    joint = root.find(f"joint[@name='{name}']")
    assert joint is not None, f'missing joint {name}'
    return joint


def _floats(element, attribute):
    return [float(v) for v in element.find('origin').get(attribute).split()]


def test_names_are_unique(built):
    _, _, roots = built
    for root in roots.values():
        for tag in ('link', 'joint'):
            names = [item.get('name') for item in root.findall(tag)]
            assert len(names) == len(set(names))
        names = [item.get('name') for item in root.findall('ros2_control')]
        assert len(names) == len(set(names))


@pytest.mark.parametrize('variant', ['cell', 'arms'])
def test_every_arm_is_mounted_where_the_layout_says(built, variant):
    n, layout, roots = built
    root = roots[variant]
    links = {link.get('name') for link in root.findall('link')}
    control = {j.get('name'): j for j in root.findall('ros2_control/joint')}
    for index, arm in enumerate(layout['arms']):
        prefix = ring.arm_prefix(index)
        mount = root.find(f"joint/child[@link='{prefix}base_link']/..")
        assert mount.find('parent').get('link') == 'world'
        assert _floats(mount, 'xyz') == pytest.approx(
            [arm['xy'][0], arm['xy'][1], layout['table']['height'] + layout['mount_z']])
        assert _floats(mount, 'rpy') == pytest.approx([0.0, 0.0, arm['yaw']])

        for name in ring.arm_joints(index):
            assert _joint(root, name).get('type') == 'revolute'
            assert name in control
        assert {prefix + 'tool0', prefix + 'gripper_palm', prefix + 'gripper_tcp',
                prefix + 'left_finger', prefix + 'right_finger'} <= links
        assert _joint(root, prefix + 'gripper_mount_joint').find('parent').get('link') \
            == prefix + 'tool0'
        left, right = ring.finger_joints(index)
        assert _joint(root, right).find('mimic').get('joint') == left
        # the mimic finger has no command interface
        assert [c.get('name') for c in control[left].findall('command_interface')] == ['position']
        assert control[right].findall('command_interface') == []
    assert len(control) == n * (len(ring.ARM_JOINTS) + len(ring.FINGER_JOINTS))
    # arms of one Gazebo model stop each other only with self-collision on
    assert root.find('gazebo/self_collide').text == 'true'
    # the Gazebo camera sensors belong to the fixtures model only
    assert root.find('gazebo/sensor') is None


def test_fixtures_only_with_include_fixtures(built):
    _, layout, roots = built
    fixture_links = {'floor', 'table'}
    for item in layout['bins']:
        fixture_links |= {item['name'], item['camera_prefix'] + 'camera_link',
                          item['camera_prefix'] + 'camera_optical_frame'}
    for variant, expected in (('cell', True), ('arms', False)):
        links = {link.get('name') for link in roots[variant].findall('link')}
        assert (fixture_links <= links) == expected
        assert bool(fixture_links & links) == expected


@pytest.mark.parametrize('variant', ['cell', 'fixtures'])
def test_table_bins_and_cameras_match_the_layout(built, variant):
    _, layout, roots = built
    root = roots[variant]

    cylinder = root.find("link[@name='table']/collision/geometry/cylinder")
    assert float(cylinder.get('radius')) == pytest.approx(layout['table']['radius'])
    assert float(cylinder.get('length')) == pytest.approx(layout['table']['height'])
    table = _joint(root, 'table_joint')
    assert table.find('parent').get('link') == 'world'
    assert _floats(table, 'xyz') == pytest.approx([0.0, 0.0, layout['table']['height']])

    wall = layout['bin_wall']
    for item in layout['bins']:
        joint = _joint(root, item['name'] + '_joint')
        assert joint.find('parent').get('link') == 'table'
        # the bin frame is the inner floor surface: one wall thickness above the table top
        assert _floats(joint, 'xyz') == pytest.approx([item['xy'][0], item['xy'][1], wall])
        assert _floats(joint, 'rpy') == pytest.approx([0.0, 0.0, item['yaw']])
        assert len(root.findall(f"link[@name='{item['name']}']/collision")) == 5

        camera = _joint(root, item['camera_prefix'] + 'camera_joint')
        assert camera.find('parent').get('link') == item['name']
        assert _floats(camera, 'xyz') == pytest.approx(layout['camera_mount']['xyz'])
        assert _floats(camera, 'rpy') == pytest.approx(layout['camera_mount']['rpy'])
        optical = _joint(root, item['camera_prefix'] + 'camera_optical_joint')
        assert optical.find('child').get('link') == item['camera_prefix'] + 'camera_optical_frame'


def test_one_camera_sensor_per_bin_in_the_fixtures_model(built):
    n, layout, roots = built
    assert roots['fixtures_no_sensor'].findall('gazebo/sensor') == []
    sensors = roots['fixtures'].findall('gazebo/sensor')
    assert len(sensors) == n
    assert len({s.get('name') for s in sensors}) == n
    for item in layout['bins']:
        prefix = item['camera_prefix']
        sensor = roots['fixtures'].find(f"gazebo[@reference='{prefix}camera_link']/sensor")
        assert sensor.get('type') == 'rgbd_camera'
        assert sensor.find('topic').text == item['camera_topic']
        assert sensor.find('gz_frame_id').text == prefix + 'camera_optical_frame'
        assert float(sensor.find('update_rate').text) == pytest.approx(
            layout['camera_rate_hz'] or CAMERA['image']['rate_hz'])
        assert float(sensor.find('camera/horizontal_fov').text) == pytest.approx(
            math.radians(CAMERA['image']['hfov_deg']))
