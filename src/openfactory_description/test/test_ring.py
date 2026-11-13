"""Ring layout (openfactory_description.ring) against the single-arm cell it is derived from."""

import math
from pathlib import Path

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory
from openfactory_description import ring

CONFIG = Path(get_package_share_directory('openfactory_description')) / 'config'
CELL = yaml.safe_load((CONFIG / 'cell.yaml').read_text())
RING = yaml.safe_load((CONFIG / 'ring.yaml').read_text())
SIZES = [2, 3, 4, 5, 6, 8, 12, 40]


def _local(arm, xy):
    """A world point in the frame of an arm base (x forward, y left)."""
    dx, dy = xy[0] - arm['xy'][0], xy[1] - arm['xy'][1]
    cos, sin = math.cos(arm['yaw']), math.sin(arm['yaw'])
    return [cos * dx + sin * dy, -sin * dx + cos * dy]


def _cell_bin_in_base(name):
    """A bin of the cell in the frame of its robot base."""
    mount = CELL['robot_mount']['xyz']
    return [CELL['bins'][name]['xy'][0] - mount[0], CELL['bins'][name]['xy'][1] - mount[1]]


def test_cell_geometry():
    assert ring.cell_geometry(CELL) == pytest.approx((0.60, 0.40))


@pytest.mark.parametrize('n', [0, 1, -3, 2.0, True, '2'])
def test_a_ring_needs_at_least_two_arms(n):
    with pytest.raises(ValueError):
        ring.check_n(n)


def test_two_arms_are_the_cell_with_a_second_arm_across_the_bins():
    layout = ring.layout(2, CELL, RING)
    assert [b['xy'] for b in layout['bins']] == [pytest.approx([0.30, 0.0]),
                                                 pytest.approx([-0.30, 0.0])]
    assert [a['xy'] for a in layout['arms']] == [pytest.approx([0.0, -0.40]),
                                                 pytest.approx([0.0, 0.40])]
    assert [a['yaw'] for a in layout['arms']] == pytest.approx([math.pi / 2, -math.pi / 2])
    # the bins are not rotated relative to arm 0, as in the cell
    assert layout['bins'][0]['yaw'] == pytest.approx(layout['arms'][0]['yaw'])


@pytest.mark.parametrize('n', SIZES)
def test_every_arm_sees_its_bins_as_the_cell_arm_does(n):
    layout = ring.layout(n, CELL, RING)
    bins = {item['name']: item for item in layout['bins']}
    assert len(layout['arms']) == len(layout['bins']) == n
    for index, arm in enumerate(layout['arms']):
        assert arm['source_bin'] == ring.bin_name(index)
        assert arm['destination_bin'] == ring.bin_name((index + 1) % n)
        assert _local(arm, bins[arm['source_bin']]['xy']) == pytest.approx(
            _cell_bin_in_base('input'))
        assert _local(arm, bins[arm['destination_bin']]['xy']) == pytest.approx(
            _cell_bin_in_base('output'))
    # the relay closes: every bin is the source of one arm and the destination of another
    assert sorted(a['source_bin'] for a in layout['arms']) == sorted(bins)
    assert sorted(a['destination_bin'] for a in layout['arms']) == sorted(bins)


@pytest.mark.parametrize('n', SIZES)
def test_arms_and_bins_keep_their_distance_for_every_n(n):
    layout = ring.layout(n, CELL, RING)
    chord, standoff = layout['chord'], layout['standoff']
    arms = [a['xy'] for a in layout['arms']]
    bins = [b['xy'] for b in layout['bins']]
    spacing = min(math.dist(a, b) for i, a in enumerate(arms) for b in arms[i + 1:])
    assert spacing == pytest.approx(ring.arm_spacing(n, chord, standoff))
    assert spacing >= chord - 1e-9
    assert min(math.dist(a, b) for i, a in enumerate(bins) for b in bins[i + 1:]) \
        == pytest.approx(chord)
    # a bin is nearer to its own two arms than to any other
    reach = math.hypot(standoff, chord / 2)
    for index, xy in enumerate(bins):
        others = [math.dist(xy, arm) for other, arm in enumerate(arms)
                  if other not in (index, (index - 1) % n)]
        assert all(distance > reach + 1e-9 for distance in others)


@pytest.mark.parametrize('n', SIZES)
def test_everything_stands_on_the_table(n):
    layout = ring.layout(n, CELL, RING)
    radius = layout['table']['radius']
    ix, iy, _ = layout['bin_inner_size']
    half_diagonal = math.hypot(ix / 2 + layout['bin_wall'], iy / 2 + layout['bin_wall'])
    for item in layout['bins']:
        assert math.hypot(*item['xy']) + half_diagonal < radius
    for item in layout['arms']:
        assert math.hypot(*item['xy']) + RING['table_margin'] == pytest.approx(radius)
    assert layout['floor_size'] > 2 * radius
    assert layout['table']['height'] == pytest.approx(0.75)


def test_camera_mount_is_the_cell_camera_in_the_bin_frame():
    layout = ring.layout(3, CELL, RING)
    mount = CELL['camera_mount']
    assert layout['camera_mount']['xyz'] == pytest.approx(
        [0.0, 0.0, mount['xyz'][2] - CELL['bin_wall']])
    assert layout['camera_mount']['rpy'] == pytest.approx(mount['rpy'])


def test_bin_poses_are_the_inner_floor_centres():
    layout = ring.layout(2, CELL, RING)
    poses = ring.bin_poses(layout)
    (x, y, z), quat = poses['bin_0']
    assert (x, y) == pytest.approx((0.30, 0.0))
    assert z == pytest.approx(0.75 + CELL['bin_wall'])
    assert quat == pytest.approx((0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))


def test_names():
    assert ring.arm_joints(1)[0] == 'arm_1_shoulder_pan_joint'
    assert ring.finger_joints(0) == ['arm_0_left_finger_joint', 'arm_0_right_finger_joint']
    assert ring.destination_bin(2, 3) == 0
    assert ring.bin_namespace(4) == '/bin_4'
    assert ring.bin_topic(1, '/camera/camera_info') == '/bin_1/camera/camera_info'


def test_written_layout_round_trips(tmp_path):
    path = ring.write_layout(3, tmp_path)
    assert yaml.safe_load(path.read_text()) == ring.load_layout(3)
