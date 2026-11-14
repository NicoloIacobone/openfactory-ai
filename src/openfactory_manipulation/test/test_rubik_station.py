import math

import pytest

from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core.rubik import station as st

GRIPPER = {
    'palm': {'size': [0.06, 0.12, 0.04]},
    'finger': {'size': [0.02, 0.01, 0.11], 'stroke': 0.0425},
    'tcp_offset': 0.135,
}
PITCH = 0.019
X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def station():
    return st.Station(PITCH, GRIPPER, tip_margin=0.002)


def axes(quat):
    return [g.quat_rotate(quat, v) for v in (X, Y, Z)]


@pytest.mark.parametrize('tool, finger', [
    (st.DOWN, Y), (st.DOWN, X), (X, Y), ((-1.0, 0.0, 0.0), Y), (Y, X), ((0.0, -1.0, 0.0), X),
    (st.DOWN, (math.sqrt(0.5), math.sqrt(0.5), 0.0)),
])
def test_tool_quat_puts_the_axes_where_asked(tool, finger):
    x, y, z = axes(st.tool_quat(tool, finger))
    assert z == pytest.approx(tool, abs=1e-12)
    assert y == pytest.approx(finger, abs=1e-12)
    assert x == pytest.approx(st._cross(finger, tool), abs=1e-12)


def test_grasp_from_above_holds_the_two_upper_layers():
    s = station()
    assert s.tcp_to_tip == pytest.approx(0.015)
    assert s.tip_height() == pytest.approx(PITCH + 0.002)
    xyz, quat = s.tcp_pose(s.seat, st.DOWN, Y)
    # the TCP is on the vertical through the cube centre, 15 mm above the fingertips
    assert xyz == pytest.approx((0.0, 0.0, s.tip_height() + 0.015))
    assert axes(quat)[2] == pytest.approx(st.DOWN)


def test_lift_and_retreat():
    s = station()
    assert s.lifted(0.04) == pytest.approx((0.0, 0.0, 1.5 * PITCH + 0.04))
    pose = s.tcp_pose(s.seat, X, Y)
    back, quat = s.retreat(pose, 0.05)
    assert quat == pose[1]
    assert back == pytest.approx((pose[0][0] - 0.05, pose[0][1], pose[0][2]))


def test_arc_tips_the_cube_about_the_finger_axis_through_its_centre():
    s = station()
    center = s.lifted(0.05)
    poses = s.arc(center, Y, -math.pi / 2, 9)
    assert len(poses) == 9
    for xyz, quat in poses:
        x, y, z = axes(quat)
        assert y == pytest.approx(Y, abs=1e-12)
        # the cube centre stays where it is, at the same distance in front of the TCP
        ahead = tuple(p + (s.tcp_to_tip - s.tip_depth) * a for p, a in zip(xyz, z))
        assert ahead == pytest.approx(center, abs=1e-12)
    # -90 degrees about +y takes the tool from pointing down to pointing along +x
    assert axes(poses[-1][1])[2] == pytest.approx(X, abs=1e-12)
    end = s.tcp_pose(center, X, Y)
    assert poses[-1][0] == pytest.approx(end[0], abs=1e-12)
    assert poses[-1][1] == pytest.approx(end[1], abs=1e-12)


def test_spin_turns_the_fingers_about_the_vertical():
    s = station()
    poses = s.spin(s.seat, Y, math.pi / 2, 6)
    for xyz, quat in poses:
        assert xyz == pytest.approx(s.tcp_pose(s.seat, st.DOWN, Y)[0])
        assert axes(quat)[2] == pytest.approx(st.DOWN, abs=1e-12)
    # a quarter turn anticlockwise seen from above takes +y to -x
    assert axes(poses[-1][1])[1] == pytest.approx((-1.0, 0.0, 0.0), abs=1e-12)
