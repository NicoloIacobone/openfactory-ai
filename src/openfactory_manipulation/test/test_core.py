import math

import pytest

from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core import pick_place as pp
from openfactory_manipulation.core import selection as sel

GRIPPER = {
    'palm': {'size': [0.06, 0.12, 0.04]},
    'finger': {'size': [0.02, 0.01, 0.06], 'stroke': 0.0425},
    'tcp_offset': 0.085,
}
PARAMS = {
    'approach_distance': 0.10, 'lift_height': 0.15, 'floor_clearance': 0.005,
    'palm_clearance': 0.003, 'preclose_margin': 0.005, 'squeeze_depth': 0.005,
    'release_clearance': 0.005,
}
CUBE = {'shape': 'box', 'size': [0.05, 0.05, 0.05]}
CYLINDER = {'shape': 'cylinder', 'radius': 0.025, 'length': 0.08}
PRISM = {'shape': 'triangular_prism', 'side': 0.05, 'length': 0.06}
IDENTITY = (0.0, 0.0, 0.0, 1.0)
IN_BIN = ((0.40, -0.30, 0.76), IDENTITY)
OUT_BIN = ((0.40, 0.30, 0.76), IDENTITY)


def _axes(q):
    return [g.quat_rotate(q, axis) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]


def test_quaternion_helpers():
    q = g.quat_from_rpy(0.0, 0.0, math.pi / 2)
    assert g.quat_rotate(q, (1, 0, 0)) == pytest.approx((0, 1, 0))
    assert g.yaw_of(q) == pytest.approx(math.pi / 2)
    both = g.quat_multiply(q, q)
    assert g.quat_rotate(both, (1, 0, 0)) == pytest.approx((-1, 0, 0), abs=1e-12)
    pose = g.compose(((1.0, 2.0, 3.0), q), ((1.0, 0.0, 0.0), IDENTITY))
    assert pose[0] == pytest.approx((1.0, 3.0, 3.0))
    assert g.wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert g.wrap_angle(math.pi) == pytest.approx(math.pi)


@pytest.mark.parametrize('yaw', [0.0, 0.4, -1.2, 2.5, math.pi])
def test_top_down_orientation(yaw):
    x_axis, y_axis, z_axis = _axes(pp.top_down_quat(yaw))
    assert z_axis == pytest.approx((0, 0, -1), abs=1e-12)
    # finger axis parallel (either sign) to the object x axis
    obj_x = (math.cos(yaw), math.sin(yaw), 0.0)
    assert abs(sum(a * b for a, b in zip(y_axis, obj_x))) == pytest.approx(1.0)
    assert -math.pi / 2 - 1e-9 <= math.atan2(x_axis[1], x_axis[0]) <= math.pi / 2 + 1e-9


def test_tool0_is_above_a_downward_tcp():
    tcp = ((0.4, 0.0, 1.0), pp.top_down_quat(0.0))
    tool0 = pp.tool0_from_tcp(tcp, 0.085)
    assert tool0[0] == pytest.approx((0.4, 0.0, 1.085))
    assert tool0[1] == tcp[1]


def test_object_pose_in_bin_rests_on_the_floor():
    pose = pp.object_pose_in_bin(IN_BIN, 0.02, -0.01, 0.3, CYLINDER)
    assert pose[0] == pytest.approx((0.42, -0.31, 0.76 + 0.04))
    assert g.yaw_of(pose[1]) == pytest.approx(0.3)


@pytest.mark.parametrize('spec', [CUBE, CYLINDER, PRISM])
def test_grasp_at_centre_respects_clearances(spec):
    tip_below, palm_above = pp.finger_geometry(GRIPPER)
    assert (tip_below, palm_above) == pytest.approx((0.015, 0.045))
    offset = pp.grasp_offset(spec, GRIPPER, PARAMS)
    height = {'box': 0.05, 'cylinder': 0.08, 'triangular_prism': 0.05 * math.sqrt(3) / 2}[
        spec['shape']]
    assert offset == pytest.approx(0.0)          # all phase 2 objects are grasped at the centre
    assert offset - tip_below >= -height / 2 + PARAMS['floor_clearance'] - 1e-12
    assert offset + palm_above >= height / 2 + PARAMS['palm_clearance'] - 1e-12


def test_grasp_raised_for_a_flat_object_and_refused_when_too_short():
    flat = {'shape': 'box', 'size': [0.05, 0.05, 0.02]}
    # fingertips 15 mm below the TCP: centre grasp would leave them 5 mm under the floor plane
    assert pp.grasp_offset(flat, GRIPPER, PARAMS) == pytest.approx(0.015 - 0.01 + 0.005)
    # 5 mm tall object, 10 mm floor clearance: the fingertips cannot reach below its top
    with pytest.raises(ValueError):
        pp.grasp_offset({'shape': 'box', 'size': [0.05, 0.05, 0.005]}, GRIPPER,
                        {**PARAMS, 'floor_clearance': 0.01})


def test_plan_pick_place_waypoints():
    obj = pp.object_pose_in_bin(IN_BIN, 0.0, 0.0, 0.0, CUBE)
    plan = pp.plan_pick_place(obj, OUT_BIN, (0.0, 0.0), CUBE, GRIPPER, PARAMS)
    assert plan.grasp[0] == pytest.approx((0.40, -0.30, 0.785))
    assert plan.pre_grasp[0] == pytest.approx((0.40, -0.30, 0.885))
    assert plan.lift[0] == pytest.approx((0.40, -0.30, 0.935))
    # released 5 mm above the destination floor, same TCP orientation as the grasp
    assert plan.place_object_pose[0] == pytest.approx((0.40, 0.30, 0.76 + 0.025 + 0.005))
    assert plan.place[0] == pytest.approx((0.40, 0.30, 0.79))
    assert plan.pre_place[0] == pytest.approx((0.40, 0.30, 0.94))
    assert plan.retreat[0] == pytest.approx((0.40, 0.30, 0.89))
    assert plan.place[1] == plan.grasp[1] == plan.retreat[1]
    assert (plan.open_position, plan.preclose_position, plan.close_position) == pytest.approx(
        (0.0425, 0.030, 0.020))
    assert plan.held_opening == pytest.approx(0.05)


def test_prism_is_grasped_across_its_length():
    obj = pp.object_pose_in_bin(IN_BIN, 0.0, 0.0, 0.5, PRISM)
    plan = pp.plan_pick_place(obj, OUT_BIN, (0.0, 0.0), PRISM, GRIPPER, PARAMS)
    assert plan.grasp_width == pytest.approx(0.06)
    finger_axis = _axes(plan.grasp[1])[1]
    assert abs(finger_axis[0] * math.cos(0.5) + finger_axis[1] * math.sin(0.5)) == pytest.approx(1)
    assert g.yaw_of(plan.place_object_pose[1]) == pytest.approx(0.5)


def test_too_wide_object_is_refused():
    wide = {'shape': 'box', 'size': [0.083, 0.05, 0.05]}
    with pytest.raises(ValueError):
        pp.plan_pick_place(pp.object_pose_in_bin(IN_BIN, 0, 0, 0, wide), OUT_BIN, (0, 0), wide,
                           GRIPPER, PARAMS)


def test_grasp_and_hold_checks_use_the_jaw_opening():
    obj = pp.object_pose_in_bin(IN_BIN, 0.0, 0.0, 0.0, CUBE)
    plan = pp.plan_pick_place(obj, OUT_BIN, (0.0, 0.0), CUBE, GRIPPER, PARAMS)
    assert plan.held_opening == pytest.approx(0.05)
    assert pp.check_grasp(0.0239 + 0.0240, plan, 0.004) is None
    # asymmetric but still squeezing (left at its target, mimic finger yielded): held
    assert pp.check_hold(0.0200 + 0.0298, plan, 0.004) is None
    assert pp.check_grasp(0.020 + 0.020, plan, 0.004) == 'grasp_failed'   # closed on nothing
    assert pp.check_grasp(None, plan, 0.004) == 'grasp_failed'
    assert pp.check_hold(0.040, plan, 0.004) == 'grasp_slipped'


@pytest.mark.parametrize('code, reason', [
    (1, None), (-31, 'ik_failed'), (-1, 'planning_failed'), (-12, 'planning_failed'),
    (99999, 'planning_failed'), (-4, 'execution_failed'), (-6, 'timeout'),
    (-25, 'moveit_error_-25'),
])
def test_moveit_failure(code, reason):
    assert pp.moveit_failure(code) == reason


def test_raised_moves_along_world_z_only():
    pose = ((0.1, 0.2, 0.3), pp.top_down_quat(0.7))
    up = pp.raised(pose, 0.05)
    assert up[0] == pytest.approx((0.1, 0.2, 0.35)) and up[1] == pose[1]


@pytest.mark.parametrize('reference, pose', [
    (((0.4, -0.3, 0.76), (0.0, 0.0, 0.0, 1.0)), ((0.45, -0.28, 0.785), g.quat_from_rpy(0, 0, 0.5))),
    (((0.1, 0.2, 0.3), g.quat_from_rpy(0.2, -0.4, 1.1)),
     ((-0.3, 0.5, 0.9), g.quat_from_rpy(1.0, 0.3, -2.0))),
])
def test_relative_inverts_compose(reference, pose):
    xyz, quat = g.compose(reference, g.relative(reference, pose))
    assert xyz == pytest.approx(pose[0])
    assert quat == pytest.approx(pose[1]) or quat == pytest.approx([-v for v in pose[1]])


def _hypothesis(class_id, score, xyz=(0.4, -0.3, 0.785), yaw=0.0):
    return {'class_id': class_id, 'score': score, 'xyz': xyz, 'quat': g.quat_from_rpy(0, 0, yaw)}


def test_select_object_takes_the_most_confident_catalog_hypothesis():
    catalog = {'objects': {'cube': {}, 'prism': {}}}
    hypotheses = [_hypothesis('cube', 0.7), _hypothesis('unknown', 0.99),
                  _hypothesis('prism', 0.9), _hypothesis('cube', 0.4)]
    assert sel.select_object(hypotheses, catalog, 0.5)['class_id'] == 'prism'
    assert sel.select_object(hypotheses, catalog, 0.95) is None
    assert sel.select_object([], catalog, 0.5) is None
    assert sel.select_object([_hypothesis('unknown', 0.99)], catalog, 0.5) is None


def test_pose_in_bin_of_a_world_hypothesis():
    bin_pose = ((0.40, -0.30, 0.76), g.quat_from_rpy(0.0, 0.0, 0.0))
    x, y, yaw = sel.pose_in_bin(bin_pose, _hypothesis('cube', 0.9, (0.45, -0.28, 0.785), 0.5))
    assert (x, y, yaw) == pytest.approx((0.05, 0.02, 0.5))
    # a rotated bin: the same world pose seen from a frame turned by 90 degrees
    turned = ((0.40, -0.30, 0.76), g.quat_from_rpy(0.0, 0.0, math.pi / 2))
    x, y, yaw = sel.pose_in_bin(turned, _hypothesis('cube', 0.9, (0.45, -0.28, 0.785), 0.5))
    assert (x, y, yaw) == pytest.approx((0.02, -0.05, 0.5 - math.pi / 2))
