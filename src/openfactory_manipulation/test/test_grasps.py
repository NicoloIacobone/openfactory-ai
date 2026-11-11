import math

import pytest

from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core import grasps as gr
from openfactory_manipulation.core import pick_place as pp

GRIPPER = {
    'palm': {'size': [0.06, 0.12, 0.04]},
    'finger': {'size': [0.02, 0.01, 0.11], 'stroke': 0.0425},
    'tcp_offset': 0.135,
}
# the gripper of phases 2 and 3: its palm went below the bin rim (D-014)
SHORT_GRIPPER = {**GRIPPER, 'finger': {'size': [0.02, 0.01, 0.06], 'stroke': 0.0425},
                 'tcp_offset': 0.085}
PARAMS = {
    'approach_distance': 0.10, 'lift_height': 0.15, 'floor_clearance': 0.005,
    'palm_clearance': 0.003, 'preclose_margin': 0.005, 'squeeze_depth': 0.005,
    'release_clearance': 0.005, 'cylinder_directions': 8, 'finger_clearance': 0.003,
}
CATALOG = {'objects': {
    'cube': {'shape': 'box', 'size': [0.05, 0.05, 0.05]},
    'cylinder': {'shape': 'cylinder', 'radius': 0.025, 'length': 0.08},
    'prism': {'shape': 'triangular_prism', 'side': 0.05, 'length': 0.06},
}}
PRISM_HEIGHT = 0.05 * math.sqrt(3) / 2
SIZES = {'cube': (0.05, 0.05, 0.05), 'cylinder': (0.05, 0.05, 0.08),
         'prism': (0.06, 0.05, PRISM_HEIGHT)}
INNER = [0.30, 0.22, 0.10]
IDENTITY = (0.0, 0.0, 0.0, 1.0)
IN_BIN = ((0.40, -0.30, 0.76), IDENTITY)
OUT_BIN = ((0.40, 0.30, 0.76), IDENTITY)


def obj(name, x=0.0, y=0.0, yaw=0.0, index=0, score=0.9):
    return gr.SceneObject(index, name, score, x, y, yaw, SIZES[name])


def candidates(item):
    return gr.generate(item, CATALOG['objects'][item.class_id], GRIPPER, PARAMS)


def check(grasp, objects=(), gripper=GRIPPER):
    return gr.prefilter(grasp, gripper, INNER, gr.obstacles_for(grasp, objects, CATALOG), PARAMS)


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0), (math.pi / 2, math.pi / 2), (-math.pi / 2, math.pi / 2), (math.pi, 0.0),
    (2.0, 2.0 - math.pi), (-2.0, math.pi - 2.0)])
def test_half_turn(angle, expected):
    assert gr.half_turn(angle) == pytest.approx(expected, abs=1e-12)


def test_cube_has_two_perpendicular_candidates():
    across_x, across_y = candidates(obj('cube', 0.02, -0.01, 0.3))
    assert (across_x.label, across_y.label) == ('across_x', 'across_y')
    assert across_x.finger_yaw == pytest.approx(0.3)
    assert across_y.finger_yaw == pytest.approx(0.3 - math.pi / 2)
    for grasp in (across_x, across_y):
        assert (grasp.x, grasp.y, grasp.z) == pytest.approx((0.02, -0.01, 0.025))
        assert grasp.width == pytest.approx(0.05)
        assert grasp.approach_position == pytest.approx(0.030)


def test_cylinder_candidates_spread_over_half_a_turn_whatever_its_yaw():
    found = candidates(obj('cylinder', yaw=1.1))
    assert len(found) == 8
    directions = sorted(grasp.finger_yaw for grasp in found)
    assert directions[0] > -math.pi / 2 and directions[-1] <= math.pi / 2
    steps = [b - a for a, b in zip(directions, directions[1:])]
    assert steps == pytest.approx([math.pi / 8] * 7)
    assert all(grasp.z == pytest.approx(0.04) and grasp.width == pytest.approx(0.05)
               for grasp in found)


def test_prism_is_held_on_its_triangular_end_faces_only():
    grasp, = candidates(obj('prism', yaw=2.0))
    assert grasp.finger_yaw == pytest.approx(2.0 - math.pi)
    assert grasp.width == pytest.approx(0.06)
    assert grasp.z == pytest.approx(PRISM_HEIGHT / 2)


def test_polygon_distance():
    square = gr.rectangle(0.0, 0.0, 0.0, 1.0, 1.0)
    assert gr.polygon_distance(square, gr.rectangle(2.0, 0.0, 0.0, 1.0, 1.0)) == pytest.approx(1.0)
    # corner to corner
    assert gr.polygon_distance(square, gr.rectangle(2.0, 2.0, 0.0, 1.0, 1.0)) == pytest.approx(
        math.sqrt(2.0))
    # a corner of a turned square pointing at a face
    turned = gr.rectangle(2.0, 0.0, math.pi / 4, 1.0, 1.0)
    assert gr.polygon_distance(square, turned) == pytest.approx(1.5 - math.sqrt(0.5))
    assert gr.polygon_distance(square, gr.rectangle(0.8, 0.8, 0.3, 1.0, 1.0)) == 0.0
    assert gr.polygon_distance(square, gr.rectangle(0.1, 0.0, 0.0, 0.2, 0.2)) == 0.0   # inside


def test_cylinder_footprint_encloses_the_circle():
    outline = gr.footprint(obj('cylinder', 0.1, 0.05), CATALOG['objects']['cylinder'])
    assert all(math.hypot(x - 0.1, y - 0.05) >= 0.025 for x, y in outline)
    midpoints = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in gr._edges(outline)]
    assert [math.hypot(x - 0.1, y - 0.05) for x, y in midpoints] == pytest.approx([0.025] * 16)
    # an unknown cluster is its bounding rectangle
    unknown = gr.SceneObject(1, gr.UNKNOWN, 0.1, 0.0, 0.0, 0.0, (0.10, 0.04, 0.05))
    assert sorted(gr.footprint(unknown)) == pytest.approx(
        sorted([(0.05, 0.02), (-0.05, 0.02), (-0.05, -0.02), (0.05, -0.02)]))


def test_gripper_parts_at_the_approach_opening():
    across_x, _ = candidates(obj('cube'))
    left, right, palm = gr.gripper_parts(across_x, GRIPPER)
    xs, ys = [p[0] for p in left[0]], [p[1] for p in left[0]]
    # pads 5 mm from the cube faces, fingers 10 mm thick and 20 mm wide
    assert (min(xs), max(xs), min(ys), max(ys)) == pytest.approx((0.030, 0.040, -0.010, 0.010))
    assert max(p[0] for p in right[0]) == pytest.approx(-0.030)
    assert left[1] == right[1] == pytest.approx(0.025 - 0.015)        # fingertips above the floor
    assert max(p[0] for p in palm[0]) == pytest.approx(0.06)
    assert max(p[1] for p in palm[0]) == pytest.approx(0.03)
    assert palm[1] == pytest.approx(0.025 + 0.095)                    # above the 0.10 m rim


def test_object_in_the_middle_of_an_empty_bin_passes():
    across_x, across_y = candidates(obj('cube'))
    reason, features = check(across_x)
    assert reason is None
    assert features['wall_clearance'] == pytest.approx(0.10)          # finger side to the y wall
    assert features['neighbour_clearance'] == math.inf
    assert check(across_y)[1]['wall_clearance'] == pytest.approx(0.07)


def test_next_to_a_wall_only_the_grasp_along_the_wall_fits():
    # cube 5 mm from the +x wall
    across_x, across_y = candidates(obj('cube', x=0.12))
    assert check(across_x)[0] == 'wall_collision'
    reason, features = check(across_y)
    assert reason is None and features['wall_clearance'] == pytest.approx(0.02)
    # the short-finger gripper put its palm (30 mm half width) below the rim, into that wall
    for grasp in gr.generate(obj('cube', x=0.12), CATALOG['objects']['cube'], SHORT_GRIPPER,
                             PARAMS):
        assert check(grasp, gripper=SHORT_GRIPPER)[0] == 'wall_collision'


def test_a_neighbour_blocks_the_fingers_on_its_side_only():
    target, neighbour = obj('cube'), obj('cube', x=0.06, index=1)    # 10 mm apart along x
    across_x, across_y = candidates(target)
    reason, features = check(across_x, [target, neighbour])
    assert reason == 'neighbour_collision' and features['neighbour_clearance'] == 0.0
    reason, features = check(across_y, [target, neighbour])
    # finger corner to cube corner: 25 mm along x, 5 mm along y
    assert reason is None
    assert features['neighbour_clearance'] == pytest.approx(math.hypot(0.025, 0.005))
    # something lower than the fingertips is not in their way
    flat = gr.SceneObject(1, gr.UNKNOWN, 0.1, 0.06, 0.0, 0.0, (0.05, 0.05, 0.008))
    assert check(across_x, [target, flat])[0] is None


def test_too_wide_for_the_gripper():
    wide = gr.SceneObject(0, 'slab', 0.9, 0.0, 0.0, 0.0, (0.083, 0.03, 0.05))
    spec = {'shape': 'box', 'size': [0.083, 0.03, 0.05]}
    across_x, across_y = gr.generate(wide, spec, GRIPPER, PARAMS)
    assert gr.prefilter(across_x, GRIPPER, INNER, [], PARAMS)[0] == 'gripper_width'
    assert gr.prefilter(across_y, GRIPPER, INNER, [], PARAMS)[0] is None


def test_score_prefers_confidence_and_room_and_penalises_motion():
    weights = {'confidence': 1.0, 'clearance': 0.5, 'motion': 0.3, 'clearance_cap': 0.03,
               'motion_cap': 3.0}
    base = gr.score(0.9, 0.01, 1.0, weights)
    assert gr.score(0.95, 0.01, 1.0, weights) > base
    assert gr.score(0.9, 0.02, 1.0, weights) > base
    assert gr.score(0.9, 0.01, 2.0, weights) < base
    # beyond the caps nothing changes
    assert gr.score(0.9, 0.05, 1.0, weights) == pytest.approx(gr.score(0.9, 0.03, 1.0, weights))
    assert gr.score(0.9, math.inf, 9.0, weights) == pytest.approx(0.9 + 0.5 - 0.3)
    assert gr.joint_distance([0.0, 1.0, -3.0], [0.5, 1.0, 3.0]) == pytest.approx(6.0)


def test_scene_objects_are_expressed_in_the_bin_frame():
    hypothesis = {'class_id': 'prism', 'score': 0.8, 'xyz': (0.45, -0.32, 0.78),
                  'quat': g.quat_from_rpy(0.0, 0.0, 0.7), 'size': SIZES['prism']}
    item, = gr.scene_objects([hypothesis], IN_BIN)
    assert (item.index, item.class_id, item.score) == (0, 'prism', 0.8)
    assert (item.x, item.y, item.yaw) == pytest.approx((0.05, -0.02, 0.7))
    assert item.size == pytest.approx(SIZES['prism'])


def test_tcp_pose_points_down_with_the_fingers_along_the_grasp_direction():
    grasp, = candidates(obj('prism', 0.05, -0.02, 0.7))
    xyz, quat = gr.tcp_pose(grasp, IN_BIN)
    assert xyz == pytest.approx((0.45, -0.32, 0.76 + PRISM_HEIGHT / 2))
    assert g.quat_rotate(quat, (0, 0, 1)) == pytest.approx((0, 0, -1), abs=1e-12)
    finger_axis = g.quat_rotate(quat, (0, 1, 0))
    assert abs(finger_axis[0] * math.cos(0.7) + finger_axis[1] * math.sin(0.7)) == pytest.approx(1)


def test_plan_grasp_preshapes_the_fingers_and_keeps_the_grasp_height_at_the_place():
    item = obj('cylinder', 0.05, 0.0)
    grasp = candidates(item)[2]
    tcp = gr.tcp_pose(grasp, IN_BIN)
    object_pose = pp.object_pose_in_bin(IN_BIN, item.x, item.y, item.yaw,
                                        CATALOG['objects']['cylinder'])
    plan = pp.plan_grasp(tcp, object_pose, grasp.width, item.size[2], OUT_BIN, (-0.09, 0.05),
                         GRIPPER, PARAMS, preshape=True)
    assert plan.grasp == tcp
    assert plan.pre_grasp[0] == pytest.approx((0.45, -0.30, 0.76 + 0.04 + 0.10))
    assert plan.place[0] == pytest.approx((0.31, 0.35, 0.76 + 0.04 + 0.005))
    assert plan.place[1] == tcp[1]
    assert plan.place_object_pose[0] == pytest.approx((0.31, 0.35, 0.76 + 0.04 + 0.005))
    assert plan.approach_position == plan.release_position == pytest.approx(0.030)
    assert plan.approach_position == pytest.approx(grasp.approach_position)
    # the known-pose cycle still approaches and releases fully open
    known = pp.plan_pick_place(object_pose, OUT_BIN, (0.0, 0.0), CATALOG['objects']['cylinder'],
                               GRIPPER, PARAMS)
    assert known.approach_position == known.release_position == pytest.approx(0.0425)
