import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import synthetic  # noqa: E402
from openfactory_perception.core import camera, pipeline, primitives, segmentation  # noqa: E402

DESCRIPTION = PACKAGE.parent / 'openfactory_description' / 'config'
CATALOG = yaml.safe_load((DESCRIPTION / 'objects.yaml').read_text())
CELL = yaml.safe_load((DESCRIPTION / 'cell.yaml').read_text())
CAMERA = yaml.safe_load((DESCRIPTION / 'camera.yaml').read_text())
PARAMS = yaml.safe_load((PACKAGE / 'config' / 'perception.yaml').read_text())

IMAGE = CAMERA['image']
SHAPE = (IMAGE['height'], IMAGE['width'])
K = camera.pinhole_intrinsics(IMAGE['width'], IMAGE['height'], math.radians(IMAGE['hfov_deg']))
INNER = CELL['bins']['input']['inner_size']
CAMERA_HEIGHT = CELL['camera_mount']['xyz'][2] - CELL['bin_wall']      # above the bin floor
VIEW = synthetic.overhead_camera(CAMERA_HEIGHT)
MODELS = primitives.shape_models(CATALOG, PARAMS)
SPECS = CATALOG['objects']

POSES = [(0.0, 0.0, 0.0), (0.09, 0.05, 0.6), (-0.10, -0.06, -1.2), (0.06, -0.07, 2.4),
         (-0.11, 0.07, 0.3)]


def scene(objects, noise_seed=None):
    """Depth image of [(name, x, y, yaw), ...] in the input bin."""
    clouds = [synthetic.place(SPECS[name], x, y, yaw) for name, x, y, yaw in objects]
    depth = synthetic.render(clouds, VIEW, K, SHAPE)
    if noise_seed is None:
        return depth
    d = CAMERA['depth']
    return synthetic.add_noise(depth, K.fx, d['baseline'], d['disparity_noise_px'],
                               d['quantization'], np.random.default_rng(noise_seed))


def detect(depth):
    return pipeline.detect(depth, K, VIEW, INNER, MODELS, PARAMS)[0]


def yaw_error(measured, expected, period):
    if period == 0:
        return 0.0
    return abs(primitives.wrap_to_period(measured - expected, period))


# -- camera geometry -------------------------------------------------------------------------

def test_intrinsics_from_camera_info():
    k = [500.0, 0.0, 320.0, 0.0, 501.0, 180.0, 0.0, 0.0, 1.0]
    assert camera.intrinsics_from_k(k) == camera.Intrinsics(500.0, 501.0, 320.0, 180.0)
    assert camera.intrinsics_from_k(k, (-0.5, 0.25)) == camera.Intrinsics(500.0, 501.0, 319.5, 180.25)
    assert len(PARAMS['principal_point_offset']) == 2
    assert K.fx == pytest.approx(502.3, abs=0.1) and (K.cx, K.cy) == (320.0, 180.0)


def test_back_projection_inverts_projection():
    rng = np.random.default_rng(0)
    points = np.column_stack((rng.uniform(-0.2, 0.2, 50), rng.uniform(-0.1, 0.1, 50),
                              rng.uniform(0.4, 0.9, 50)))
    pixels = camera.project(points, K)
    back_x = (pixels[:, 0] - K.cx) / K.fx * points[:, 2]
    back_y = (pixels[:, 1] - K.cy) / K.fy * points[:, 2]
    assert np.column_stack((back_x, back_y)) == pytest.approx(points[:, :2])


def test_depth_to_points_rejects_invalid_pixels():
    depth = np.full((4, 5), 0.7)
    depth[0, 0], depth[1, 1], depth[2, 2], depth[3, 3], depth[0, 4] = (
        np.nan, np.inf, -np.inf, 0.0, 9.0)
    points, pixels = camera.depth_to_points(depth, K, 0.1, 3.0)
    assert len(points) == 20 - 5
    assert not {(0, 0), (1, 1), (2, 2), (3, 3), (0, 4)} & {tuple(p) for p in pixels}
    assert points[:, 2] == pytest.approx(0.7)


def test_floor_seen_from_above_is_the_plane_z0_of_the_bin():
    depth = synthetic.render([], VIEW, K, SHAPE)
    assert depth[SHAPE[0] // 2, SHAPE[1] // 2] == pytest.approx(CAMERA_HEIGHT)
    optical, _ = camera.depth_to_points(depth, K, 0.1, 3.0)
    floor = camera.transform_points(VIEW, optical)
    normal, centroid, rms = camera.fit_plane(floor)
    assert abs(floor[:, 2]).max() < 1e-9
    assert normal == pytest.approx([0, 0, 1], abs=1e-9) and rms < 1e-9


def test_pose_matrix_and_fit_plane_on_a_tilted_camera():
    tilt = 0.3                                                    # rad about the bin x axis
    quat = (math.sin((math.pi + tilt) / 2), 0.0, 0.0, math.cos((math.pi + tilt) / 2))
    view = camera.pose_matrix((0.0, 0.2, 0.7), quat)
    depth = synthetic.render([], view, K, SHAPE)
    optical, _ = camera.depth_to_points(depth, K, 0.1, 3.0)
    normal, _, rms = camera.fit_plane(optical)                    # in the optical frame
    assert rms < 1e-9
    assert math.acos(abs(normal[2])) == pytest.approx(tilt, abs=1e-6)
    assert abs(camera.transform_points(view, optical)[:, 2]).max() < 1e-9


@pytest.mark.parametrize('quat', [
    (0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0),
    (0.5, 0.5, 0.5, 0.5), (0.1826, 0.3651, 0.5477, 0.7303), (-0.6, 0.0, 0.8, 0.0),
])
def test_quaternion_matrix_round_trip(quat):
    back = camera.quat_from_rotation(camera.rotation_from_quat(quat))
    sign = 1.0 if np.dot(back, quat) > 0 else -1.0             # q and -q are the same rotation
    assert np.multiply(back, sign) == pytest.approx(quat, abs=1e-4)
    assert back[3] >= 0


def test_yaw_matrix_composes_with_a_frame_pose():
    world_from_bin = camera.pose_matrix((0.4, -0.3, 0.76), (0.0, 0.0, 0.0, 1.0))
    pose = world_from_bin @ camera.yaw_matrix((0.05, 0.02, 0.025), 0.5)
    assert pose[:3, 3] == pytest.approx([0.45, -0.28, 0.785])
    assert camera.quat_from_rotation(pose[:3, :3]) == pytest.approx(
        (0.0, 0.0, math.sin(0.25), math.cos(0.25)))


# -- segmentation ----------------------------------------------------------------------------

def test_crop_keeps_only_object_points_inside_the_bin():
    points = np.array([
        [0.0, 0.0, 0.03],                      # object point
        [0.0, 0.0, 0.002],                     # floor
        [0.148, 0.0, 0.05],                    # on the wall
        [0.0, -0.108, 0.05],                   # on the wall
        [0.0, 0.0, 0.2],                       # arm above the bin
        [0.3, 0.0, 0.03],                      # outside
    ])
    mask = segmentation.crop_to_bin(points, INNER, 0.005, 0.006, 0.12)
    assert mask.tolist() == [True, False, False, False, False, False]


def test_height_map_and_segmentation_of_two_blobs_and_a_speck():
    xs, ys = np.meshgrid(np.arange(-0.10, -0.06, 0.001), np.arange(-0.02, 0.02, 0.001))
    a = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.05)))
    b = a + (0.15, 0.03, 0.03)
    speck = np.array([[0.0, -0.08, 0.02]])
    points = np.vstack([a, b, speck])
    heights, count = segmentation.height_map(points, INNER, 0.002)
    assert heights.shape == segmentation.grid_shape(INNER, 0.002) == (110, 150)
    assert count.sum() == len(points)
    labels, blobs = segmentation.segment(count, min_cells=75)
    assert len(blobs) == 2
    row, col = segmentation.grid_indices(points, INNER, 0.002)
    assert labels[row[-1], col[-1]] == 0                         # the speck is gone
    assert {round(float(np.nanmean(heights[labels == blob])), 3) for blob in blobs} == {0.05, 0.08}


# -- shape models ----------------------------------------------------------------------------

def test_shape_models_follow_the_catalog():
    models = {m.name: m for m in MODELS}
    assert set(models) == set(SPECS)
    cube, cylinder, prism = models['cube'], models['cylinder'], models['prism']
    assert (cube.height, cube.long, cube.short, cube.yaw_period) == (0.05, 0.05, 0.05, math.pi / 2)
    assert (cylinder.height, cylinder.long, cylinder.yaw_period) == (0.08, 0.05, 0.0)
    assert cylinder.fill == pytest.approx(math.pi / 4)
    ridge = 0.05 * math.sqrt(3) / 2
    assert prism.box == pytest.approx((0.06, 0.05, ridge))
    assert prism.long == 0.06 and prism.short < 0.05              # the floor threshold trims the base
    assert prism.height < ridge and prism.flat < 0.3
    assert prism.yaw_offset == 0.0 and prism.yaw_period == math.pi


@pytest.mark.parametrize('angle, period, expected', [
    (0.1, math.pi / 2, 0.1), (math.pi / 2 + 0.1, math.pi / 2, 0.1), (-3.0, math.pi, math.pi - 3.0),
    (2.0, 0.0, 2.0), (math.pi / 4, math.pi / 2, math.pi / 4), (-math.pi / 4, math.pi / 2, math.pi / 4),
])
def test_wrap_to_period(angle, period, expected):
    assert primitives.wrap_to_period(angle, period) == pytest.approx(expected)


# -- whole pipeline on synthetic views -------------------------------------------------------

@pytest.mark.parametrize('name', sorted(SPECS))
@pytest.mark.parametrize('pose', POSES)
def test_single_object_ideal_depth(name, pose):
    found = detect(scene([(name, *pose)]))
    assert [d.class_id for d in found] == [name]
    d = found[0]
    assert d.score > 0.6
    assert math.hypot(d.xyz[0] - pose[0], d.xyz[1] - pose[1]) < 0.002
    assert d.xyz[2] == pytest.approx(synthetic.bounding_height(SPECS[name]) / 2)
    assert yaw_error(d.yaw, pose[2], d.yaw_period) < math.radians(3)


@pytest.mark.parametrize('name', sorted(SPECS))
@pytest.mark.parametrize('pose', POSES)
def test_single_object_with_depth_noise(name, pose):
    found = detect(scene([(name, *pose)], noise_seed=7))
    assert [d.class_id for d in found] == [name]
    d = found[0]
    assert d.score > 0.4
    assert math.hypot(d.xyz[0] - pose[0], d.xyz[1] - pose[1]) < 0.003
    assert yaw_error(d.yaw, pose[2], d.yaw_period) < math.radians(6)


@pytest.mark.parametrize('noise_seed', [None, 3])
def test_three_separate_objects(noise_seed):
    objects = [('cube', -0.09, 0.05, 0.4), ('cylinder', 0.02, -0.05, 0.0), ('prism', 0.10, 0.04, 1.0)]
    found = detect(scene(objects, noise_seed))
    assert sorted(d.class_id for d in found) == ['cube', 'cylinder', 'prism']
    for name, x, y, yaw in objects:
        d = next(d for d in found if d.class_id == name)
        assert math.hypot(d.xyz[0] - x, d.xyz[1] - y) < 0.003
        assert yaw_error(d.yaw, yaw, d.yaw_period) < math.radians(6)
    assert [d.score for d in found] == sorted((d.score for d in found), reverse=True)


@pytest.mark.parametrize('noise_seed', [None, 1, 2, 3, 4])
def test_empty_bin_gives_no_detection(noise_seed):
    assert detect(scene([], noise_seed)) == []


def test_all_invalid_depth_gives_no_detection():
    assert detect(np.full(SHAPE, np.nan, dtype=np.float32)) == []


def test_a_shape_outside_the_catalog_is_unknown():
    slab = {'shape': 'box', 'size': [0.09, 0.03, 0.02]}
    depth = synthetic.render([synthetic.place(slab, 0.0, 0.0, 0.3)], VIEW, K, SHAPE)
    found = detect(depth)
    assert [d.class_id for d in found] == [primitives.UNKNOWN]
    assert found[0].score < PARAMS['min_score']
    assert found[0].size == pytest.approx((0.09, 0.03, 0.02), abs=0.004)


def test_the_arm_above_the_bin_is_ignored():
    arm = {'shape': 'box', 'size': [0.08, 0.08, 0.08]}
    blob = synthetic.place(arm, 0.0, 0.0, 0.0) + (0.0, 0.0, 0.20)      # floating 0.2 m above
    found = detect(synthetic.render([blob, synthetic.place(SPECS['cube'], 0.10, 0.05, 0.0)],
                                    VIEW, K, SHAPE))
    assert [d.class_id for d in found] == ['cube']
