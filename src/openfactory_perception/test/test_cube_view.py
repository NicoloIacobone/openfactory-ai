import math
from pathlib import Path

import numpy as np
import pytest
import yaml

import synthetic
from openfactory_perception.core import camera, cube_view

PARAMS = yaml.safe_load(
    (Path(__file__).resolve().parents[1] / 'config' / 'perception.yaml').read_text())['cube']
PITCH = 0.019
SIDE = 3 * PITCH
SHAPE = (360, 640)
INTRINSICS = camera.pinhole_intrinsics(640, 360, math.radians(65.0))
CAMERA = synthetic.overhead_camera(0.70)
RIM = 0.013
# sticker colours of a face, row 0 on the +y side, column 0 on the -x side
COLORS = np.array([[[255, 255, 255], [200, 0, 0], [0, 150, 40]],
                   [[255, 215, 0], [255, 100, 0], [0, 50, 200]],
                   [[200, 0, 0], [0, 150, 40], [255, 255, 255]]], np.uint8)


def box(size, z_low, x=0.0, y=0.0, yaw=0.0):
    """Surface samples of a box standing on the plane z = z_low, nest frame."""
    points = synthetic.place({'shape': 'box', 'size': size}, x, y, yaw)
    return points + (0.0, 0.0, z_low)


def nest():
    """The rim of the nest: a square ring up to RIM around the seat."""
    ring = synthetic._grid(0.075, 0.075)
    ring = ring[(np.abs(ring[:, 0]) > 0.0293) | (np.abs(ring[:, 1]) > 0.0293)]
    return np.column_stack((ring, np.full(len(ring), RIM)))


def view(objects, top=None, noise_seed=None):
    """(depth, colour) images of the nest with `objects`; `top` = (x, y, yaw) of a cube whose
    top face carries COLORS."""
    depth = synthetic.render([nest(), *objects], CAMERA, INTRINSICS, SHAPE)
    color = np.full((*SHAPE, 3), 90, np.uint8)
    if top is not None:
        x, y, yaw = top
        c, s = math.cos(yaw), math.sin(yaw)
        grid = synthetic._grid(SIDE, SIDE)
        cell = np.clip(np.floor(grid / PITCH + 1.5).astype(int), 0, 2)
        inner = np.abs(grid / PITCH + 1.5 - cell - 0.5).max(axis=1) < 0.42     # black borders
        points = np.column_stack((x + c * grid[:, 0] - s * grid[:, 1],
                                  y + s * grid[:, 0] + c * grid[:, 1], np.full(len(grid), SIDE)))
        pixel = np.rint(camera.project(
            camera.transform_points(np.linalg.inv(CAMERA), points), INTRINSICS)).astype(int)
        rgb = np.where(inner[:, None], COLORS[2 - cell[:, 1], cell[:, 0]], 5)
        color[pixel[:, 1], pixel[:, 0]] = rgb
    if noise_seed is not None:
        depth = synthetic.add_noise(depth, INTRINSICS.fx, 0.055, 0.08, 0.001,
                                    np.random.default_rng(noise_seed))
    return depth.astype(np.float32), color


def find(depth, color):
    return cube_view.find(depth, color, INTRINSICS, CAMERA, PITCH, PARAMS)


def cube(x=0.0, y=0.0, yaw=0.0):
    return [box([SIDE, SIDE, SIDE], 0.0, x, y, yaw)]


@pytest.mark.parametrize('x, y, yaw_deg', [
    (0.0, 0.0, 0.0), (0.0007, -0.0005, 1.2), (-0.0006, 0.0006, -1.4), (0.002, 0.001, 8.0),
    (0.0, 0.0, 30.0), (0.0, 0.0, -40.0),
])
@pytest.mark.parametrize('noise_seed', [None, 3])
def test_pose_of_a_seated_cube(x, y, yaw_deg, noise_seed):
    yaw = math.radians(yaw_deg)
    seen = find(*view(cube(x, y, yaw), (x, y, yaw), noise_seed))
    assert seen.present
    assert seen.center == pytest.approx((x, y, SIDE / 2), abs=0.0012)
    assert seen.yaw == pytest.approx(yaw, abs=math.radians(1.5))
    assert seen.size == pytest.approx((SIDE, SIDE), abs=0.002)
    assert seen.tilt < math.radians(1.5)
    assert seen.protrusion < 3e-5
    assert seen.points > 1500


@pytest.mark.parametrize('yaw_deg', [0.0, 1.4, -8.0])
@pytest.mark.parametrize('noise_seed', [None, 5])
def test_sticker_colours_follow_the_cube(yaw_deg, noise_seed):
    yaw = math.radians(yaw_deg)
    seen = find(*view(cube(0.0005, -0.0004, yaw), (0.0005, -0.0004, yaw), noise_seed))
    assert (seen.stickers == COLORS).all()


def test_sticker_centers_are_seen_from_above():
    grid = cube_view.sticker_centers((0.0, 0.0, SIDE / 2), 0.0, PITCH)
    assert grid[0, 0] == pytest.approx((-PITCH, PITCH, SIDE))
    assert grid[2, 1] == pytest.approx((0.0, -PITCH, SIDE))
    turned = cube_view.sticker_centers((0.0, 0.0, SIDE / 2), math.pi / 2, PITCH)
    assert turned[1, 2] == pytest.approx((0.0, PITCH, SIDE))


@pytest.mark.parametrize('noise_seed', [None, 7])
def test_empty_nest(noise_seed):
    seen = find(*view([], None, noise_seed))
    assert not seen.present and seen.points < 50


def test_cube_out_of_its_seat_is_not_at_cube_height():
    # lying on the rim, 10 mm too high and tilted: no top face at cube height
    raised = box([SIDE, SIDE, SIDE], 0.012)
    seen = find(*view([raised]))
    assert not seen.present


@pytest.mark.parametrize('turn_deg, sticks_out', [(0.0, False), (12.0, True), (45.0, True),
                                                  (-20.0, True)])
@pytest.mark.parametrize('noise_seed', [None, 9])
def test_bottom_layer_out_of_square_sticks_out(turn_deg, sticks_out, noise_seed):
    bottom = box([SIDE, SIDE, PITCH], 0.0, yaw=math.radians(turn_deg))
    upper = box([SIDE, SIDE, 2 * PITCH], PITCH)
    seen = find(*view([bottom, upper], (0.0, 0.0, 0.0), noise_seed))
    assert seen.present
    assert seen.yaw == pytest.approx(0.0, abs=math.radians(1.5))
    assert (seen.protrusion > 6e-5) == sticks_out


def test_top_layer_out_of_square_changes_the_outline():
    lower = box([SIDE, SIDE, 2 * PITCH], 0.0)
    top = box([SIDE, SIDE, PITCH], 2 * PITCH, yaw=math.radians(25.0))
    seen = find(*view([lower, top]))
    assert seen.present
    # the top face is seen turned, and the layers under it stick out
    assert abs(seen.yaw) == pytest.approx(math.radians(25.0), abs=math.radians(2.0))
    assert seen.protrusion > 6e-5


def test_no_colour_image():
    depth, _ = view(cube())
    seen = cube_view.find(depth, None, INTRINSICS, CAMERA, PITCH, PARAMS)
    assert seen.present and not seen.stickers.any()
