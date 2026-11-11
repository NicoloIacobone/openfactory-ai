"""Synthetic depth images of catalog primitives in a bin, for the perception tests.

Surfaces are sampled densely (finer than a pixel) in the bin frame, projected into the camera
and z-buffered, so occlusion and perspective are those of a real view. No ROS, no simulator.
"""

import math

import numpy as np

from openfactory_perception.core import camera

STEP = 0.0004                     # m, surface sampling: a floor pixel is about 1.4 mm wide


def _grid(a, b):
    """Points of the rectangle [-a/2, a/2] x [-b/2, b/2] on a STEP grid, N x 2."""
    u = np.arange(-a / 2, a / 2 + STEP / 2, STEP)
    v = np.arange(-b / 2, b / 2 + STEP / 2, STEP)
    return np.stack(np.meshgrid(u, v), -1).reshape(-1, 2)


def surface(spec):
    """Visible surface samples of a catalog object, N x 3, in its object frame (bbox centre)."""
    shape = spec['shape']
    if shape == 'box':
        x, y, z = spec['size']
        top = np.column_stack((_grid(x, y), np.full(len(_grid(x, y)), z / 2)))
        sides = []
        for sign in (-1, 1):
            g = _grid(y, z)
            sides.append(np.column_stack((np.full(len(g), sign * x / 2), g[:, 0], g[:, 1])))
            g = _grid(x, z)
            sides.append(np.column_stack((g[:, 0], np.full(len(g), sign * y / 2), g[:, 1])))
        return np.vstack([top, *sides])
    if shape == 'cylinder':
        r, length = spec['radius'], spec['length']
        g = _grid(2 * r, 2 * r)
        g = g[np.hypot(g[:, 0], g[:, 1]) <= r]
        top = np.column_stack((g, np.full(len(g), length / 2)))
        angle = np.arange(0, 2 * math.pi, STEP / r)
        height = np.arange(-length / 2, length / 2 + STEP / 2, STEP)
        a, h = np.meshgrid(angle, height)
        side = np.column_stack((r * np.cos(a).ravel(), r * np.sin(a).ravel(), h.ravel()))
        return np.vstack([top, side])
    if shape == 'triangular_prism':
        s, length = spec['side'], spec['length']
        ridge = s * math.sqrt(3) / 2
        g = _grid(length, s)                          # x along the ridge, y across the base
        slopes = np.column_stack((g[:, 0], g[:, 1], ridge / 2 - np.abs(g[:, 1]) * math.sqrt(3)))
        ends = []
        for sign in (-1, 1):
            e = _grid(s, ridge)
            e = e[e[:, 1] <= ridge / 2 - np.abs(e[:, 0]) * math.sqrt(3)]
            ends.append(np.column_stack((np.full(len(e), sign * length / 2), e[:, 0], e[:, 1])))
        return np.vstack([slopes, *ends])
    raise ValueError(shape)


def bounding_height(spec):
    shape = spec['shape']
    if shape == 'box':
        return spec['size'][2]
    if shape == 'cylinder':
        return spec['length']
    return spec['side'] * math.sqrt(3) / 2


def place(spec, x, y, yaw):
    """Surface samples of an object resting on the bin floor at (x, y, yaw), bin frame."""
    points = surface(spec)
    c, s = math.cos(yaw), math.sin(yaw)
    rotated = np.column_stack((c * points[:, 0] - s * points[:, 1],
                               s * points[:, 0] + c * points[:, 1], points[:, 2]))
    return rotated + (x, y, bounding_height(spec) / 2)


def overhead_camera(height):
    """4 x 4 bin-from-optical transform of a camera `height` above the bin centre, looking
    straight down with image right = bin +x and image down = bin -y (the cell's mount)."""
    matrix = np.eye(4)
    matrix[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    matrix[:3, 3] = (0, 0, height)
    return matrix


def render(objects, bin_from_optical, intrinsics, shape, floor_size=(2.0, 2.0)):
    """Ideal z-depth image (float64, metres, inf where nothing is hit).

    `objects`: iterable of N x 3 point sets in the bin frame; the floor is the plane z = 0 of
    the bin frame, `floor_size` wide (the table around the bin is at about the same depth).
    """
    height, width = shape
    optical_from_bin = np.linalg.inv(bin_from_optical)
    depth = np.full(shape, np.inf)

    # floor: analytic ray-plane intersection per pixel
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack(((u - intrinsics.cx) / intrinsics.fx, (v - intrinsics.cy) / intrinsics.fy,
                     np.ones_like(u, dtype=float)), -1)
    normal = optical_from_bin[:3, 2]                   # bin z axis in the optical frame
    origin = optical_from_bin[:3, 3]                   # bin origin in the optical frame
    with np.errstate(divide='ignore', invalid='ignore'):
        z = (normal @ origin) / (rays @ normal)
    hit = camera.transform_points(bin_from_optical, (rays * z[..., None]).reshape(-1, 3))
    on_floor = ((z.ravel() > 0) & (np.abs(hit[:, 0]) <= floor_size[0] / 2)
                & (np.abs(hit[:, 1]) <= floor_size[1] / 2))
    depth.ravel()[on_floor] = z.ravel()[on_floor]

    for points in objects:
        optical = camera.transform_points(optical_from_bin, points)
        pixel = np.rint(camera.project(optical, intrinsics)).astype(int)
        ok = ((optical[:, 2] > 0) & (pixel[:, 0] >= 0) & (pixel[:, 0] < width)
              & (pixel[:, 1] >= 0) & (pixel[:, 1] < height))
        np.minimum.at(depth, (pixel[ok, 1], pixel[ok, 0]), optical[ok, 2])
    return depth


def add_noise(depth, fx, baseline, sigma_px, quantization, rng):
    """The cell's depth error model (openfactory_simulation.depth_noise), restated for tests."""
    valid = np.isfinite(depth)
    disparity = fx * baseline / depth[valid] + rng.normal(0, sigma_px, valid.sum())
    noisy = np.full(depth.shape, np.nan, dtype=np.float32)
    noisy[valid] = np.round(fx * baseline / disparity / quantization) * quantization
    return noisy
