"""The Rubik's cube in the nest, from one view of the overhead RGB-D camera. ROS-free.

Inputs are the measured depth image, the colour image, the intrinsics and the camera pose
(D-025): nothing from the simulator. The nest is fixed infrastructure: its frame (origin on its
floor, z up) and the cube's size are given.

    depth -> points in the nest frame -> points near the cube-top height: the top face
    top face -> smallest rectangle around it: centre, yaw (mod 90 degrees), sides; plane: tilt
    points lower than the top face and outside its outline: a layer that is out of square
    colour image at the nine sticker centres of the top face: their colours

What the colours mean (which face, which cube state) is not decided here: a view sees one face
of six.
"""

import math
from typing import NamedTuple

import cv2
import numpy as np

from . import camera


class CubeView(NamedTuple):
    present: bool          # a top face was seen at the height of a cube standing in the nest
    center: tuple          # cube centre (x, y, z) in the nest frame, m
    yaw: float             # rotation of the top face about the nest z axis, rad in (-pi/4, pi/4]
    size: tuple            # the two sides of the top face, m
    tilt: float            # angle of the top face from horizontal, rad
    roughness: float       # RMS distance of the top face points from their plane, m
    protrusion: float      # area seen below the top face and outside its outline, m^2
    stickers: np.ndarray   # 3 x 3 x 3 uint8 RGB as seen from above: row 0 is the +y side of
    #                        the top face, column 0 its -x side (axes turned by `yaw`)
    points: int            # depth pixels on the top face


ABSENT = CubeView(False, (0.0, 0.0, 0.0), 0.0, (0.0, 0.0), 0.0, 0.0, 0.0,
                  np.zeros((3, 3, 3), np.uint8), 0)


def sticker_centers(center, yaw, pitch):
    """Centres of the nine stickers of the top face in the nest frame, 3 x 3 x 3 (rows from
    the +y side, columns from the -x side, axes turned by `yaw`)."""
    c, s = math.cos(yaw), math.sin(yaw)
    grid = np.empty((3, 3, 3))
    for row in range(3):
        for col in range(3):
            dx, dy = (col - 1) * pitch, (1 - row) * pitch
            grid[row, col] = (center[0] + c * dx - s * dy, center[1] + s * dx + c * dy,
                              center[2] + 1.5 * pitch)
    return grid


def find(depth, color, intrinsics, nest_from_optical, pitch, params):
    """CubeView of the cube standing in the nest; `ABSENT` (with the points counted) if no
    top face is seen at cube height.

    `depth`: H x W z-depth in metres (NaN invalid); `color`: H x W x 3 uint8 RGB taken with
    the same optical centre, or None (the stickers are then black); `pitch`: side of a cubie.
    """
    side = 3 * pitch
    optical, pixels = camera.depth_to_points(depth, intrinsics, *params['depth_range'])
    points = camera.transform_points(nest_from_optical, optical)
    near = ((np.abs(points[:, 0]) <= params['region']) & (np.abs(points[:, 1]) <= params['region'])
            & (points[:, 2] > params['rim_height']) & (points[:, 2] < side + params['region']))
    points, z_optical = points[near], optical[near, 2]
    on_top = np.abs(points[:, 2] - side) <= params['top_tolerance']
    top = points[on_top]
    if len(top) < params['min_points']:
        return ABSENT._replace(points=len(top))

    (cx, cy), (w, h), angle = cv2.minAreaRect(top[:, :2].astype(np.float32))
    yaw = math.radians(angle)
    yaw -= math.floor(yaw / (math.pi / 2) + 0.5) * (math.pi / 2)   # into (-45, 45] degrees
    if abs(math.cos(math.radians(angle) - yaw)) < 0.5:
        w, h = h, w                                                # sides along the turned x, y
    normal, centroid, roughness = camera.fit_plane(top)
    tilt = math.acos(min(1.0, normal[2]))
    center = (float(cx), float(cy), float(centroid[2] - side / 2))

    # points below the top face that are not under it: a lower layer turned out of square
    below = points[~on_top & (points[:, 2] < side - params['top_tolerance'])]
    c, s = math.cos(yaw), math.sin(yaw)
    dx = c * (below[:, 0] - cx) + s * (below[:, 1] - cy)
    dy = -s * (below[:, 0] - cx) + c * (below[:, 1] - cy)
    outside = ((np.abs(dx) > w / 2 + params['outline_margin'])
               | (np.abs(dy) > h / 2 + params['outline_margin']))
    pixel_area = (z_optical[~on_top & (points[:, 2] < side - params['top_tolerance'])][outside]
                  ** 2 / (intrinsics.fx * intrinsics.fy))
    protrusion = float(pixel_area.sum())

    stickers = np.zeros((3, 3, 3), np.uint8)
    if color is not None:
        centers = sticker_centers(center, yaw, pitch).reshape(-1, 3)
        in_optical = camera.transform_points(np.linalg.inv(nest_from_optical), centers)
        uv = np.rint(camera.project(in_optical, intrinsics)).astype(int)
        half = np.maximum(1, np.rint(
            params['sticker_patch'] * pitch * intrinsics.fx / in_optical[:, 2] / 2).astype(int))
        height, width = color.shape[:2]
        for index, ((u, v), k) in enumerate(zip(uv, half)):
            patch = color[max(0, v - k):min(height, v + k + 1),
                          max(0, u - k):min(width, u + k + 1)].reshape(-1, 3)
            if len(patch):
                stickers[index // 3, index % 3] = np.median(patch, axis=0)
    return CubeView(True, center, yaw, (float(w), float(h)), tilt, roughness, protrusion,
                    stickers, len(top))
