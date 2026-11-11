"""Pinhole camera geometry on NumPy arrays. ROS-free.

Conventions: optical frame (z along the view, x right, y down), z-depth in metres, pixel
(u, v) = (column, row). Intrinsics are (fx, fy, cx, cy) and always come from camera_info.
"""

import math
from typing import NamedTuple

import numpy as np


class Intrinsics(NamedTuple):
    fx: float
    fy: float
    cx: float
    cy: float


def intrinsics_from_k(k, principal_point_offset=(0.0, 0.0)):
    """Intrinsics from the row-major 3x3 K of sensor_msgs/CameraInfo.

    `principal_point_offset` (pixels) is a calibration correction added to (cx, cy): this
    module puts pixel centres at integer coordinates, and a camera that reports its principal
    point with another convention is off by a constant.
    """
    return Intrinsics(float(k[0]), float(k[4]), float(k[2]) + principal_point_offset[0],
                      float(k[5]) + principal_point_offset[1])


def pinhole_intrinsics(width, height, hfov_rad):
    """Intrinsics of an ideal camera with square pixels and a centred principal point."""
    f = (width / 2.0) / math.tan(hfov_rad / 2.0)
    return Intrinsics(f, f, width / 2.0, height / 2.0)


def valid_depth(depth, z_min, z_max):
    """Mask of usable pixels: finite and inside [z_min, z_max] (rejects NaN, inf and 0)."""
    depth = np.asarray(depth)
    with np.errstate(invalid='ignore'):
        return np.isfinite(depth) & (depth >= z_min) & (depth <= z_max)


def depth_to_points(depth, intrinsics, z_min, z_max):
    """(points N x 3 in the optical frame, pixel indices N x 2 as (v, u)) of the valid pixels."""
    depth = np.asarray(depth, dtype=np.float64)
    v, u = np.nonzero(valid_depth(depth, z_min, z_max))
    z = depth[v, u]
    x = (u - intrinsics.cx) / intrinsics.fx * z
    y = (v - intrinsics.cy) / intrinsics.fy * z
    return np.column_stack((x, y, z)), np.column_stack((v, u))


def project(points, intrinsics):
    """Pixel coordinates (u, v), N x 2, of points in the optical frame (z > 0)."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    u = intrinsics.fx * points[:, 0] / points[:, 2] + intrinsics.cx
    v = intrinsics.fy * points[:, 1] / points[:, 2] + intrinsics.cy
    return np.column_stack((u, v))


def rotation_from_quat(quat):
    """3 x 3 rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_rotation(rotation):
    """Unit quaternion (x, y, z, w) of a 3 x 3 rotation matrix, w >= 0."""
    m = np.asarray(rotation, dtype=np.float64)
    # the largest of the four squared components keeps the division well conditioned
    squares = np.array([1 + m[0, 0] - m[1, 1] - m[2, 2], 1 - m[0, 0] + m[1, 1] - m[2, 2],
                        1 - m[0, 0] - m[1, 1] + m[2, 2], 1 + m[0, 0] + m[1, 1] + m[2, 2]])
    i = int(np.argmax(squares))
    r = math.sqrt(squares[i]) * 2.0
    if i == 0:
        q = (r / 4, (m[0, 1] + m[1, 0]) / r, (m[0, 2] + m[2, 0]) / r, (m[2, 1] - m[1, 2]) / r)
    elif i == 1:
        q = ((m[0, 1] + m[1, 0]) / r, r / 4, (m[1, 2] + m[2, 1]) / r, (m[0, 2] - m[2, 0]) / r)
    elif i == 2:
        q = ((m[0, 2] + m[2, 0]) / r, (m[1, 2] + m[2, 1]) / r, r / 4, (m[1, 0] - m[0, 1]) / r)
    else:
        q = ((m[2, 1] - m[1, 2]) / r, (m[0, 2] - m[2, 0]) / r, (m[1, 0] - m[0, 1]) / r, r / 4)
    return tuple(-v for v in q) if q[3] < 0 else tuple(q)


def yaw_matrix(xyz, yaw):
    """4 x 4 pose at `xyz` rotated by `yaw` about z."""
    c, s = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    matrix[:3, 3] = xyz
    return matrix


def pose_matrix(xyz, quat):
    """4 x 4 homogeneous transform from a translation and a quaternion (x, y, z, w)."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_from_quat(quat)
    matrix[:3, 3] = xyz
    return matrix


def transform_points(matrix, points):
    """Apply a 4 x 4 transform to N x 3 points."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def fit_plane(points):
    """Least-squares plane through N x 3 points: (unit normal with n_z >= 0, centroid, RMS)."""
    points = np.asarray(points, dtype=np.float64)
    centroid = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[2] if vt[2, 2] >= 0 else -vt[2]
    return normal, centroid, float(singular[2] / math.sqrt(len(points)))
