"""Rigid-body helpers on plain tuples. Quaternions are (x, y, z, w); poses are (xyz, quat)."""

import math


def quat_from_rpy(roll, pitch, yaw):
    """Quaternion for R = Rz(yaw) Ry(pitch) Rx(roll), the SDF/URDF convention."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 q_vec x v; v' = v + w t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def compose(parent, child):
    """Pose of `child` (given in the parent's frame) expressed in the parent's reference frame."""
    (pp, pq), (cp, cq) = parent, child
    offset = quat_rotate(pq, cp)
    return tuple(a + b for a, b in zip(pp, offset)), quat_multiply(pq, cq)


def yaw_of(q):
    """Rotation about z of a quaternion (exact when the rotation is about z only)."""
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(a):
    """Angle in (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi
