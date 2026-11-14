"""The 24 rotations of a cube as integer matrices, and conversions from and to measured poses.

A matrix is a tuple of three rows; `mat_vec(m, v)` rotates the vector v. All rotations are
right-handed: `quarter_turn(axis, 1)` is +90 degrees about `axis`.
"""

import math

IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def mat_vec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def det(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def from_columns(columns):
    """Matrix whose j-th column is columns[j]."""
    return transpose(tuple(tuple(c) for c in columns))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quarter_turn(axis, turns):
    """Rotation of turns x 90 degrees about a signed unit axis such as (0, -1, 0)."""
    def rotate(v):
        # Rodrigues at 90 degrees: the component along the axis stays, the rest turns
        along = sum(a * c for a, c in zip(axis, v))
        return tuple(along * a + c for a, c in zip(axis, _cross(axis, v)))

    once = from_columns([rotate(e) for e in IDENTITY])
    result = IDENTITY
    for _ in range(turns % 4):
        result = mat_mul(once, result)
    return result


def _all_rotations():
    found = {IDENTITY}
    frontier = [IDENTITY]
    generators = [quarter_turn(axis, 1) for axis in (X, Y, Z)]
    while frontier:
        m = frontier.pop()
        for g in generators:
            n = mat_mul(g, m)
            if n not in found:
                found.add(n)
                frontier.append(n)
    return tuple(sorted(found))


ROTATIONS = _all_rotations()


def from_quaternion(q):
    """Rotation matrix (floats) of the unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def nearest_rotation(m):
    """(rotation among the 24 closest to the float matrix m, angle between them in radians)."""
    def trace_with(r):
        return sum(r[i][j] * m[i][j] for i in range(3) for j in range(3))

    best = max(ROTATIONS, key=trace_with)
    cosine = max(-1.0, min(1.0, (trace_with(best) - 1.0) / 2.0))
    return best, math.acos(cosine)


def rpy_of(m):
    """(roll, pitch, yaw) with m = Rz(yaw) Ry(pitch) Rx(roll), the SDF/URDF convention.

    At pitch = +-90 degrees roll and yaw turn about the same axis: roll is set to 0.
    """
    sin_pitch = max(-1.0, min(1.0, -m[2][0]))
    if abs(sin_pitch) > 1.0 - 1e-9:
        return 0.0, math.copysign(math.pi / 2, sin_pitch), math.atan2(-m[0][1], m[1][1])
    return (math.atan2(m[2][1], m[2][2]), math.asin(sin_pitch), math.atan2(m[1][0], m[0][0]))


def from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))
