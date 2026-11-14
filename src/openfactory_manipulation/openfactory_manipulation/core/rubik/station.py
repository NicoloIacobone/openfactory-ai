"""Tool poses of the Rubik's cube cell, in the nest frame (z up, origin on the nest floor).

The gripper always holds the cube the same way: its tool axis through the cube centre, the
fingers on two opposite faces, their tips `tip_depth` past the centre. With the tool pointing
down the tips are then `tip_margin` above the top of the bottom layer, so the pads hold the
two layers above it and never touch the one the nest holds. A grasp is named by two unit
vectors of the nest frame: the direction the tool points and the axis the fingers close along.

Poses are (xyz, quaternion xyzw) of `gripper_tcp`; `core.pick_place.tool0_from_tcp` gives the
`tool0` pose MoveIt is asked for.
"""

import math

DOWN = (0.0, 0.0, -1.0)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quat_from_axes(x, y, z):
    """Quaternion of the rotation whose columns are the unit vectors x, y, z."""
    trace = x[0] + y[1] + z[2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((y[2] - z[1]) / s, (z[0] - x[2]) / s, (x[1] - y[0]) / s, 0.25 * s)
    if x[0] > y[1] and x[0] > z[2]:
        s = 2.0 * math.sqrt(1.0 + x[0] - y[1] - z[2])
        return (0.25 * s, (y[0] + x[1]) / s, (z[0] + x[2]) / s, (y[2] - z[1]) / s)
    if y[1] > z[2]:
        s = 2.0 * math.sqrt(1.0 + y[1] - x[0] - z[2])
        return ((y[0] + x[1]) / s, 0.25 * s, (z[1] + y[2]) / s, (z[0] - x[2]) / s)
    s = 2.0 * math.sqrt(1.0 + z[2] - x[0] - y[1])
    return ((z[0] + x[2]) / s, (z[1] + y[2]) / s, 0.25 * s, (x[1] - y[0]) / s)


def tool_quat(tool_axis, finger_axis):
    """Orientation of the tool frame: z along `tool_axis`, y along `finger_axis`."""
    return quat_from_axes(_cross(finger_axis, tool_axis), finger_axis, tool_axis)


def rotate_about(axis, angle, v):
    """`v` turned by `angle` (rad, right-handed) about the unit vector `axis`."""
    c, s = math.cos(angle), math.sin(angle)
    k = sum(a * b for a, b in zip(axis, v)) * (1.0 - c)
    cross = _cross(axis, v)
    return tuple(v[i] * c + cross[i] * s + axis[i] * k for i in range(3))


class Station:
    """Geometry of the grasps on a cube of side 3 x `pitch` seated in the nest."""

    def __init__(self, pitch, gripper, tip_margin):
        self.pitch = pitch
        self.side = 3.0 * pitch
        # gripper_tcp to the fingertips, along the tool axis
        finger, palm = gripper['finger'], gripper['palm']
        self.tcp_to_tip = palm['size'][2] + finger['size'][2] - gripper['tcp_offset']
        self.tip_depth = 0.5 * pitch - tip_margin
        self.seat = (0.0, 0.0, 1.5 * pitch)        # centre of a cube seated in the nest

    def tip_height(self):
        """Fingertips above the nest floor in the grasp from above."""
        return self.seat[2] - self.tip_depth

    def tcp_pose(self, center, tool_axis, finger_axis):
        """TCP pose that holds a cube whose centre is at `center`."""
        back = self.tcp_to_tip - self.tip_depth
        xyz = tuple(c - back * a for c, a in zip(center, tool_axis))
        return xyz, tool_quat(tool_axis, finger_axis)

    def lifted(self, height):
        """Centre of a cube held `height` above its seat."""
        return (self.seat[0], self.seat[1], self.seat[2] + height)

    def retreat(self, pose, distance):
        """`pose` moved back `distance` along its own tool axis."""
        xyz, quat = pose
        x, y, z, w = quat
        axis = (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))
        return tuple(p - distance * a for p, a in zip(xyz, axis)), quat

    def arc(self, center, finger_axis, angle, steps):
        """TCP poses that tip the held cube about `finger_axis` through `center`, from the
        tool pointing down to `angle` (rad, right-handed about the finger axis)."""
        return [self.tcp_pose(center, rotate_about(finger_axis, angle * k / steps, DOWN),
                              finger_axis) for k in range(1, steps + 1)]

    def spin(self, center, finger_axis, angle, steps):
        """TCP poses that turn the held cube about the vertical through `center`, the tool
        pointing down: the finger axis goes from `finger_axis` to `angle` further."""
        return [self.tcp_pose(center, DOWN,
                              rotate_about((0.0, 0.0, 1.0), angle * k / steps, finger_axis))
                for k in range(1, steps + 1)]
