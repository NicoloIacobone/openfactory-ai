"""ROS-free evaluation of how much a held object turns between the gripper pads (harness only).

The poses are simulator ground truth sampled by the probe harness (scripts/grasp_hold_probe.py)
for measurement and debugging; nothing here is seen by perception or manipulation (rule 5).
The parallel gripper closes across the object x axis, so a slip between the pads is a rotation
of the object about its own x axis.
"""

import math

from openfactory_manipulation.core.geometry import quat_multiply, quat_rotate


def relative_rotation(initial_quat, quat):
    """Rotation since `initial_quat`, expressed in the initial object frame (quaternion xyzw)."""
    x, y, z, w = initial_quat
    return quat_multiply((-x, -y, -z, w), quat)


def roll_about_x(initial_quat, quat):
    """Rotation of the object about its own x axis since `initial_quat`, in (-pi, pi]."""
    _, y, z = quat_rotate(relative_rotation(initial_quat, quat), (0.0, 0.0, 1.0))
    return math.atan2(-y, z)


def rotation_angle(initial_quat, quat):
    """Total rotation since `initial_quat` about any axis, in [0, pi]."""
    w = abs(relative_rotation(initial_quat, quat)[3])
    return 2.0 * math.acos(min(1.0, w))


def face_index(roll, period):
    """Which of the equivalent resting faces the object is on: 0 for the initial one.

    `period` is the object's rotational symmetry about its x axis (2 pi / 3 for the
    equilateral prism, pi / 2 for the cube)."""
    return round(roll / period) % round(2 * math.pi / period)


def summarize_hold(samples, period, air_margin):
    """In-hand rotation over one pick-and-place cycle.

    `samples`: [(t, xyz, quat)] in time order, the first one taken with the object at rest
    before the cycle and the last one after it; `air_margin`: the object counts as held while
    its origin is this far above where it started.
    """
    if not samples:
        return None
    _, (_, _, z0), q0 = samples[0]
    held = [(t, xyz, quat) for t, xyz, quat in samples if xyz[2] > z0 + air_margin]
    rolls = [roll_about_x(q0, quat) for _, _, quat in held]
    final_roll = roll_about_x(q0, samples[-1][2])
    return {
        'samples': len(samples),
        'samples_held': len(held),
        'max_roll_held_rad': max((abs(r) for r in rolls), default=None),
        'roll_before_release_rad': rolls[-1] if rolls else None,
        'max_rotation_held_rad': max((rotation_angle(q0, quat) for _, _, quat in held),
                                     default=None),
        'final_roll_rad': final_roll,
        'final_face': face_index(final_roll, period),
        'face_changed': face_index(final_roll, period) != 0,
    }


def deep_merge(base, overrides):
    """A copy of nested dict `base` with `overrides` applied key by key."""
    merged = dict(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged
