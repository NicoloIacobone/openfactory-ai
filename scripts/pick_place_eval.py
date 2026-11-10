"""ROS-free evaluation of a pick-and-place trial from simulator ground truth (harness only).

Success needs evidence from several sources (docs/agent-guides/manipulation.md): the
manipulation result (MoveIt and controller outcomes, finger checks), the object's final pose
from the simulator, and the executed arm states re-checked for collisions.
"""

import math

from openfactory_manipulation.core.geometry import quat_rotate
from openfactory_manipulation.core.objects import bounding_box

import scene_objects as so


def distance(a, b):
    return math.dist(a, b)


def tilt(quat):
    """Angle between the object z axis and the world z axis (rad)."""
    z = quat_rotate(quat, (0.0, 0.0, 1.0))[2]
    return math.acos(max(-1.0, min(1.0, z)))


def in_bin(xyz, cell, bin_name):
    """True if the point lies over the bin's inner floor area."""
    origin = so.bin_origin(cell, bin_name)
    inner = cell['bins'][bin_name]['inner_size']
    return abs(xyz[0] - origin[0]) <= inner[0] / 2 and abs(xyz[1] - origin[1]) <= inner[1] / 2


def evaluate_place(final_pose, cell, spec, place_xy, tolerances):
    """Where the object ended up, relative to the output bin and the place point.

    `final_pose` is (xyz, quat) in the world frame, or None if the object is not in the world.
    """
    if final_pose is None:
        return {'object_found': False, 'in_output_bin': False, 'in_input_bin': False,
                'resting_upright': False, 'place_error_m': None, 'final_xyz': None}
    xyz, quat = final_pose
    out = so.bin_origin(cell, 'output')
    target = (out[0] + place_xy[0], out[1] + place_xy[1])
    resting_z = out[2] + bounding_box(spec)[2] / 2
    upright = (tilt(quat) <= tolerances['max_tilt_rad']
               and abs(xyz[2] - resting_z) <= tolerances['rest_height_m'])
    in_output = in_bin(xyz, cell, 'output')
    return {
        'object_found': True,
        'in_output_bin': in_output and upright,
        'in_input_bin': in_bin(xyz, cell, 'input'),
        'resting_upright': upright,
        'place_error_m': distance(xyz[:2], target),
        'final_xyz': list(xyz),
    }


def outcome(manipulation, evaluation, states_invalid):
    """(success, failure_reason) for one attempt; the first failing source wins."""
    if not manipulation['success']:
        return False, manipulation['failure_reason']
    if not evaluation['in_output_bin']:
        return False, 'place_failed'
    if states_invalid > 0:
        return False, 'collision'
    return True, None
