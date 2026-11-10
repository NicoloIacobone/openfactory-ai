"""ROS-free pick-and-place planning for top-down grasps of catalog primitives.

All poses are (xyz, quat xyzw) in the world frame unless stated. TCP poses refer to
`gripper_tcp`; MoveIt goals are given for `tool0` (tool0_from_tcp). Phase 2 grasps close the
fingers across the object x axis: cube and cylinder in any case, the triangular prism on its
two parallel triangular end faces.
"""

import math
from dataclasses import dataclass

from .geometry import compose, quat_from_rpy, quat_multiply, wrap_angle, yaw_of
from .objects import bounding_box

# Failure reasons logged per attempt (docs/agent-guides/manipulation.md)
FAILURE_REASONS = (
    'scene_failed', 'ik_failed', 'planning_failed', 'execution_failed', 'gripper_failed',
    'grasp_failed', 'grasp_slipped', 'place_failed', 'timeout',
)

MOVEIT_SUCCESS = 1
_MOVEIT_IK = {-31}                                            # NO_IK_SOLUTION
# PLANNING_FAILED, INVALID_MOTION_PLAN, start/goal state and constraint errors, and FAILURE
# (99999), which move_group returns when a response adapter such as ValidateSolution rejects a plan
_MOVEIT_PLANNING = {-1, -2, -10, -11, -12, -13, -14, -16, -26, -27, 99999}
_MOVEIT_EXECUTION = {-3, -4, -7}                              # invalidated, control failed, preempted
_MOVEIT_TIMEOUT = {-6}

# tool z pointing down: rotation of pi about x
_DOWN = quat_from_rpy(math.pi, 0.0, 0.0)
_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def moveit_failure(code):
    """None for MoveIt SUCCESS, otherwise the failure reason for a MoveItErrorCodes value."""
    if code == MOVEIT_SUCCESS:
        return None
    if code in _MOVEIT_IK:
        return 'ik_failed'
    if code in _MOVEIT_PLANNING:
        return 'planning_failed'
    if code in _MOVEIT_EXECUTION:
        return 'execution_failed'
    if code in _MOVEIT_TIMEOUT:
        return 'timeout'
    return f'moveit_error_{code}'


def top_down_quat(object_yaw):
    """TCP orientation: tool z down, finger axis (tool y) along the object x axis.

    The parallel gripper is symmetric under a half turn about the tool axis; of the two
    equivalent orientations the one with tool yaw in [-pi/2, pi/2] is returned.
    """
    yaw = wrap_angle(object_yaw + math.pi / 2)
    if yaw > math.pi / 2:
        yaw -= math.pi
    elif yaw < -math.pi / 2:
        yaw += math.pi
    return quat_multiply(quat_from_rpy(0.0, 0.0, yaw), _DOWN)


def tool0_from_tcp(tcp_pose, tcp_offset):
    """tool0 pose for a gripper_tcp pose (gripper_tcp is tcp_offset along tool0 z)."""
    return compose(tcp_pose, ((0.0, 0.0, -tcp_offset), _IDENTITY))


def object_pose_in_bin(bin_pose, x, y, yaw, spec):
    """World pose of an object resting at (x, y, yaw) on the floor of a bin (bin frame pose)."""
    height = bounding_box(spec)[2]
    return compose(bin_pose, ((x, y, height / 2), quat_from_rpy(0.0, 0.0, yaw)))


def finger_geometry(gripper):
    """(fingertips below the TCP, palm face above the TCP) along the tool axis, metres."""
    palm, finger = gripper['palm']['size'][2], gripper['finger']['size'][2]
    return palm + finger - gripper['tcp_offset'], gripper['tcp_offset'] - palm


def grasp_offset(spec, gripper, params):
    """Height of the grasp TCP above the object centre.

    The TCP goes to the object centre unless the fingertips would come closer than
    `floor_clearance` to the surface the object rests on, or the palm closer than
    `palm_clearance` to the object top; then it is raised just enough.
    """
    height = bounding_box(spec)[2]
    tip_below, palm_above = finger_geometry(gripper)
    offset = max(0.0,
                 tip_below - height / 2 + params['floor_clearance'],
                 height / 2 + params['palm_clearance'] - palm_above)
    if offset - tip_below >= height / 2:
        raise ValueError('object too short: the fingertips would stay above it')
    return offset


@dataclass(frozen=True)
class PickPlacePlan:
    """TCP waypoints (world) and gripper commands (left finger position, m) for one cycle."""

    object_pose: tuple
    place_object_pose: tuple
    pre_grasp: tuple
    grasp: tuple
    lift: tuple
    pre_place: tuple
    place: tuple
    retreat: tuple
    grasp_width: float
    open_position: float
    preclose_position: float
    close_position: float

    @property
    def held_position(self):
        """Finger position expected while holding the object (half the grasp width)."""
        return self.grasp_width / 2


def raised(pose, dz):
    """The same pose moved dz along the world z axis."""
    (x, y, z), q = pose
    return (x, y, z + dz), q


def plan_pick_place(object_pose, place_bin_pose, place_xy, spec, gripper, params):
    """Waypoints for a top-down pick of `object_pose` and a place at `place_xy` in a bin.

    The object keeps its world yaw (same TCP orientation for grasp and place) and is released
    `release_clearance` above the destination floor.
    """
    width = bounding_box(spec)[0]
    height = bounding_box(spec)[2]
    stroke = gripper['finger']['stroke']
    half = width / 2
    if half + params['preclose_margin'] > stroke:
        raise ValueError(f'object width {width} m does not fit the gripper opening')

    yaw = yaw_of(object_pose[1])
    tcp_quat = top_down_quat(yaw)
    offset = grasp_offset(spec, gripper, params)

    grasp = raised((object_pose[0], tcp_quat), offset)
    place_centre, _ = compose(
        place_bin_pose,
        ((place_xy[0], place_xy[1], height / 2 + params['release_clearance']), _IDENTITY))
    place_object = (place_centre, quat_from_rpy(0.0, 0.0, yaw))
    place = raised((place_centre, tcp_quat), offset)

    return PickPlacePlan(
        object_pose=object_pose,
        place_object_pose=place_object,
        pre_grasp=raised(grasp, params['approach_distance']),
        grasp=grasp,
        lift=raised(grasp, params['lift_height']),
        pre_place=raised(place, params['lift_height']),
        place=place,
        retreat=raised(place, params['approach_distance']),
        grasp_width=width,
        open_position=stroke,
        preclose_position=half + params['preclose_margin'],
        close_position=max(0.0, half - params['squeeze_depth']),
    )


def check_grasp(finger_position, plan, tolerance):
    """None if the fingers stopped on the object, else 'grasp_failed' (missed or blocked)."""
    if finger_position is None or abs(finger_position - plan.held_position) > tolerance:
        return 'grasp_failed'
    return None


def check_hold(finger_position, plan, tolerance):
    """None if the object is still between the fingers, else 'grasp_slipped'."""
    if finger_position is None or abs(finger_position - plan.held_position) > tolerance:
        return 'grasp_slipped'
    return None
