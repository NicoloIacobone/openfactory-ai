"""ROS-free scene and task state of the bin-picking loop (scene_manager, phase 4, D-017).

The loop is: view of the input bin -> grasp selection -> pick and place -> next view, which
both verifies the last pick and feeds the next one. This module holds the decisions that need
no ROS: what the cell remembers between attempts (failed grasps, objects it placed), when a
pick counts as verified, when to stop, and where to place. Everything the cell knows comes
from perception and from its own actions, never from the simulator.
"""

import math
from dataclasses import dataclass, field

from .objects import bounding_box

# Why a scene ends; `bin_empty` is the only one that means the job is done
END_REASONS = ('bin_empty', 'no_view', 'only_unknown', 'no_feasible_grasp', 'output_full',
               'attempt_budget', 'failure_budget')
# Failure reasons the loop adds to those of the pick-and-place cycle
OBJECT_NOT_REMOVED = 'object_not_removed'


@dataclass
class TaskState:
    """What the cell remembers within one scene (one input bin, one empty output bin)."""

    attempts: int = 0
    consecutive_failures: int = 0
    max_attempts: int = None                       # fixed at the first view of the scene
    failures: list = field(default_factory=list)   # [(x, y, finger_yaw)] of failed grasps
    placed: dict = field(default_factory=dict)     # slot index -> {'spec', 'pose'}


def direction_difference(a, b):
    """Angle between two finger directions, which repeat every half turn: in [0, pi/2]."""
    difference = math.fmod(abs(a - b), math.pi)
    return min(difference, math.pi - difference)


def excluded(grasp, failures, limits):
    """True if this grasp should not be tried (again) in this scene.

    A grasp that failed is not repeated (same place within `match_radius`, same finger
    direction within `match_angle`), and an object is left alone after
    `max_attempts_per_object` failed grasps on it.
    """
    here = [(x, y, yaw) for x, y, yaw in failures
            if math.hypot(x - grasp.x, y - grasp.y) <= limits['match_radius']]
    if len(here) >= limits['max_attempts_per_object']:
        return True
    return any(direction_difference(yaw, grasp.finger_yaw) <= limits['match_angle']
               for _, _, yaw in here)


def still_there(x, y, objects, radius):
    """True if a view shows something within `radius` of where the picked object was."""
    return any(math.hypot(obj.x - x, obj.y - y) <= radius for obj in objects)


def attempt_outcome(manipulation_reason, removed):
    """(success, failure_reason) of one attempt as the cell itself can tell.

    `manipulation_reason`: None if every stage of the cycle and the finger checks were fine;
    `removed`: whether the next view shows the object gone from the input bin (None without a
    view). Success needs both sources (docs/agent-guides/manipulation.md).
    """
    if manipulation_reason is not None:
        return False, manipulation_reason
    if removed is None:
        return False, 'no_view'
    if not removed:
        return False, OBJECT_NOT_REMOVED
    return True, None


def attempt_budget(n_objects, limits):
    """Attempts allowed for a scene that showed `n_objects` in its first view."""
    return math.ceil(limits['attempts_per_object'] * n_objects)


def end_reason(objects, pickable, state, free_slots, limits):
    """Why the scene ends before selecting a grasp, or None to go on.

    `objects`: everything the view shows in the input bin; `pickable`: those of a catalog type
    with enough confidence.
    """
    if not objects:
        return 'bin_empty'
    if state.attempts >= state.max_attempts:
        return 'attempt_budget'
    if state.consecutive_failures >= limits['max_consecutive_failures']:
        return 'failure_budget'
    if not pickable:
        return 'only_unknown'
    if not free_slots:
        return 'output_full'
    return None


def free_slots(slots, placed):
    """[(index, xy)] of the place points nothing was placed on yet, in the configured order."""
    return [(index, tuple(xy)) for index, xy in enumerate(slots) if index not in placed]


def obstacle_spec(spec, padding):
    """Catalog-style shape of a placed object as an obstacle, grown by `padding` sideways: the
    object shifts when the fingers let go and the output bin is not observed."""
    if spec['shape'] == 'cylinder':
        return {'shape': 'cylinder', 'radius': spec['radius'] + padding,
                'length': spec['length']}
    x, y, z = bounding_box(spec)
    return {'shape': 'box', 'size': [x + 2 * padding, y + 2 * padding, z]}


def unknown_spec(size):
    """Box standing for a cluster perception could not name."""
    return {'shape': 'box', 'size': list(size)}
