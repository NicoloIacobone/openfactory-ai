import math

import pytest

from openfactory_manipulation.core import grasps as gr
from openfactory_manipulation.core import task

LIMITS = {'match_radius': 0.02, 'match_angle': 0.2, 'max_attempts_per_object': 2,
          'attempts_per_object': 2.0, 'max_consecutive_failures': 3}
CUBE = {'shape': 'box', 'size': [0.05, 0.05, 0.05]}
CYLINDER = {'shape': 'cylinder', 'radius': 0.025, 'length': 0.08}
PRISM = {'shape': 'triangular_prism', 'side': 0.05, 'length': 0.06}


def grasp(x, y, finger_yaw):
    return gr.Grasp(0, 'cube', 'across_x', x, y, 0.025, finger_yaw, 0.05, 0.03)


def seen(x, y, name='cube'):
    return gr.SceneObject(0, name, 0.9, x, y, 0.0, (0.05, 0.05, 0.05))


@pytest.mark.parametrize('a, b, expected', [
    (0.0, 0.1, 0.1), (math.pi / 2, -math.pi / 2, 0.0), (1.5, -1.5, math.pi - 3.0),
    (0.0, math.pi / 2, math.pi / 2)])
def test_direction_difference_repeats_every_half_turn(a, b, expected):
    assert task.direction_difference(a, b) == pytest.approx(expected)


def test_a_failed_grasp_is_not_repeated_but_the_other_direction_is_tried():
    failures = [(0.05, 0.02, 0.3)]
    assert task.excluded(grasp(0.051, 0.02, 0.35), failures, LIMITS)
    assert task.excluded(grasp(0.05, 0.02, 0.3 - math.pi + 0.05), failures, LIMITS)
    assert not task.excluded(grasp(0.05, 0.02, 0.3 + math.pi / 2), failures, LIMITS)
    assert not task.excluded(grasp(0.12, 0.02, 0.3), failures, LIMITS)     # another object
    assert not task.excluded(grasp(0.05, 0.02, 0.3), [], LIMITS)


def test_an_object_is_left_alone_after_too_many_failed_grasps():
    failures = [(0.05, 0.02, 0.0), (0.052, 0.021, math.pi / 2)]
    assert task.excluded(grasp(0.05, 0.02, math.pi / 4), failures, LIMITS)
    assert not task.excluded(grasp(0.12, 0.02, math.pi / 4), failures, LIMITS)


def test_still_there():
    assert task.still_there(0.05, 0.02, [seen(0.06, 0.02)], 0.02)
    assert not task.still_there(0.05, 0.02, [seen(0.11, 0.02)], 0.02)
    assert not task.still_there(0.05, 0.02, [], 0.02)


@pytest.mark.parametrize('reason, removed, expected', [
    (None, True, (True, None)),
    (None, False, (False, 'object_not_removed')),
    (None, None, (False, 'no_view')),
    ('grasp_failed', False, (False, 'grasp_failed')),
    ('grasp_slipped', True, (False, 'grasp_slipped')),    # gone from the bin, but not placed
])
def test_attempt_outcome_needs_the_cycle_and_the_view(reason, removed, expected):
    assert task.attempt_outcome(reason, removed) == expected


def test_end_reasons_in_order():
    slots = [(0, (0.0, 0.0))]
    state = task.TaskState(max_attempts=task.attempt_budget(5, LIMITS))
    assert state.max_attempts == 10
    objects = [seen(0.0, 0.0)]
    assert task.end_reason([], [], state, slots, LIMITS) == 'bin_empty'
    assert task.end_reason(objects, objects, state, slots, LIMITS) is None
    assert task.end_reason(objects, [], state, slots, LIMITS) == 'only_unknown'
    assert task.end_reason(objects, objects, state, [], LIMITS) == 'output_full'
    state.consecutive_failures = 3
    assert task.end_reason(objects, objects, state, slots, LIMITS) == 'failure_budget'
    state.attempts = 10
    assert task.end_reason(objects, objects, state, slots, LIMITS) == 'attempt_budget'
    # an empty bin is success whatever was spent
    assert task.end_reason([], [], state, slots, LIMITS) == 'bin_empty'
    assert set(('bin_empty', 'only_unknown', 'output_full', 'failure_budget',
                'attempt_budget')) <= set(task.END_REASONS)


def test_free_slots_keep_the_configured_order():
    slots = [[0.1, 0.05], [0.0, 0.05], [-0.1, 0.05]]
    assert task.free_slots(slots, {}) == [(0, (0.1, 0.05)), (1, (0.0, 0.05)), (2, (-0.1, 0.05))]
    assert task.free_slots(slots, {1: {}}) == [(0, (0.1, 0.05)), (2, (-0.1, 0.05))]
    assert task.free_slots(slots, {0: {}, 1: {}, 2: {}}) == []


def test_obstacle_specs():
    assert task.obstacle_spec(CUBE, 0.015) == {'shape': 'box', 'size': [0.08, 0.08, 0.05]}
    assert task.obstacle_spec(CYLINDER, 0.015) == pytest.approx(
        {'shape': 'cylinder', 'radius': 0.04, 'length': 0.08})
    prism = task.obstacle_spec(PRISM, 0.01)
    assert prism['shape'] == 'box'
    assert prism['size'] == pytest.approx([0.08, 0.07, 0.05 * math.sqrt(3) / 2])
    assert task.unknown_spec((0.1, 0.04, 0.03)) == {'shape': 'box', 'size': [0.1, 0.04, 0.03]}
