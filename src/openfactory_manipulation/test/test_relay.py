"""Decisions of the relay loop of the ring (core.relay): pure logic, no ROS."""

import math

import pytest

from openfactory_manipulation.core import relay
from openfactory_manipulation.core.grasps import SceneObject

SLOTS = [[0.10, 0.055], [0.10, -0.055], [0.0, 0.055], [0.0, -0.055], [-0.10, 0.055],
         [-0.10, -0.055]]
MARGIN = 0.045


def _cube(x, y, index=0, size=0.05):
    return SceneObject(index, 'cube', 0.95, x, y, 0.0, (size, size, size))


def test_bins_of_an_arm():
    assert [relay.source_bin(arm) for arm in range(3)] == [0, 1, 2]
    assert [relay.destination_bin(arm, 3) for arm in range(3)] == [1, 2, 0]
    assert [relay.destination_bin(arm, 2) for arm in range(2)] == [1, 0]


def test_an_empty_bin_offers_every_place_point_in_the_configured_order():
    free = relay.free_slots(SLOTS, [], MARGIN)
    assert [index for index, _ in free] == list(range(len(SLOTS)))
    assert free[0] == (0, (0.10, 0.055))


def test_a_place_point_under_an_object_is_not_free():
    free = relay.free_slots(SLOTS, [_cube(0.10, 0.055)], MARGIN)
    assert 0 not in [index for index, _ in free]
    assert len(free) == len(SLOTS) - 1


def test_an_object_between_two_place_points_takes_both():
    free = relay.free_slots(SLOTS, [_cube(0.05, 0.055)], MARGIN)
    assert {0, 2}.isdisjoint(index for index, _ in free)


def test_the_roomiest_place_point_comes_first():
    # one object in a corner: the opposite corner has the most room
    free = relay.free_slots(SLOTS, [_cube(0.10, 0.055)], MARGIN)
    assert free[0][0] == 5
    rooms = [relay.room(xy, [_cube(0.10, 0.055)]) for _, xy in free]
    assert all(a >= b - 0.01 for a, b in zip(rooms, rooms[1:]))


def test_room_is_measured_to_the_footprint_circle():
    obj = _cube(0.0, 0.0)
    assert relay.footprint_radius(obj) == pytest.approx(0.05 * math.sqrt(2) / 2)
    assert relay.room((0.1, 0.0), [obj]) == pytest.approx(0.1 - relay.footprint_radius(obj))
    assert relay.room((0.0, 0.0), []) == math.inf


def test_a_full_bin_has_no_free_place_point():
    objects = [_cube(x, y, index) for index, (x, y) in enumerate(SLOTS)]
    assert relay.free_slots(SLOTS, objects, MARGIN) == []


def test_turns_go_round_the_ring_and_skip_arms_without_work():
    assert relay.next_arm(None, [True, True, True]) == 0
    assert relay.next_arm(0, [True, True, True]) == 1
    assert relay.next_arm(2, [True, True, True]) == 0
    assert relay.next_arm(0, [True, False, True]) == 2
    assert relay.next_arm(0, [True, False, False]) == 0      # the only arm with work, again
    assert relay.next_arm(1, [False, False, False]) is None
    assert relay.next_arm(None, [False, True]) == 1


def test_an_arm_needs_an_object_and_a_free_place_point():
    assert relay.can_work([_cube(0, 0)], [(0, (0.1, 0.055))])
    assert not relay.can_work([], [(0, (0.1, 0.055))])
    assert not relay.can_work([_cube(0, 0)], [])


def test_failed_grasps_are_forgotten_when_their_object_is_gone():
    failures = [(0.10, 0.05, 0.0), (-0.10, -0.05, 1.0)]
    objects = [_cube(0.105, 0.05)]
    assert relay.prune_failures(failures, objects, 0.02) == [failures[0]]
    assert relay.prune_failures(failures, [], 0.02) == []


def test_arrival_needs_an_object_at_the_place_point():
    assert relay.arrived((0.10, 0.055), [_cube(0.11, 0.05)], 0.03)
    assert not relay.arrived((0.10, 0.055), [_cube(0.0, 0.055)], 0.03)
    assert not relay.arrived((0.10, 0.055), [], 0.03)


def test_a_handover_counts_only_with_the_pick_and_the_arrival():
    assert relay.handover_outcome(True, None, True) == (True, None)
    assert relay.handover_outcome(True, None, False) == (False, relay.NOT_ARRIVED)
    assert relay.handover_outcome(True, None, None) == (False, 'no_view')
    assert relay.handover_outcome(False, 'grasp_failed', None) == (False, 'grasp_failed')
    assert relay.handover_outcome(False, 'grasp_slipped', True) == (False, 'grasp_slipped')


def test_summary_counts_handovers_per_arm_and_full_laps():
    def turn(arm, success, tried=True, reason=None):
        return {'arm': arm, 'attempt': {} if tried else None, 'success': success,
                'failure_reason': reason}

    records = [turn(0, True), turn(1, True), turn(0, True), turn(1, False, reason='grasp_failed'),
               turn(0, False, tried=False), turn(1, True)]
    summary = relay.summarize(records, 2)
    assert summary['turns'] == 6 and summary['skipped_turns'] == 1
    assert summary['handovers_tried'] == 5 and summary['handovers_done'] == 4
    assert summary['handovers_by_arm'] == [2, 2] and summary['laps'] == 2
    assert summary['failures'] == ['grasp_failed']
    assert relay.summarize([], 3)['laps'] == 0


def test_a_run_ends_when_asked_or_when_no_arm_finds_work():
    assert relay.run_end(None, 3, 10, 0, 2) is None
    assert relay.run_end(None, 10, 10, 0, 2) == 'handovers_done'
    assert relay.run_end(None, 3, None, 0, 2) is None
    assert relay.run_end(None, 3, 10, 2, 2) == 'no_work'
    assert relay.run_end(None, 3, 10, 2, 3) is None
    assert relay.run_end('park_failed', 10, 10, 0, 2) == 'park_failed'
    assert set(relay.END_REASONS) >= {'handovers_done', 'no_work', 'park_failed'}


def test_the_object_that_waited_longest_goes_first():
    arrivals = [(0.10, 0.055, 3), (0.0, -0.055, 7)]
    old, early, late = _cube(-0.10, 0.055, 0), _cube(0.105, 0.05, 1), _cube(0.0, -0.05, 2)
    assert relay.waiting_since(old, arrivals, 0.02) == -1
    assert relay.waiting_since(early, arrivals, 0.02) == 3
    assert relay.waiting_since(late, arrivals, 0.02) == 7
    order = sorted([late, old, early], key=lambda obj: relay.waiting_since(obj, arrivals, 0.02))
    assert [obj.index for obj in order] == [0, 1, 2]
    # a place point used twice: the latest arrival is the one standing there
    assert relay.waiting_since(early, arrivals + [(0.10, 0.055, 9)], 0.02) == 9


def test_arrivals_are_forgotten_when_their_object_left():
    arrivals = [(0.10, 0.055, 3), (0.0, -0.055, 7)]
    assert relay.prune_arrivals(arrivals, [_cube(0.10, 0.06)], 0.02) == [arrivals[0]]
    assert relay.prune_arrivals(arrivals, [], 0.02) == []


def test_nearest_object_to_a_point():
    objects = [_cube(0.10, 0.05, 0), _cube(0.11, 0.055, 1), _cube(-0.1, 0.0, 2)]
    assert relay.nearest((0.10, 0.055), objects, 0.03).index == 0
    assert relay.nearest((0.0, 0.0), objects, 0.03) is None
    assert relay.nearest((0.0, 0.0), [], 0.03) is None
