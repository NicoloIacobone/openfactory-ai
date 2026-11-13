"""Ground-truth evaluation of relay-loop runs (scripts/relay_eval.py): pure logic."""

import math

import pytest

import relay_eval as re
import scene_objects as so

INNER = [0.30, 0.22, 0.10]
FLOOR = 0.76
# two bins as in the ring of two arms: rotated a quarter turn, facing each other
POSES = {
    'bin_0': ((0.30, 0.0, FLOOR), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))),
    'bin_1': ((-0.30, 0.0, FLOOR), (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))),
}
TOLERANCES = {'rest_motion_m': 0.001, 'disturbed_m': 0.005}
CUBE = {'shape': 'box', 'size': [0.05, 0.05, 0.05], 'mass': 0.1}
CYLINDER = {'shape': 'cylinder', 'radius': 0.025, 'length': 0.08, 'mass': 0.1}
THRESHOLDS = {'min_handover_success': 0.90, 'max_invalid_states': 0, 'max_objects_lost': 1,
              'max_undetected_failures': 3, 'min_handovers_per_arm': 1}


def test_a_rotated_bin_has_its_long_side_along_world_y():
    # bin x (0.30 long) points along world y for a quarter-turn bin
    assert re.location((0.30, 0.14, FLOOR + 0.02), POSES, INNER) == 'bin_0'
    assert re.location((0.30 + 0.14, 0.0, FLOOR + 0.02), POSES, INNER) == re.ELSEWHERE
    assert re.location((-0.30, -0.14, FLOOR + 0.02), POSES, INNER) == 'bin_1'
    assert re.location((0.0, 0.0, FLOOR + 0.02), POSES, INNER) == re.ELSEWHERE
    assert re.location((0.30, 0.0, FLOOR + 0.2), POSES, INNER) == re.ELSEWHERE      # above the rim
    assert re.location((0.30, 0.0, 0.3), POSES, INNER) == re.ELSEWHERE              # on the floor
    assert re.location(None, POSES, INNER) == re.MISSING


def test_in_bin_inverts_the_bin_rotation():
    assert re.in_bin(POSES['bin_0'], (0.30, 0.10, FLOOR)) == pytest.approx((0.10, 0.0, 0.0))
    assert re.in_bin(POSES['bin_1'], (-0.30, 0.10, FLOOR)) == pytest.approx((-0.10, 0.0, 0.0))


def test_spawn_pose_puts_the_object_where_it_is_then_found():
    xyz, rpy = re.spawn_pose(POSES['bin_0'], CUBE, 0.10, -0.05, 0.3)
    assert re.in_bin(POSES['bin_0'], xyz) == pytest.approx((0.10, -0.05, 0.025 + 0.002))
    assert rpy == pytest.approx((0.0, 0.0, math.pi / 2 + 0.3))
    assert re.location(xyz, POSES, INNER) == 'bin_0'
    # a lying cylinder keeps its tumbled roll and gets the bin yaw added
    xyz, rpy = re.spawn_pose(POSES['bin_1'], CYLINDER, 0.0, 0.0, 0.5, so.TUMBLED)
    assert rpy == pytest.approx((math.pi / 2, 0.0, -math.pi / 2 + 0.5))
    assert xyz[2] == pytest.approx(FLOOR + 0.025 + 0.002)


def test_scene_names_are_unique_across_bins():
    sampled = [{'name': 'cube_0', 'type': 'cube', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
    names = [item['name'] for index in range(3) for item in re.scene_names(index, sampled)]
    assert names == ['b0_cube_0', 'b1_cube_0', 'b2_cube_0']
    assert re.scene_names(2, sampled)[0]['bin'] == 2


def _state(location, xyz, at_rest=True, kind='cube'):
    return {'type': kind, 'location': location, 'at_rest': at_rest, 'xyz': list(xyz),
            'quat': [0.0, 0.0, 0.0, 1.0]}


def test_object_states_tell_bin_and_rest():
    objects = [{'name': 'a', 'type': 'cube'}, {'name': 'b', 'type': 'prism'},
               {'name': 'c', 'type': 'cube'}]
    quat = (0.0, 0.0, 0.0, 1.0)
    first = {'a': ((0.30, 0.0, 0.785), quat), 'b': ((0.0, 0.0, 0.8), quat)}
    second = {'a': ((0.30, 0.0, 0.785), quat), 'b': ((0.0, 0.0, 0.775), quat)}
    states = re.object_states(objects, first, second, POSES, INNER, TOLERANCES)
    assert states['a']['location'] == 'bin_0' and states['a']['at_rest']
    assert states['b']['location'] == re.ELSEWHERE and not states['b']['at_rest']
    assert states['c']['location'] == re.MISSING and states['c']['xyz'] is None


def test_the_target_is_matched_in_the_frame_of_its_bin():
    states = {'near': _state('bin_0', (0.30, 0.10, 0.785)),
              'far': _state('bin_0', (0.30, -0.10, 0.785)),
              'other_bin': _state('bin_1', (-0.30, 0.10, 0.785))}
    # (0.10, 0) in bin_0 is world (0.30, 0.10)
    assert re.match_target(0.10, 0.0, states, 'bin_0', POSES['bin_0'], 0.02) == 'near'
    assert re.match_target(0.105, 0.005, states, 'bin_0', POSES['bin_0'], 0.02) == 'near'
    assert re.match_target(0.0, 0.0, states, 'bin_0', POSES['bin_0'], 0.02) is None
    assert re.match_target(-0.10, 0.0, states, 'bin_1', POSES['bin_1'], 0.02) == 'other_bin'


def test_a_turn_is_judged_by_where_the_target_ends():
    before = {'t': _state('bin_0', (0.30, 0.10, 0.785)),
              'still': _state('bin_1', (-0.30, 0.05, 0.785)),
              'pushed': _state('bin_1', (-0.30, -0.05, 0.785))}
    after = {'t': _state('bin_1', (-0.30, 0.0, 0.785)),
             'still': _state('bin_1', (-0.30, 0.0505, 0.785)),
             'pushed': _state('bin_1', (-0.30, -0.07, 0.785))}
    evaluation = re.evaluate_turn('t', before, after, 'bin_1', TOLERANCES)
    assert evaluation['target_arrived'] and evaluation['target_location'] == 'bin_1'
    assert evaluation['disturbed'] == ['pushed'] and evaluation['lost'] == []

    dropped = {**after, 't': _state(re.ELSEWHERE, (0.0, 0.0, 0.775))}
    evaluation = re.evaluate_turn('t', before, dropped, 'bin_1', TOLERANCES)
    assert not evaluation['target_arrived'] and evaluation['lost'] == ['t']

    moving = {**after, 't': _state('bin_1', (-0.30, 0.0, 0.785), at_rest=False)}
    assert not re.evaluate_turn('t', before, moving, 'bin_1', TOLERANCES)['target_arrived']
    # a skipped turn has no target
    idle = re.evaluate_turn(None, before, before, 'bin_1', TOLERANCES)
    assert not idle['target_arrived'] and idle['disturbed'] == [] and idle['lost'] == []


def test_handover_result_needs_cell_truth_and_no_collision():
    arrived, absent = {'target_arrived': True}, {'target_arrived': False}
    assert re.handover_result(True, None, arrived, 0) == (True, None, 'agree')
    assert re.handover_result(True, None, absent, 0) == (False, 'place_failed',
                                                         'undetected_failure')
    assert re.handover_result(True, None, arrived, 2) == (False, 'collision', 'agree')
    assert re.handover_result(False, 'grasp_failed', absent, 0) == (False, 'grasp_failed',
                                                                    'agree')
    assert re.handover_result(False, 'object_not_arrived', arrived, 0) == (
        False, 'object_not_arrived', 'false_alarm')


def _handover(arm, success, detection='agree', reason=None):
    return {'arm': arm, 'success': success, 'failure_reason': None if success else reason or
            'grasp_failed', 'detection': detection, 'object_category': 'cube',
            'evaluation': {'target': f'o{arm}'},
            'execution_result': {'failed_stage': None if success else 'close'},
            'perception_s': 1.0, 'planning_s': 0.1, 'execution_s': 17.0, 'total_s': 25.0}


def _turn(skipped=None, invalid=0, parked=True, lost=()):
    return {'skipped': skipped, 'parked': parked, 'states_checked': 200,
            'states_invalid': invalid, 'disturbed': [], 'lost': list(lost), 'total_s': 25.0}


def test_summary_of_a_clean_run_passes():
    handovers = [_handover(arm % 2, True) for arm in range(20)]
    turns = [_turn() for _ in handovers] + [_turn(skipped='no_feasible_grasp')]
    final = {f'o{i}': _state(f'bin_{i % 2}', (0.3, 0.0, 0.785)) for i in range(6)}
    summary = re.summarize(handovers, turns, final, 2, 'handovers_done', 20, THRESHOLDS)
    assert summary['passed'] and all(summary['checks'].values())
    assert summary['handover_success']['successes'] == 20 and summary['laps'] == 10
    assert summary['skipped_turns'] == {'no_feasible_grasp': 1}
    assert summary['objects_lost'] == 0 and summary['objects_by_location'] == {
        'bin_0': 3, 'bin_1': 3}
    assert summary['handovers_per_hour'] == pytest.approx(3600 * 20 / (21 * 25.0))
    assert summary['objects_handed_over'] == 2 and summary['handovers_of_busiest_object'] == 10
    assert summary['states_checked'] == 21 * 200


def test_each_threshold_can_fail_the_run():
    good = [_handover(arm % 2, True) for arm in range(20)]
    final = {f'o{i}': _state('bin_0', (0.3, 0.0, 0.785)) for i in range(6)}

    def run(handovers=good, turns=None, states=final, end='handovers_done', n=2):
        turns = [_turn() for _ in handovers] if turns is None else turns
        return re.summarize(handovers, turns, states, n, end, 20, THRESHOLDS)['checks']

    assert not run(end='failure_budget')['ran_to_the_end']
    assert not run(handovers=good[:10])['ran_to_the_end']
    failing = good[:17] + [_handover(0, False)] * 3
    assert not run(handovers=failing)['handover_success']
    assert run(handovers=good[:18] + [_handover(0, False)] * 2)['handover_success']
    assert not run(turns=[_turn(invalid=1)] + [_turn()] * 19)['collisions']
    lost = {**final, 'o0': _state(re.ELSEWHERE, (0, 0, 0.775)),
            'o1': _state(re.MISSING, (0, 0, 0))}
    assert not run(states=lost)['objects_lost']
    assert run(states={**final, 'o0': _state(re.ELSEWHERE, (0, 0, 0.775))})['objects_lost']
    undetected = good[:16] + [_handover(0, False, 'undetected_failure', 'place_failed')] * 4
    assert not run(handovers=undetected)['undetected_failures']
    assert not run(handovers=[_handover(0, True)] * 20)['every_arm_works']
    assert not run(n=3)['every_arm_works']
