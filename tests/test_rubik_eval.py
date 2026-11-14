import rubik_eval as ev
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import rotations as rot

TOL = {'seat_offset_m': 0.002, 'seat_tilt_rad': 0.035}
SEATED = {'orientation': rot.IDENTITY, 'tilt_rad': 0.01, 'offset_m': (0.0005, -0.0003, 0.0)}
OFF = {'orientation': rot.IDENTITY, 'tilt_rad': 0.3, 'offset_m': (0.004, 0.0, 0.006)}
SCRAMBLED = c.apply_moves(c.SOLVED, "R U F'")


def result(solved, reason='solved'):
    return {'solved': solved, 'end_reason': reason}


def test_solved_needs_the_cell_the_simulator_and_no_collision():
    assert ev.judge_cube(result(True), c.SOLVED, True, SEATED, 0, TOL) == (True, None)
    assert ev.judge_cube(result(True), c.SOLVED, True, SEATED, 3, TOL) == (False, 'collision')


def test_the_cell_saying_solved_when_it_is_not_is_its_own_failure():
    assert ev.judge_cube(result(True), SCRAMBLED, True, SEATED, 0, TOL) == (
        False, 'false_solved')
    # solved colours, but a layer out of square or the cube out of its seat
    assert ev.judge_cube(result(True), c.SOLVED, False, SEATED, 0, TOL) == (
        False, 'false_solved')
    assert ev.judge_cube(result(True), c.SOLVED, True, OFF, 0, TOL) == (False, 'false_solved')
    assert ev.judge_cube(result(True), None, False, None, 0, TOL) == (False, 'false_solved')


def test_a_cube_the_cell_gave_up_on_fails_with_its_reason():
    assert ev.judge_cube(result(False, 'cube_lost'), SCRAMBLED, True, OFF, 0, TOL) == (
        False, 'cube_lost')
    # even if it happens to be solved: the cell did not know
    assert ev.judge_cube(result(False, 'scan_failed'), c.SOLVED, True, SEATED, 0, TOL) == (
        False, 'scan_failed')


def test_scan_judgement():
    other = rot.quarter_turn(rot.Z, 1)
    assert ev.judge_scan(SCRAMBLED, other, SCRAMBLED, other) == (True, None)
    assert ev.judge_scan(None, None, SCRAMBLED, other) == (False, 'scan_failed')
    assert ev.judge_scan(c.SOLVED, other, SCRAMBLED, other) == (False, 'wrong_state')
    assert ev.judge_scan(SCRAMBLED, rot.IDENTITY, SCRAMBLED, other) == (
        False, 'wrong_orientation')


def test_summary():
    cubes = [
        {'success': True, 'failure_reason': None, 'operations': 50, 'sim_s': 600.0,
         'solution': ['R'] * 20},
        {'success': False, 'failure_reason': 'cube_lost', 'operations': 10, 'sim_s': 100.0,
         'solution': None},
        {'success': False, 'failure_reason': 'false_solved', 'operations': 40, 'sim_s': 500.0,
         'solution': ['R'] * 18},
    ]
    operations = [
        {'kind': 'turn', 'truth_ok': True, 'belief_ok': True, 'cell_success': True,
         'total_s': 10.0, 'states_checked': 100, 'states_in_collision': 0},
        {'kind': 'flip', 'truth_ok': False, 'belief_ok': False, 'cell_success': True,
         'total_s': 16.0, 'states_checked': 150, 'states_in_collision': 0},
        {'kind': 'flip', 'truth_ok': True, 'belief_ok': None, 'cell_success': False,
         'total_s': 14.0, 'states_checked': 120, 'states_in_collision': 2},
        {'kind': 'assemble'},
    ]
    s = ev.summarize(cubes, operations)
    assert (s['cubes'], s['successes'], s['false_solved']) == (3, 1, 1)
    assert s['failures'] == {'cube_lost': 1, 'false_solved': 1}
    assert s['operations'] == 3 and s['undetected_failures'] == 1
    assert s['by_kind']['flip'] == {'count': 2, 'correct': 1, 'cell_failures': 1, 'mean_s': 15.0}
    assert s['states_checked'] == 370 and s['states_in_collision'] == 2
    assert s['mean_solution_moves'] == 19.0
    assert ev.summarize([], [])['success_rate'] is None
