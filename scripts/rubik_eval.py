"""Judgement of the Rubik's cube cell against ground truth (phase 7, E-007). ROS-free.

The cell's own verdict is `result` of `RubikSolver.run`; the simulator's is what the harness
read from the cube's link poses (`rubik_truth`, `rubik_scene.cube_in_nest`) after every
operation and at the end. A cube counts as solved only if both agree and no executed robot
state was in collision.
"""

import math

from openfactory_manipulation.core.rubik.cube import SOLVED

import trial_log as tl


def seated(where, tolerances):
    """True if the cube stands in its seat: `where` is rubik_scene.cube_in_nest."""
    return (where is not None
            and math.dist(where['offset_m'], (0.0, 0.0, 0.0)) <= tolerances['seat_offset_m']
            and where['tilt_rad'] <= tolerances['seat_tilt_rad'])


def truly_solved(truth_state, square, where, tolerances):
    """Ground truth: the solved state, every cubie on the grid, the cube in its seat."""
    return truth_state == SOLVED and bool(square) and seated(where, tolerances)


def judge_cube(result, truth_state, square, where, states_in_collision, tolerances):
    """(success, failure reason) of one cube.

    `false_solved` is the failure that must never happen: the cell says solved and the cube
    is not. A cube the cell gave up on fails with the cell's own end reason.
    """
    solved = truly_solved(truth_state, square, where, tolerances)
    if result['solved'] and not solved:
        return False, 'false_solved'
    if not result['solved']:
        return False, result['end_reason'] or 'not_solved'
    if states_in_collision:
        return False, 'collision'
    return True, None


def judge_scan(believed_state, believed_orientation, truth_state, truth_orientation):
    """(success, failure reason) of a scan: the state and the orientation the cell read."""
    if believed_state is None:
        return False, 'scan_failed'
    if believed_state != truth_state:
        return False, 'wrong_state'
    if believed_orientation != truth_orientation:
        return False, 'wrong_orientation'
    return True, None


def summarize(cubes, operations):
    """Summary of a run from its cube records and its operation records."""
    done = len(cubes)
    good = sum(c['success'] for c in cubes)
    reasons = {}
    for cube in cubes:
        if not cube['success']:
            reasons[cube['failure_reason']] = reasons.get(cube['failure_reason'], 0) + 1
    moved = [o for o in operations if o.get('kind') in ('turn', 'flip', 'realign')]
    by_kind = {}
    for kind in ('turn', 'flip', 'realign'):
        of_kind = [o for o in moved if o['kind'] == kind]
        if of_kind:
            by_kind[kind] = {
                'count': len(of_kind),
                'correct': sum(bool(o['truth_ok']) for o in of_kind),
                'cell_failures': sum(not o['cell_success'] for o in of_kind),
                'mean_s': sum(o['total_s'] for o in of_kind) / len(of_kind),
            }
    low, high = tl.wilson_interval(good, done) if done else (None, None)
    return {
        'cubes': done, 'successes': good, 'success_rate': good / done if done else None,
        'wilson_95': [low, high], 'failures': reasons,
        'false_solved': reasons.get('false_solved', 0),
        'operations': len(moved), 'by_kind': by_kind,
        # operations after which the cell believed something the simulator does not show
        'undetected_failures': sum(1 for o in moved if o.get('belief_ok') is False),
        'states_checked': sum(o.get('states_checked', 0) for o in moved),
        'states_in_collision': sum(o.get('states_in_collision', 0) for o in moved),
        'mean_operations': (sum(c['operations'] for c in cubes) / done) if done else None,
        'mean_sim_s': (sum(c['sim_s'] for c in cubes) / done) if done else None,
        'mean_solution_moves': (
            sum(len(c['solution']) for c in cubes if c.get('solution')) /
            max(1, sum(1 for c in cubes if c.get('solution')))),
    }
