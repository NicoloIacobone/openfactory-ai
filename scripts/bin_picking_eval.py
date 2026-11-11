"""ROS-free evaluation of bin-picking trials from simulator ground truth (harness only).

The cell reports what it believes happened (openfactory_manipulation.bin_picking); here the
trial harness compares it with the object poses read from the simulator, which neither
perception nor manipulation ever sees (rule 5). An object counts as deposited when it rests
inside the output bin, on any face (D-017); whether it stands in its catalog orientation, the
criterion of E-002 and E-003, is reported next to it.
"""

import math
from collections import Counter

import pick_place_eval as ev
import scene_objects as so
import trial_log as tl
from openfactory_manipulation.core.objects import bounding_box

OUTPUT, INPUT, ELSEWHERE, MISSING = 'output_bin', 'input_bin', 'elsewhere', 'missing'
MOVEIT_REJECTIONS = ('ik_failed', 'approach_blocked', 'lift_blocked')
PLANNING_REASONS = ('ik_failed', 'planning_failed')


def location(xyz, cell):
    """Which bin volume a point is in: inner floor area, between the floor and the rim."""
    if xyz is None:
        return MISSING
    for name, label in (('output', OUTPUT), ('input', INPUT)):
        origin = so.bin_origin(cell, name)
        inner = cell['bins'][name]['inner_size']
        if (abs(xyz[0] - origin[0]) <= inner[0] / 2 and abs(xyz[1] - origin[1]) <= inner[1] / 2
                and 0.0 <= xyz[2] - origin[2] <= inner[2]):
            return label
    return ELSEWHERE


def object_states(objects, first, second, cell, catalog, tolerances):
    """Ground-truth state of every spawned object from two pose readings a moment apart.

    `objects`: the scene [{'name', 'type', ...}]; `first`, `second`: {name: (xyz, quat)}.
    Returns {name: {'type', 'location', 'at_rest', 'upright', 'xyz', 'quat'}}.
    """
    states = {}
    for item in objects:
        name, spec = item['name'], catalog['objects'][item['type']]
        pose, earlier = second.get(name), first.get(name)
        if pose is None:
            states[name] = {'type': item['type'], 'location': MISSING, 'at_rest': False,
                            'upright': False, 'xyz': None, 'quat': None}
            continue
        xyz, quat = pose
        where = location(xyz, cell)
        upright = False
        if where in (OUTPUT, INPUT):
            floor = so.bin_origin(cell, 'output' if where == OUTPUT else 'input')[2]
            upright = (ev.tilt(quat) <= tolerances['max_tilt_rad']
                       and abs(xyz[2] - floor - bounding_box(spec)[2] / 2)
                       <= tolerances['rest_height_m'])
        states[name] = {
            'type': item['type'], 'location': where,
            'at_rest': earlier is not None
            and math.dist(xyz, earlier[0]) <= tolerances['rest_motion_m'],
            'upright': upright, 'xyz': list(xyz), 'quat': list(quat)}
    return states


def deposited(state):
    return state['location'] == OUTPUT and state['at_rest']


def match_target(x, y, states, cell, max_distance):
    """Name of the object in the input bin nearest to (x, y) of the input_bin frame, where the
    cell aimed; None if nothing is within `max_distance`."""
    origin = so.bin_origin(cell, 'input')
    best, best_distance = None, max_distance
    for name, state in states.items():
        if state['location'] != INPUT:
            continue
        distance = math.hypot(state['xyz'][0] - origin[0] - x, state['xyz'][1] - origin[1] - y)
        if distance <= best_distance:
            best, best_distance = name, distance
    return best


def evaluate_attempt(target, before, after, tolerances):
    """What one attempt did to the scene.

    `target`: name of the object the cell aimed at (match_target on `before`) or None;
    `before`, `after`: object_states around the attempt.
    """
    disturbed = sorted(
        name for name, state in after.items()
        if name != target and before[name]['location'] == INPUT
        and (state['location'] != INPUT
             or math.dist(state['xyz'], before[name]['xyz']) > tolerances['disturbed_m']
             or (before[name]['upright'] and not state['upright'])))
    newly = sorted(name for name, state in after.items()
                   if deposited(state) and not deposited(before[name]))
    return {
        'target': target,
        'target_type': None if target is None else after[target]['type'],
        'target_location': None if target is None else after[target]['location'],
        'target_deposited': target is not None and deposited(after[target]),
        'target_upright': target is not None and after[target]['upright'],
        'newly_deposited': newly,
        'disturbed': disturbed,
        'lost': sorted(name for name, state in after.items()
                       if state['location'] in (ELSEWHERE, MISSING)
                       and before[name]['location'] not in (ELSEWHERE, MISSING)),
    }


def attempt_result(cell_success, cell_reason, evaluation, states_invalid):
    """(success, failure_reason, detection) of one attempt.

    Success needs every source: the cell's own verdict, the object deposited according to
    ground truth, and no executed state in collision. `detection` compares the cell's verdict
    with ground truth: 'agree', 'undetected_failure' (the cell believes a pick that did not
    deposit its object) or 'false_alarm' (the cell reports a failure but the object arrived).
    """
    arrived = evaluation['target_deposited']
    detection = ('agree' if cell_success == arrived
                 else 'undetected_failure' if cell_success else 'false_alarm')
    if not cell_success:
        return False, cell_reason, detection
    if not arrived:
        return False, 'place_failed', detection
    if states_invalid > 0:
        return False, 'collision', detection
    return True, None, detection


def scene_result(objects, final, end_reason, attempts):
    """Outcome of one scene from the final ground-truth states."""
    done = [name for name, state in final.items() if deposited(state)]
    by_type = {}
    for item in objects:
        entry = by_type.setdefault(item['type'], {'objects': 0, 'deposited': 0})
        entry['objects'] += 1
        entry['deposited'] += item['name'] in done
    return {
        'n_objects': len(objects), 'deposited': len(done),
        'completion': len(done) / len(objects) if objects else None,
        'upright': sum(final[name]['upright'] for name in done),
        'left_in_input': sorted(n for n, s in final.items() if s['location'] == INPUT),
        'lost': sorted(n for n, s in final.items() if s['location'] in (ELSEWHERE, MISSING)),
        'by_type': by_type, 'end_reason': end_reason, 'attempts': attempts,
        # the cell says the bin is empty exactly when it is
        'end_correct': (end_reason == 'bin_empty') == all(
            state['location'] != INPUT for state in final.values()),
    }


def recovered(record):
    """True if, after a failure it detected, the cell was back in a useful state on its own:
    arm at ready with a new view of the bin, whichever way it got there (the recovery of the
    cycle, or the straight escape of the loop)."""
    return record['verification']['view_stamp'] is not None


def _rate(successes, n):
    low, high = tl.wilson_interval(successes, n)
    return {'successes': successes, 'n': n, 'rate': successes / n if n else None,
            'ci95': [low, high]}


def summarize(attempts, scenes, thresholds):
    """Metrics of a run (docs/agent-guides/benchmarking.md) and whether it passes."""
    summary = tl.summarize(attempts)
    n_attempts = len(attempts)
    filled = [s for s in scenes if s['n_objects'] > 0]
    empty = [s for s in scenes if s['n_objects'] == 0]
    objects = sum(s['n_objects'] for s in filled)
    done = sum(s['deposited'] for s in filled)
    by_type = {}
    for scene in filled:
        for name, entry in scene['by_type'].items():
            total = by_type.setdefault(name, {'objects': 0, 'deposited': 0})
            total['objects'] += entry['objects']
            total['deposited'] += entry['deposited']

    detected = [r for r in attempts if not r['execution_result']['cell_success']]
    evaluated = sum(r['candidates_total'] for r in attempts)
    planning = (sum(count for r in attempts for reason, count in r['candidates_rejected'].items()
                    if reason in MOVEIT_REJECTIONS)
                + sum(r['execution_result']['cell_failure'] in PLANNING_REASONS
                      for r in attempts))
    collisions = sum(r['evaluation']['states_invalid'] > 0 for r in attempts)
    summary.update({
        'pick_success': _rate(sum(r['success'] for r in attempts), n_attempts),
        'completion': _rate(done, objects),
        'completion_by_type': {name: _rate(entry['deposited'], entry['objects'])
                               for name, entry in sorted(by_type.items())},
        'scenes': len(filled), 'scenes_cleared': sum(s['completion'] == 1.0 for s in filled),
        'end_reasons': dict(Counter(s['end_reason'] for s in scenes)),
        'end_correct': sum(s['end_correct'] for s in scenes),
        'empty_scenes': len(empty), 'attempts_in_empty_scenes': sum(s['attempts'] for s in empty),
        'deposited_upright': sum(s['upright'] for s in filled),
        'undetected_failures': sum(r['detection'] == 'undetected_failure' for r in attempts),
        'false_alarms': sum(r['detection'] == 'false_alarm' for r in attempts),
        'recovery': _rate(sum(recovered(r) for r in detected), len(detected)),
        'collision': _rate(collisions, n_attempts),
        'states_checked': sum(r['evaluation']['states_checked'] for r in attempts),
        'states_invalid': sum(r['evaluation']['states_invalid'] for r in attempts),
        'planning_failure': _rate(planning, evaluated),
        'objects_disturbed': sum(len(r['evaluation']['disturbed']) for r in attempts),
        'objects_lost': sum(len(s['lost']) for s in scenes),
        'escapes': sum(bool(r['verification'].get('escaped')) for r in attempts),
    })
    summary['checks'] = checks(summary, thresholds)
    summary['passed'] = all(summary['checks'].values())
    return summary


def checks(summary, thresholds):
    """Each closing-test threshold and whether the run meets it."""
    def at_least(rate, limit):
        return rate['rate'] is not None and rate['rate'] >= limit

    recovery = summary['recovery']
    return {
        'completion': at_least(summary['completion'], thresholds['min_completion']),
        'completion_by_type': bool(summary['completion_by_type']) and all(
            at_least(rate, thresholds['min_completion_per_type'])
            for rate in summary['completion_by_type'].values()),
        'pick_success': at_least(summary['pick_success'], thresholds['min_pick_success']),
        'collisions': summary['states_invalid'] <= thresholds['max_invalid_states'],
        'undetected_failures':
            summary['undetected_failures'] <= thresholds['max_undetected_failures'],
        # no detected failure at all is a pass
        'recovery': recovery['n'] == 0 or recovery['rate'] >= thresholds['min_recovery'],
        'empty_scenes': summary['attempts_in_empty_scenes'] == 0,
    }
