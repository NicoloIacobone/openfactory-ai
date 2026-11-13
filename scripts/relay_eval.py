"""ROS-free evaluation of relay-loop runs from simulator ground truth (harness only).

The ring reports what it believes happened (openfactory_manipulation.relay); here the trial
harness compares it with the object poses read from the simulator, which neither perception nor
manipulation ever sees (rule 5). A handover counts when the object the arm aimed at rests
inside the destination bin, on any face (the deposit criterion of D-017).
"""

import math
from collections import Counter

import scene_objects as so
import trial_log as tl
from openfactory_manipulation.core.geometry import compose, relative

ELSEWHERE, MISSING = 'elsewhere', 'missing'
OUTSIDE = (ELSEWHERE, MISSING)


def in_bin(bin_pose, xyz):
    """A world point in the frame of a bin (bins of the ring are rotated about z)."""
    return relative(bin_pose, (tuple(xyz), (0.0, 0.0, 0.0, 1.0)))[0]


def location(xyz, bin_poses, inner_size):
    """Name of the bin whose inner volume holds a point (floor area, floor to rim),
    'elsewhere' outside every bin, 'missing' without a pose."""
    if xyz is None:
        return MISSING
    for name, pose in bin_poses.items():
        x, y, z = in_bin(pose, xyz)
        if (abs(x) <= inner_size[0] / 2 and abs(y) <= inner_size[1] / 2
                and 0.0 <= z <= inner_size[2]):
            return name
    return ELSEWHERE


def spawn_pose(bin_pose, spec, x, y, yaw, rest=so.CATALOG_REST, drop=0.002):
    """World (xyz, rpy) of an object resting at (x, y, yaw) of a bin frame, `drop` above its
    floor. The bin is rotated about z only, so its yaw adds to the object's."""
    bin_yaw = 2 * math.atan2(bin_pose[1][2], bin_pose[1][3])
    height = so.resting_size(spec, rest)[2] / 2 + drop
    xyz, _ = compose(bin_pose, ((x, y, height), (0.0, 0.0, 0.0, 1.0)))
    roll, pitch, _ = so.rest_rpy(spec, rest)
    return list(xyz), (roll, pitch, bin_yaw + yaw)


def scene_names(bin_index, objects):
    """The objects of one sampled bin with names unique in the ring and their bin."""
    return [{**item, 'name': f"b{bin_index}_{item['name']}", 'bin': bin_index}
            for item in objects]


def object_states(objects, first, second, bin_poses, inner_size, tolerances):
    """Ground-truth state of every spawned object from two pose readings a moment apart:
    {name: {'type', 'location', 'at_rest', 'xyz', 'quat'}}."""
    states = {}
    for item in objects:
        name = item['name']
        pose, earlier = second.get(name), first.get(name)
        if pose is None:
            states[name] = {'type': item['type'], 'location': MISSING, 'at_rest': False,
                            'xyz': None, 'quat': None}
            continue
        xyz, quat = pose
        states[name] = {
            'type': item['type'], 'location': location(xyz, bin_poses, inner_size),
            'at_rest': earlier is not None
            and math.dist(xyz, earlier[0]) <= tolerances['rest_motion_m'],
            'xyz': list(xyz), 'quat': list(quat)}
    return states


def match_target(x, y, states, bin_name, bin_pose, max_distance):
    """Name of the object in bin `bin_name` nearest to (x, y) of that bin's frame, where the
    arm aimed; None if nothing is within `max_distance`."""
    best, best_distance = None, max_distance
    for name, state in states.items():
        if state['location'] != bin_name:
            continue
        ox, oy, _ = in_bin(bin_pose, state['xyz'])
        distance = math.hypot(ox - x, oy - y)
        if distance <= best_distance:
            best, best_distance = name, distance
    return best


def moved(before, after, tolerance):
    return (before['location'] != after['location']
            or (after['xyz'] is not None and before['xyz'] is not None
                and math.dist(before['xyz'], after['xyz']) > tolerance))


def evaluate_turn(target, before, after, destination, tolerances):
    """What one turn did to the ring.

    `target`: name of the object the arm aimed at (match_target on `before`) or None;
    `before`, `after`: object_states around the turn; `destination`: name of the bin the
    object was to reach.
    """
    arrived = (target is not None and after[target]['location'] == destination
               and after[target]['at_rest'])
    return {
        'target': target,
        'target_type': None if target is None else after[target]['type'],
        'target_location': None if target is None else after[target]['location'],
        'target_arrived': arrived,
        # other objects that changed bin or moved, wherever they are
        'disturbed': sorted(name for name, state in after.items()
                            if name != target
                            and moved(before[name], state, tolerances['disturbed_m'])),
        'lost': sorted(name for name, state in after.items()
                       if state['location'] in OUTSIDE
                       and before[name]['location'] not in OUTSIDE),
    }


def handover_result(cell_success, cell_reason, evaluation, states_invalid):
    """(success, failure_reason, detection) of one handover.

    Success needs every source: the cell's own verdict, the object resting in the destination
    bin according to ground truth, and no executed state in collision. `detection` compares
    the cell's verdict with ground truth: 'agree', 'undetected_failure' (the cell believes a
    handover whose object did not arrive) or 'false_alarm' (the cell reports a failure but the
    object arrived).
    """
    arrived = evaluation['target_arrived']
    detection = ('agree' if cell_success == arrived
                 else 'undetected_failure' if cell_success else 'false_alarm')
    if not cell_success:
        return False, cell_reason, detection
    if not arrived:
        return False, 'place_failed', detection
    if states_invalid > 0:
        return False, 'collision', detection
    return True, None, detection


def _rate(successes, n):
    low, high = tl.wilson_interval(successes, n)
    return {'successes': successes, 'n': n, 'rate': successes / n if n else None,
            'ci95': [low, high]}


def summarize(handovers, turns, final_states, n, end_reason, wanted, thresholds):
    """Metrics of a run and whether it passes.

    `handovers`: the attempt records (one per handover tried); `turns`: one light record per
    turn of the loop, skipped ones included; `final_states`: object_states at the end;
    `wanted`: the number of handovers the run was asked for.
    """
    summary = tl.summarize(handovers)
    by_arm = []
    for arm in range(n):
        own = [r for r in handovers if r['arm'] == arm]
        by_arm.append(_rate(sum(r['success'] for r in own), len(own)))
    where = Counter(state['location'] for state in final_states.values())
    wall_s = sum(t['total_s'] for t in turns)
    moved_objects = Counter(r['evaluation']['target'] for r in handovers
                            if r['success'] and r['evaluation']['target'] is not None)
    done = sum(r['success'] for r in handovers)
    summary.update({
        'arms': n,
        'end_reason': end_reason,
        'handovers_wanted': wanted,
        'turns': len(turns),
        'skipped_turns': dict(Counter(t['skipped'] for t in turns if t['skipped'] is not None)),
        'handover_success': _rate(done, len(handovers)),
        'handover_success_by_arm': by_arm,
        'laps': min((rate['successes'] for rate in by_arm), default=0),
        'handovers_per_hour': 3600 * done / wall_s if wall_s else None,
        'undetected_failures': sum(r['detection'] == 'undetected_failure' for r in handovers),
        'false_alarms': sum(r['detection'] == 'false_alarm' for r in handovers),
        'not_parked': sum(not t['parked'] for t in turns),
        'collision': _rate(sum(t['states_invalid'] > 0 for t in turns), len(turns)),
        'states_checked': sum(t['states_checked'] for t in turns),
        'states_invalid': sum(t['states_invalid'] for t in turns),
        'objects': len(final_states),
        # ground truth: how many different objects went round, and how often the busiest did
        'objects_handed_over': len(moved_objects),
        'handovers_of_busiest_object': max(moved_objects.values(), default=0),
        'objects_by_location': dict(sorted(where.items())),
        'objects_lost': sum(count for place, count in where.items() if place in OUTSIDE),
        'objects_disturbed': sum(len(t['disturbed']) for t in turns),
    })
    summary['checks'] = checks(summary, thresholds)
    summary['passed'] = all(summary['checks'].values())
    return summary


def checks(summary, thresholds):
    """Each closing-test threshold and whether the run meets it."""
    success = summary['handover_success']
    return {
        'ran_to_the_end': (summary['end_reason'] == 'handovers_done'
                           and success['n'] >= summary['handovers_wanted']),
        'handover_success': (success['rate'] is not None
                             and success['rate'] >= thresholds['min_handover_success']),
        'collisions': summary['states_invalid'] <= thresholds['max_invalid_states'],
        'objects_lost': summary['objects_lost'] <= thresholds['max_objects_lost'],
        'undetected_failures':
            summary['undetected_failures'] <= thresholds['max_undetected_failures'],
        'every_arm_works': all(rate['successes'] >= thresholds['min_handovers_per_arm']
                               for rate in summary['handover_success_by_arm']),
    }
