"""ROS-free evaluation of perception output against simulator ground truth (harness only).

Ground truth never reaches the perception node (rule 5): these functions run in the trial
harness after the detections have been received. Poses are (x, y, yaw) in the input_bin frame.
"""

import math
import statistics

from openfactory_manipulation.core.geometry import quat_rotate, yaw_of


def pose_in_bin(pose, bin_origin):
    """Ground-truth world pose (xyz, quat) -> {'x', 'y', 'z', 'yaw', 'tilt'} in a bin frame
    whose origin is `bin_origin` (bins are not rotated in the cell)."""
    xyz, quat = pose
    up = quat_rotate(quat, (0.0, 0.0, 1.0))[2]
    return {'x': xyz[0] - bin_origin[0], 'y': xyz[1] - bin_origin[1], 'z': xyz[2] - bin_origin[2],
            'yaw': yaw_of(quat), 'tilt': math.acos(max(-1.0, min(1.0, up)))}


def yaw_period(spec):
    """Rotational symmetry about z of a catalog object resting in its catalog orientation;
    0 when every yaw looks the same (cylinder)."""
    shape = spec['shape']
    if shape == 'cylinder':
        return 0.0
    if shape == 'box':
        x, y, _ = spec['size']
        return math.pi / 2 if math.isclose(x, y) else math.pi
    if shape == 'triangular_prism':
        return math.pi
    raise ValueError(f'unknown shape {shape!r}')


def yaw_error(measured, expected, period):
    """Absolute yaw difference modulo the object's symmetry; None when yaw is undefined."""
    if period <= 0:
        return None
    error = math.fmod(measured - expected, period)
    if error > period / 2:
        error -= period
    elif error < -period / 2:
        error += period
    return abs(error)


def match(truth, detections, max_distance):
    """Greedy nearest-neighbour pairing on the xy distance.

    Returns (pairs [(truth index, detection index, distance)], missed truth indices, extra
    detection indices). A detection farther than `max_distance` from every free object is extra.
    """
    candidates = sorted(
        (math.hypot(t['x'] - d['x'], t['y'] - d['y']), ti, di)
        for ti, t in enumerate(truth) for di, d in enumerate(detections))
    used_truth, used_detections, pairs = set(), set(), []
    for distance, ti, di in candidates:
        if distance > max_distance:
            break
        if ti in used_truth or di in used_detections:
            continue
        used_truth.add(ti)
        used_detections.add(di)
        pairs.append((ti, di, distance))
    missed = [ti for ti in range(len(truth)) if ti not in used_truth]
    extra = [di for di in range(len(detections)) if di not in used_detections]
    return pairs, missed, extra


def score_scene(truth, detections, catalog, max_distance):
    """Per-scene result: matched objects with their errors, missed objects, false positives.

    `truth`: [{'name', 'type', 'x', 'y', 'yaw'}]; `detections`: [{'class_id', 'score', 'x',
    'y', 'yaw'}].
    """
    pairs, missed, extra = match(truth, detections, max_distance)
    matches = []
    for ti, di, distance in pairs:
        t, d = truth[ti], detections[di]
        matches.append({
            'name': t['name'], 'type': t['type'], 'class_id': d['class_id'],
            'class_ok': d['class_id'] == t['type'], 'score': d['score'], 'xy_error_m': distance,
            'yaw_error_rad': yaw_error(d['yaw'], t['yaw'],
                                       yaw_period(catalog['objects'][t['type']])),
        })
    return {
        'n_objects': len(truth),
        'n_detections': len(detections),
        'matches': matches,
        'missed': [truth[ti]['name'] for ti in missed],
        'false_positives': [detections[di] for di in extra],
    }


def scene_failure(result):
    """None when every object was found with the right class and nothing else was reported."""
    if result['missed']:
        return 'missed_object'
    if result['false_positives']:
        return 'false_positive'
    if not all(m['class_ok'] for m in result['matches']):
        return 'wrong_class'
    return None


def percentile(values, q):
    """Linear-interpolation percentile (q in [0, 100]) of a non-empty list."""
    ordered = sorted(values)
    position = (len(ordered) - 1) * q / 100.0
    low = int(math.floor(position))
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _spread(values):
    if not values:
        return None
    return {'median': statistics.median(values), 'p95': percentile(values, 95),
            'max': max(values), 'n': len(values)}


def summarize(results, thresholds):
    """Run-level metrics and the pass verdict against `thresholds`.

    Pose errors are taken over matched objects of the right class; an empty-bin scene passes
    only with zero detections.
    """
    objects = sum(r['n_objects'] for r in results)
    detections = sum(r['n_detections'] for r in results)
    matches = [m for r in results for m in r['matches']]
    correct = [m for m in matches if m['class_ok']]
    empty = [r for r in results if r['n_objects'] == 0]
    xy = _spread([m['xy_error_m'] for m in correct])
    yaw = _spread([m['yaw_error_rad'] for m in correct if m['yaw_error_rad'] is not None])
    summary = {
        'scenes': len(results), 'objects': objects, 'detections': detections,
        'matched': len(matches),
        'recall': len(matches) / objects if objects else None,
        'precision': len(matches) / detections if detections else None,
        'class_accuracy': len(correct) / len(matches) if matches else None,
        'xy_error_m': xy, 'yaw_error_rad': yaw,
        'score_correct': _spread([m['score'] for m in correct]),
        'score_wrong': _spread([m['score'] for m in matches if not m['class_ok']]),
        'empty_scenes': len(empty),
        'empty_scene_detections': sum(r['n_detections'] for r in empty),
    }
    checks = {
        'recall': summary['recall'] is not None and summary['recall'] >= thresholds['min_recall'],
        'precision': (summary['precision'] is not None
                      and summary['precision'] >= thresholds['min_precision']),
        'class_accuracy': (summary['class_accuracy'] is not None
                           and summary['class_accuracy'] >= thresholds['min_class_accuracy']),
        'xy_median': xy is not None and xy['median'] <= thresholds['max_xy_median_m'],
        'xy_p95': xy is not None and xy['p95'] <= thresholds['max_xy_p95_m'],
        'yaw_median': yaw is not None and yaw['median'] <= thresholds['max_yaw_median_rad'],
        'yaw_p95': yaw is not None and yaw['p95'] <= thresholds['max_yaw_p95_rad'],
        'empty_scenes': summary['empty_scene_detections'] == 0,
    }
    summary['checks'] = checks
    summary['passed'] = all(checks.values())
    return summary
