import math

import pytest

import perception_eval as pe

CATALOG = {'objects': {
    'cube': {'shape': 'box', 'size': [0.05, 0.05, 0.05]},
    'cylinder': {'shape': 'cylinder', 'radius': 0.025, 'length': 0.08},
    'prism': {'shape': 'triangular_prism', 'side': 0.05, 'length': 0.06},
    'bar': {'shape': 'box', 'size': [0.08, 0.03, 0.03]},
}}
THRESHOLDS = {
    'min_recall': 0.95, 'min_precision': 0.95, 'min_class_accuracy': 0.95,
    'max_xy_median_m': 0.003, 'max_xy_p95_m': 0.008,
    'max_yaw_median_rad': math.radians(3), 'max_yaw_p95_rad': math.radians(8),
}


def truth(name, x, y, yaw=0.0):
    return {'name': f'{name}_0', 'type': name, 'x': x, 'y': y, 'yaw': yaw}


def detection(class_id, x, y, yaw=0.0, score=0.9):
    return {'class_id': class_id, 'score': score, 'x': x, 'y': y, 'yaw': yaw}


def test_pose_in_bin():
    half = 0.25
    pose = ((0.45, -0.28, 0.785), (0.0, 0.0, math.sin(half), math.cos(half)))
    in_bin = pe.pose_in_bin(pose, (0.40, -0.30, 0.76))
    assert (in_bin['x'], in_bin['y'], in_bin['z']) == pytest.approx((0.05, 0.02, 0.025))
    assert in_bin['yaw'] == pytest.approx(0.5) and in_bin['tilt'] == pytest.approx(0.0)
    lying = ((0.4, -0.3, 0.8), (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))
    assert pe.pose_in_bin(lying, (0.4, -0.3, 0.76))['tilt'] == pytest.approx(math.pi / 2)


def test_yaw_period_follows_the_shape():
    objects = CATALOG['objects']
    assert pe.yaw_period(objects['cube']) == pytest.approx(math.pi / 2)
    assert pe.yaw_period(objects['bar']) == pytest.approx(math.pi)
    assert pe.yaw_period(objects['prism']) == pytest.approx(math.pi)
    assert pe.yaw_period(objects['cylinder']) == 0.0


@pytest.mark.parametrize('measured, expected, period, error', [
    (0.1, 0.0, math.pi / 2, 0.1),
    (0.1 + math.pi / 2, 0.0, math.pi / 2, 0.1),
    (-3.0, 3.0, math.pi, 2 * math.pi - 6.0),
    (math.pi - 0.05, 0.0, math.pi, 0.05),
    (0.7, -0.7, math.pi / 2, math.pi / 2 - 1.4),
])
def test_yaw_error_respects_symmetry(measured, expected, period, error):
    assert pe.yaw_error(measured, expected, period) == pytest.approx(error)


def test_yaw_error_is_undefined_for_a_cylinder():
    assert pe.yaw_error(1.0, 0.0, 0.0) is None


def test_match_pairs_nearest_and_reports_missed_and_extra():
    scene = [truth('cube', 0.0, 0.0), truth('prism', 0.10, 0.0), truth('cylinder', -0.10, 0.05)]
    found = [detection('prism', 0.101, 0.0), detection('cube', 0.001, 0.001),
             detection('cube', 0.0, -0.08)]
    pairs, missed, extra = pe.match(scene, found, 0.02)
    assert sorted((ti, di) for ti, di, _ in pairs) == [(0, 1), (1, 0)]
    assert missed == [2] and extra == [2]


def test_score_scene_and_failure_reasons():
    scene = [truth('cube', 0.0, 0.0, 0.2), truth('cylinder', 0.1, 0.0)]
    good = pe.score_scene(scene, [detection('cube', 0.001, 0.0, 0.2 + math.pi / 2),
                                  detection('cylinder', 0.1, 0.002)], CATALOG, 0.02)
    assert pe.scene_failure(good) is None
    cube = next(m for m in good['matches'] if m['type'] == 'cube')
    assert cube['xy_error_m'] == pytest.approx(0.001) and cube['yaw_error_rad'] == pytest.approx(0)
    assert next(m for m in good['matches'] if m['type'] == 'cylinder')['yaw_error_rad'] is None

    missed = pe.score_scene(scene, [detection('cube', 0, 0)], CATALOG, 0.02)
    assert pe.scene_failure(missed) == 'missed_object'
    wrong = pe.score_scene(scene, [detection('prism', 0, 0), detection('cylinder', 0.1, 0)],
                           CATALOG, 0.02)
    assert pe.scene_failure(wrong) == 'wrong_class'
    extra = pe.score_scene([], [detection('cube', 0, 0)], CATALOG, 0.02)
    assert pe.scene_failure(extra) == 'false_positive'
    assert pe.scene_failure(pe.score_scene([], [], CATALOG, 0.02)) is None


def test_percentile():
    assert pe.percentile([1.0], 95) == 1.0
    assert pe.percentile([0.0, 10.0], 50) == pytest.approx(5.0)
    assert pe.percentile(list(range(101)), 95) == pytest.approx(95.0)


def test_summary_passes_on_good_results_and_fails_on_bad_ones():
    def result(xy, yaw):
        return pe.score_scene([truth('cube', 0.0, 0.0)], [detection('cube', xy, 0.0, yaw)],
                              CATALOG, 0.02)

    good = [result(0.001, 0.01) for _ in range(20)] + [pe.score_scene([], [], CATALOG, 0.02)]
    summary = pe.summarize(good, THRESHOLDS)
    assert summary['passed'] and summary['recall'] == 1.0 and summary['precision'] == 1.0
    assert summary['xy_error_m']['median'] == pytest.approx(0.001)
    assert summary['empty_scenes'] == 1 and summary['empty_scene_detections'] == 0

    far = pe.summarize([result(0.006, 0.01) for _ in range(20)], THRESHOLDS)
    assert not far['passed'] and not far['checks']['xy_median'] and far['checks']['xy_p95']

    ghost = good + [pe.score_scene([], [detection('cube', 0, 0)], CATALOG, 0.02)]
    summary = pe.summarize(ghost, THRESHOLDS)
    assert not summary['passed'] and not summary['checks']['empty_scenes']

    nothing = pe.summarize([pe.score_scene([truth('cube', 0, 0)], [], CATALOG, 0.02)], THRESHOLDS)
    assert not nothing['passed'] and nothing['recall'] == 0.0 and nothing['xy_error_m'] is None
