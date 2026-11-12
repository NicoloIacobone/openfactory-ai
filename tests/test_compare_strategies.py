import pytest

import compare_strategies as cs


def _attempt(trial, ok, cls='cube', reason=None, strategy='baseline'):
    return {'trial_id': trial, 'success': ok, 'failure_reason': None if ok else reason,
            'strategy': strategy, 'selected_grasp': {'class_id': cls}, 'total_s': 18.0,
            'evaluation': {'states_invalid': 0}, 'detection': 'agree'}


def _scene(trial, n, deposited, end='bin_empty'):
    return {'trial_id': trial, 'n_objects': n, 'deposited': deposited, 'end_reason': end}


def test_run_metrics():
    attempts = [_attempt(0, True), _attempt(0, False, 'unknown', 'grasp_failed'),
                _attempt(0, True, 'unknown'), _attempt(1, True)]
    scenes = [_scene(0, 2, 2), _scene(1, 2, 1, 'no_feasible_grasp'), _scene(2, 0, 0)]
    m = cs.run_metrics(attempts, scenes)
    assert m['pick_success']['successes'] == 3 and m['pick_success']['n'] == 4
    assert m['pick_success_unknown']['n'] == 2 and m['pick_success_named']['rate'] == 1.0
    assert m['completion']['rate'] == pytest.approx(0.75)
    assert m['attempts_per_deposit'] == pytest.approx(4 / 3)
    assert m['failures'] == {'grasp_failed': 1}
    assert m['end_reasons'] == {'bin_empty': 2, 'no_feasible_grasp': 1}


def test_paired_difference_and_interval():
    ref_attempts = [_attempt(t, t % 2 == 0) for t in range(10)]
    oth_attempts = [_attempt(t, True) for t in range(10)]
    scenes_ref = [_scene(t, 1, int(t % 2 == 0)) for t in range(10)]
    scenes_oth = [_scene(t, 1, 1) for t in range(10)]
    result = cs.paired(cs.per_scene(ref_attempts, scenes_ref),
                       cs.per_scene(oth_attempts, scenes_oth), n_boot=2000)
    assert result['scenes'] == 10
    assert result['completion']['difference'] == pytest.approx(0.5)
    assert result['pick_success']['difference'] == pytest.approx(0.5)
    assert result['failed_per_scene']['difference'] == pytest.approx(-0.5)
    low, high = result['completion']['ci95']
    assert 0.0 < low <= 0.5 <= high <= 1.0
    # identical runs: no difference, zero-width interval
    same = cs.paired(cs.per_scene(oth_attempts, scenes_oth), cs.per_scene(oth_attempts, scenes_oth))
    assert same['completion'] == {'difference': 0.0, 'ci95': [0.0, 0.0]}


def test_runs_on_different_scenes_are_refused():
    a = cs.per_scene([], [_scene(0, 5, 5)])
    with pytest.raises(ValueError, match='same scenes'):
        cs.paired(a, cs.per_scene([], [_scene(0, 4, 4)]))
    with pytest.raises(ValueError, match='no scene'):
        cs.paired(a, cs.per_scene([], [_scene(1, 5, 5)]))
