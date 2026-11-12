import math
import random

import numpy as np
import pytest

import ranker_fit as rf


def _record(features, success, trial, run='r'):
    return {'run_id': run, 'trial_id': trial, 'success': success,
            'selected_grasp': {'features': features}}


def _synthetic(n=400, seed=0):
    """Success falls with `tight` (clearance-like) and is unrelated to `noise`; the unknown
    class fails more often."""
    rng = random.Random(seed)
    records = []
    for i in range(n):
        tight, noise = rng.random(), rng.random()
        unknown = rng.random() < 0.5
        z = 3.0 - 4.0 * tight - (1.5 if unknown else 0.0)
        features = {'tight': tight, 'noise': noise, 'class_unknown' if unknown else 'class_cube': 1.0}
        records.append(_record(features, rng.random() < 1 / (1 + math.exp(-z)), i // 5))
    return records


def test_dataset_fills_missing_features_with_zero():
    names, X, y, groups = rf.dataset([_record({'a': 1.0, 'class_cube': 1.0}, True, 0),
                                      _record({'a': 2.0, 'class_unknown': 1.0}, False, 1),
                                      {'run_id': 'r', 'trial_id': 2, 'success': True,
                                       'selected_grasp': {}}])
    assert names == ['a', 'class_cube', 'class_unknown']
    assert X.tolist() == [[1.0, 1.0, 0.0], [2.0, 0.0, 1.0]]
    assert y.tolist() == [1.0, 0.0] and groups == ['r:0', 'r:1']
    with pytest.raises(ValueError):
        rf.dataset([])


def test_fit_recovers_the_direction_of_the_effects():
    names, X, y, _ = rf.dataset(_synthetic())
    mean, scale = rf.standardise(X)
    weights, _ = rf.fit((X - mean) / scale, y, l2=1.0)
    w = dict(zip(names, weights))
    assert w['tight'] < -0.5
    assert abs(w['noise']) < abs(w['tight']) / 3
    assert w['class_unknown'] < w['class_cube']


def test_metrics():
    assert rf.auc([1, 1, 0, 0], [0.9, 0.8, 0.2, 0.1]) == 1.0
    assert rf.auc([1, 0], [0.5, 0.5]) == 0.5
    assert rf.auc([1, 1], [0.5, 0.6]) is None
    assert rf.brier([1, 0], [1.0, 0.0]) == 0.0
    assert rf.log_loss([1, 0], [0.5, 0.5]) == pytest.approx(math.log(2))


def test_folds_keep_a_scene_together():
    groups = [f'r:{i // 3}' for i in range(30)]
    folds = rf.grouped_folds(groups, 4)
    for group in set(groups):
        assert len({folds[i] for i, g in enumerate(groups) if g == group}) == 1
    assert set(folds.tolist()) == {0, 1, 2, 3}
    assert (rf.grouped_folds(groups, 4) == folds).all()


def test_train_beats_a_constant_and_writes_a_usable_model():
    records = _synthetic()
    model, report = rf.train(records, meta={'runs': ['r']})
    assert report['attempts'] == 400 and report['scenes'] == 80
    chosen = next(v for v in report['validation'] if v['l2'] == report['chosen_l2'])
    assert chosen['log_loss'] < report['constant_log_loss']
    assert chosen['auc'] > 0.7
    good = {'tight': 0.1, 'noise': 0.5, 'class_cube': 1.0}
    bad = {'tight': 0.9, 'noise': 0.5, 'class_unknown': 1.0}
    assert model.probability(good) > model.probability(bad)
    assert model.meta['runs'] == ['r'] and model.meta['l2'] == report['chosen_l2']


def test_a_single_outcome_cannot_be_learned():
    with pytest.raises(ValueError, match='single outcome'):
        rf.train([_record({'a': float(i)}, True, i) for i in range(10)])


def test_fit_is_stable_when_classes_separate():
    Z = np.array([[-1.0], [-0.5], [0.5], [1.0]])
    weights, bias = rf.fit(Z, np.array([0.0, 0.0, 1.0, 1.0]), l2=1.0)
    assert np.isfinite(weights).all() and math.isfinite(bias) and weights[0] > 0
