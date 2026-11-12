"""ROS-free fit and validation of the grasp-ranking model (phase 5, D-019). numpy only.

Training data are trial logs (attempts.jsonl): for each attempt, the features of the grasp the
cell chose (computed by openfactory_manipulation.core.ranking.features, from perception and
geometry only) and the outcome of the attempt as the harness judged it. Ground truth enters
only as that label (docs/agent-guides/benchmarking.md), never as a feature.

The model is an L2-regularised logistic regression on standardised features, fitted by Newton's
method. Its regularisation is chosen by cross-validation in which all the attempts of one scene
are in the same fold (attempts of a scene are not independent).
"""

import math
import random

import numpy as np
from openfactory_manipulation.core.ranking import LogisticModel


def dataset(records):
    """(names, X, y, groups) from attempt records.

    `names`: sorted feature names seen in the data; `X`: n x d (a feature missing from a
    record is 0, as at inference); `y`: 1 for a successful attempt; `groups`: scene of each
    attempt ('<run_id>:<trial_id>').
    """
    rows = [(r['selected_grasp']['features'], bool(r['success']), f"{r['run_id']}:{r['trial_id']}")
            for r in records if r['selected_grasp'].get('features')]
    if not rows:
        raise ValueError('no attempt with grasp features')
    names = sorted({name for features, _, _ in rows for name in features})
    X = np.array([[features.get(name, 0.0) for name in names] for features, _, _ in rows],
                 dtype=float)
    y = np.array([float(ok) for _, ok, _ in rows])
    return names, X, y, [group for _, _, group in rows]


def standardise(X):
    """(mean, scale) per column; a constant column gets scale 1."""
    mean = X.mean(axis=0)
    scale = X.std(axis=0)
    scale[scale < 1e-12] = 1.0
    return mean, scale


def _sigmoid(z):
    return 1.0 / (1.0 + np.exp(-np.clip(z, -35.0, 35.0)))


def fit(Z, y, l2, iterations=100, tolerance=1e-10):
    """(weights, bias) minimising the log-loss + l2 / 2 * |weights|^2 on standardised `Z`.
    The bias is not penalised."""
    n, d = Z.shape
    A = np.hstack([np.ones((n, 1)), Z])
    theta = np.zeros(d + 1)
    penalty = np.full(d + 1, float(l2))
    penalty[0] = 0.0
    for _ in range(iterations):
        p = _sigmoid(A @ theta)
        gradient = A.T @ (p - y) + penalty * theta
        hessian = (A * (p * (1 - p))[:, None]).T @ A + np.diag(penalty) + 1e-9 * np.eye(d + 1)
        step = np.linalg.solve(hessian, gradient)
        theta -= step
        if np.max(np.abs(step)) < tolerance:
            break
    return theta[1:], theta[0]


def predict(weights, bias, Z):
    return _sigmoid(Z @ weights + bias)


def auc(y, p):
    """Area under the ROC curve (Mann-Whitney, ties counted half); None with one class."""
    y, p = np.asarray(y), np.asarray(p)
    positives, negatives = p[y == 1], p[y == 0]
    if len(positives) == 0 or len(negatives) == 0:
        return None
    greater = (positives[:, None] > negatives[None, :]).sum()
    ties = (positives[:, None] == negatives[None, :]).sum()
    return float((greater + 0.5 * ties) / (len(positives) * len(negatives)))


def brier(y, p):
    return float(np.mean((np.asarray(p) - np.asarray(y)) ** 2))


def log_loss(y, p):
    p = np.clip(np.asarray(p), 1e-12, 1 - 1e-12)
    y = np.asarray(y)
    return float(-np.mean(y * np.log(p) + (1 - y) * np.log(1 - p)))


def grouped_folds(groups, k, seed=0):
    """Fold index of every sample: whole groups (scenes) go to one fold, shuffled by `seed`."""
    unique = sorted(set(groups))
    random.Random(seed).shuffle(unique)
    fold_of = {group: index % k for index, group in enumerate(unique)}
    return np.array([fold_of[group] for group in groups])


def out_of_fold(X, y, groups, l2, k, seed=0):
    """Predictions of every sample by a model fitted without its fold (standardised on the
    training folds only)."""
    folds = grouped_folds(groups, k, seed)
    p = np.empty(len(y))
    for fold in range(k):
        test = folds == fold
        if not test.any():
            continue
        train = ~test
        mean, scale = standardise(X[train])
        if len(set(y[train])) < 2:                     # one class only: its rate
            p[test] = y[train].mean()
            continue
        weights, bias = fit((X[train] - mean) / scale, y[train], l2)
        p[test] = predict(weights, bias, (X[test] - mean) / scale)
    return p


def train(records, l2_grid=(0.3, 1.0, 3.0, 10.0, 30.0, 100.0), k=5, seed=0, meta=None):
    """(LogisticModel, report). The L2 weight with the lowest out-of-fold log-loss is used to
    fit the final model on every attempt."""
    names, X, y, groups = dataset(records)
    if len(set(y)) < 2:
        raise ValueError('the data hold a single outcome: nothing to learn')
    k = min(k, len(set(groups)))
    rate = float(y.mean())
    validation = []
    for l2 in l2_grid:
        p = out_of_fold(X, y, groups, l2, k, seed)
        validation.append({'l2': l2, 'log_loss': log_loss(y, p), 'brier': brier(y, p),
                           'auc': auc(y, p)})
    best = min(validation, key=lambda row: row['log_loss'])
    mean, scale = standardise(X)
    weights, bias = fit((X - mean) / scale, y, best['l2'])
    report = {
        'attempts': len(y), 'successes': int(y.sum()), 'failures': int(len(y) - y.sum()),
        'scenes': len(set(groups)), 'folds': k, 'success_rate': rate,
        # what a constant prediction scores: the model must beat it to be worth anything
        'constant_log_loss': log_loss(y, np.full(len(y), rate)),
        'constant_brier': brier(y, np.full(len(y), rate)),
        'validation': validation, 'chosen_l2': best['l2'],
        'weights': {name: float(w) for name, w in zip(names, weights)},
    }
    model = LogisticModel(names, [float(v) for v in mean], [float(v) for v in scale],
                          [float(v) for v in weights], float(bias),
                          {**(meta or {}), 'l2': best['l2'], 'attempts': len(y),
                           'failures': report['failures'], 'cv_auc': best['auc'],
                           'cv_log_loss': best['log_loss'],
                           'constant_log_loss': report['constant_log_loss']})
    return model, report


def is_finite(value):
    return value is None or math.isfinite(value)
