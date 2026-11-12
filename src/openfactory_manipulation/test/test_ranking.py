import json
import math

import pytest

from openfactory_manipulation.core import grasps as gr
from openfactory_manipulation.core import ranking as rk
from openfactory_manipulation.core import task

GRIPPER = {
    'palm': {'size': [0.06, 0.12, 0.04]},
    'finger': {'size': [0.02, 0.01, 0.11], 'stroke': 0.0425},
    'tcp_offset': 0.135,
}
PARAMS = {'floor_clearance': 0.005, 'palm_clearance': 0.003, 'preclose_margin': 0.005,
          'cylinder_directions': 8, 'finger_clearance': 0.003}
INNER = [0.30, 0.22, 0.10]
WEIGHTS = {'strategy': 'baseline', 'confidence': 1.0, 'clearance': 0.5, 'motion': 0.3,
           'clearance_cap': 0.03, 'motion_cap': 3.0, 'learned_motion': 0.1}


def unknown(x=0.0, y=0.0, yaw=0.0, size=(0.08, 0.05, 0.048), index=0):
    return gr.SceneObject(index, gr.UNKNOWN, 0.01, x, y, yaw, size)


def test_an_unknown_cluster_is_held_across_its_long_and_its_short_side():
    item = unknown(yaw=0.4, size=(0.05, 0.0433, 0.06))
    across_long, across_short = gr.generate(item, task.unknown_spec(item.size), GRIPPER, PARAMS)
    assert (across_long.label, across_short.label) == ('across_long', 'across_short')
    assert across_long.width == pytest.approx(0.05) and across_short.width == pytest.approx(0.0433)
    assert across_long.finger_yaw == pytest.approx(0.4)
    assert across_short.finger_yaw == pytest.approx(0.4 - math.pi / 2)
    assert across_long.z == pytest.approx(0.03)       # TCP at the middle of the measured height


def test_a_lying_cylinder_is_too_long_to_hold_end_to_end():
    item = unknown()
    across_long, across_short = gr.generate(item, task.unknown_spec(item.size), GRIPPER, PARAMS)
    assert gr.prefilter(across_long, GRIPPER, INNER, [], PARAMS)[0] == 'gripper_width'
    assert gr.prefilter(across_short, GRIPPER, INNER, [], PARAMS)[0] is None


def test_unknown_clusters_are_pickable_only_when_enabled():
    catalog = {'cube': {}, 'prism': {}}
    objects = [gr.SceneObject(0, 'cube', 0.9, 0, 0, 0, (0.05,) * 3),
               gr.SceneObject(1, 'cube', 0.3, 0, 0, 0, (0.05,) * 3), unknown(index=2)]
    assert [o.index for o in task.pickable(objects, catalog, 0.5, False)] == [0]
    assert [o.index for o in task.pickable(objects, catalog, 0.5, True)] == [0, 2]


def test_failures_near():
    failures = [(0.0, 0.0, 0.0), (0.01, 0.0, 1.0), (0.1, 0.0, 0.0)]
    assert task.failures_near(0.0, 0.0, failures, 0.02) == 2
    assert task.failures_near(0.2, 0.0, failures, 0.02) == 0


def _candidate(item, grasp, clearances, key=0.5):
    return {'object_score': item.score, 'key': key,
            'clearance': min(clearances.values()),
            'features': rk.features(item, grasp, clearances, 5, 1, INNER)}


def test_features():
    item = unknown(x=0.075, y=-0.055, size=(0.05, 0.0433, 0.06))
    _, across_short = gr.generate(item, task.unknown_spec(item.size), GRIPPER, PARAMS)
    found = rk.features(item, across_short, {'wall_clearance': math.inf,
                                             'neighbour_clearance': 0.01}, 4, 2, INNER)
    assert found['class_unknown'] == 1.0 and 'class_cube' not in found
    assert found['width_ratio'] == pytest.approx(0.0433 / 0.05)
    assert found['slenderness'] == pytest.approx(0.06 / 0.0433)
    assert found['wall_clearance'] == rk.CLEARANCE_CAP
    assert found['neighbour_clearance'] == pytest.approx(0.01)
    assert (found['edge_x'], found['edge_y']) == pytest.approx((0.5, 0.5))
    assert found['fingers_across_bin'] == pytest.approx(1.0)
    assert (found['n_objects'], found['prior_failures']) == (4.0, 2.0)
    assert all(isinstance(v, float) for v in found.values())
    json.dumps(found)                                    # goes into the trial log


def test_logistic_model_round_trip_and_missing_features():
    model = rk.LogisticModel(['a', 'class_cube'], [1.0, 0.5], [2.0, 0.5], [1.0, -2.0], 0.3)
    again = rk.LogisticModel.from_dict(json.loads(json.dumps(model.to_dict())))
    x = {'a': 3.0, 'class_cube': 1.0}
    assert again.logit(x) == pytest.approx(0.3 + 1.0 - 2.0)
    assert again.probability(x) == pytest.approx(1 / (1 + math.exp(0.7)))
    # an unseen class: its one-hot is absent, i.e. 0
    assert again.logit({'a': 1.0}) == pytest.approx(0.3 + 2.0)
    assert again.probability({'a': 1e6}) == pytest.approx(1.0)
    assert again.probability({'a': -1e6}) == pytest.approx(0.0)
    with pytest.raises(ValueError):
        rk.LogisticModel(['a'], [0.0, 1.0], [1.0], [1.0], 0.0)
    with pytest.raises(ValueError):
        rk.LogisticModel.from_dict({'type': 'forest'})


def test_strategies():
    item = unknown()
    _, grasp = gr.generate(item, task.unknown_spec(item.size), GRIPPER, PARAMS)
    candidate = _candidate(item, grasp, {'wall_clearance': 0.02, 'neighbour_clearance': 0.01},
                           key=0.42)
    baseline = rk.Ranker(WEIGHTS)
    assert baseline.prior(candidate) == pytest.approx(gr.score(0.01, 0.01, 0.0, WEIGHTS))
    assert baseline.score(candidate, 1.5) == pytest.approx(gr.score(0.01, 0.01, 1.5, WEIGHTS))
    assert baseline.success_probability(candidate) is None
    shuffled = rk.Ranker({**WEIGHTS, 'strategy': 'uniform_random'})
    assert shuffled.prior(candidate) == shuffled.score(candidate, 2.0) == 0.42
    model = rk.LogisticModel(['neighbour_clearance'], [0.0], [0.01], [1.0], 0.0)
    learned = rk.Ranker({**WEIGHTS, 'strategy': 'learned'}, model)
    p = 1 / (1 + math.exp(-1.0))
    assert learned.prior(candidate) == pytest.approx(p)
    assert learned.score(candidate, 1.5) == pytest.approx(p - 0.1 * 0.5)
    with pytest.raises(ValueError):
        rk.Ranker({**WEIGHTS, 'strategy': 'learned'})
    with pytest.raises(ValueError):
        rk.Ranker({**WEIGHTS, 'strategy': 'oracle'})
