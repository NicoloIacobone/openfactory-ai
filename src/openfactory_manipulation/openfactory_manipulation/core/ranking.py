"""ROS-free ranking of the grasp candidates that passed the hard filters (phase 5, D-019).

Feasibility is never decided here: the geometric filters (core.grasps) and MoveIt
(grasp_selector) reject candidates first, and a strategy only orders the survivors
(docs/agent-guides/manipulation.md, rule 4). Strategies:

- `baseline`: the hand-designed score of D-016 (perception confidence, clearance, motion).
- `uniform_random`: every survivor equally likely; collects training outcomes without the
  preferences of any score.
- `learned`: a logistic model of the probability that the attempt succeeds, from features the
  cell has before it moves, minus a small motion term (the J(g) of the manipulation guide).

Features come from perception hypotheses, the candidate's geometry and the cell's own memory
of the scene, never from the simulator. The same function computes them for the trial log
(training data) and for inference, so the two cannot drift apart.
"""

import json
import math
from pathlib import Path

from . import grasps

STRATEGIES = ('baseline', 'uniform_random', 'learned')
UNKNOWN = grasps.UNKNOWN
CLEARANCE_CAP = 0.05          # m: clearances above this are the same for the features
_EPS = 1e-9


def features(obj, grasp, clearances, n_objects, prior_failures, inner_size):
    """Feature dict of one candidate.

    `obj`: core.grasps.SceneObject of the target, `grasp`: core.grasps.Grasp, `clearances`:
    {'wall_clearance', 'neighbour_clearance'} from core.grasps.prefilter, `n_objects`: objects
    in the view, `prior_failures`: failed grasps of this scene on this object, `inner_size`:
    input bin (x, y, wall height).
    """
    long_side = max(obj.size[0], obj.size[1])
    width_ratio = grasp.width / max(long_side, _EPS)
    return {
        f'class_{obj.class_id}': 1.0,
        'object_score': obj.score,
        'width': grasp.width,
        'width_ratio': width_ratio,
        # catalog grasps hold a known face pair; for a cluster of unknown shape the side it is
        # held across is the open question, so its width ratio gets its own weight
        'unknown_width_ratio': width_ratio if obj.class_id == UNKNOWN else 0.0,
        'height': obj.size[2],
        'slenderness': obj.size[2] / max(grasp.width, _EPS),
        'wall_clearance': min(clearances['wall_clearance'], CLEARANCE_CAP),
        'neighbour_clearance': min(clearances['neighbour_clearance'], CLEARANCE_CAP),
        'edge_x': abs(obj.x) / (inner_size[0] / 2),
        'edge_y': abs(obj.y) / (inner_size[1] / 2),
        'fingers_across_bin': abs(math.sin(grasp.finger_yaw)),
        'n_objects': float(n_objects),
        'prior_failures': float(prior_failures),
    }


class LogisticModel:
    """P(success) = sigmoid(bias + sum w_i (x_i - mean_i) / scale_i); a feature missing from a
    candidate counts as 0 (the one-hot class of a class the model never saw)."""

    def __init__(self, names, mean, scale, weights, bias, meta=None):
        if not len(names) == len(mean) == len(scale) == len(weights):
            raise ValueError('names, mean, scale and weights differ in length')
        self.names, self.mean, self.scale = list(names), list(mean), list(scale)
        self.weights, self.bias, self.meta = list(weights), float(bias), meta or {}

    @classmethod
    def from_dict(cls, data):
        if data.get('type') != 'logistic':
            raise ValueError(f"unsupported model type {data.get('type')!r}")
        return cls(data['features'], data['mean'], data['scale'], data['weights'],
                   data['bias'], data.get('meta'))

    def to_dict(self):
        return {'type': 'logistic', 'features': self.names, 'mean': self.mean,
                'scale': self.scale, 'weights': self.weights, 'bias': self.bias,
                'meta': self.meta}

    def logit(self, feature_dict):
        return self.bias + sum(
            w * (feature_dict.get(name, 0.0) - m) / s
            for name, m, s, w in zip(self.names, self.mean, self.scale, self.weights))

    def probability(self, feature_dict):
        z = self.logit(feature_dict)
        if z >= 0:
            return 1.0 / (1.0 + math.exp(-z))
        e = math.exp(z)
        return e / (1.0 + e)


def load_model(path):
    return LogisticModel.from_dict(json.loads(Path(path).read_text()))


class Ranker:
    """Orders feasible candidates. A candidate is the dict built by grasp_selector, with
    'object_score', 'clearance', 'features', 'key' (a uniform random number drawn for it) and,
    once MoveIt validated it, 'motion'."""

    def __init__(self, weights, model=None):
        self.strategy = weights['strategy']
        if self.strategy not in STRATEGIES:
            raise ValueError(f'unknown ranking strategy {self.strategy!r}')
        if self.strategy == 'learned' and model is None:
            raise ValueError('the learned strategy needs a model')
        self.weights, self.model = weights, model

    def success_probability(self, candidate):
        return None if self.model is None else self.model.probability(candidate['features'])

    def _effort(self, motion):
        cap = self.weights['motion_cap']
        return min(max(motion, 0.0), cap) / cap

    def prior(self, candidate):
        """Score before MoveIt (motion unknown): the order in which survivors are checked."""
        return self.score(candidate, 0.0)

    def score(self, candidate, motion):
        """Final score of a candidate whose pre-grasp is `motion` rad away from ready."""
        if self.strategy == 'baseline':
            return grasps.score(candidate['object_score'], candidate['clearance'], motion,
                                self.weights)
        if self.strategy == 'uniform_random':
            return candidate['key']
        return (self.success_probability(candidate)
                - self.weights['learned_motion'] * self._effort(motion))
