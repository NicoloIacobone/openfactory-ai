"""Model-based recognition of catalog primitives from one segmented cluster. ROS-free.

A cluster is the set of points (bin frame, z up from the bin floor) of one object resting on
the floor in its catalog orientation (objects.yaml): box and cylinder on a flat face, the
triangular prism on a rectangular face with its ridge up. Each catalog shape predicts the
features measured on a cluster (top height, footprint rectangle, how much of the rectangle is
filled, how much of the footprint is at top height); the class is the shape with the smallest
normalised residual and the confidence falls with that residual.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

UNKNOWN = 'unknown'
_TAN_60 = math.sqrt(3.0)
_TOP_PERCENTILE = 95.0
_FLAT_TOP = 0.9               # share of footprint cells at top height for a flat-topped shape:
                              # the rim cells average the top with the side faces


@dataclass(frozen=True)
class Features:
    """What is measured on a cluster. Lengths in metres, angles in radians."""

    centre: tuple             # footprint rectangle centre (x, y)
    long: float               # footprint rectangle sides
    short: float
    long_axis: float          # direction of the long side, in (-pi/2, pi/2]
    height: float             # median height of the top layer
    fill: float               # footprint area / rectangle area
    flat: float               # share of footprint cells in the top layer
    n_points: int


@dataclass(frozen=True)
class ShapeModel:
    """Features a catalog shape is expected to show, and how its yaw follows the footprint."""

    name: str
    height: float
    long: float
    short: float
    fill: float
    flat: float
    yaw_offset: float         # object yaw = long axis + offset
    yaw_period: float         # rotational symmetry about z; 0 = yaw undefined (cylinder)
    box: tuple                # catalog bounding box (x, y, z) in the object frame


@dataclass(frozen=True)
class Detection:
    """One object hypothesis in the bin frame; the pose is that of the catalog object frame."""

    class_id: str
    score: float
    xyz: tuple
    yaw: float
    yaw_period: float
    size: tuple               # bounding box (x, y, z) in the object frame
    features: Features
    costs: dict = field(default_factory=dict)


def _top_layer_height(low, high, flat_tolerance):
    """Expected `height` feature of a surface whose heights are uniform in [low, high]."""
    top = low + _TOP_PERCENTILE / 100.0 * (high - low)
    return (max(top - flat_tolerance, low) + high) / 2.0


def shape_models(catalog, params):
    """Expected features of every catalog object, given the segmentation parameters."""
    threshold, tolerance = params['floor_threshold'], params['flat_tolerance']
    models = []
    for name, spec in catalog['objects'].items():
        shape = spec['shape']
        if shape == 'box':
            x, y, z = spec['size']
            models.append(ShapeModel(
                name, z, max(x, y), min(x, y), 1.0, _FLAT_TOP,
                yaw_offset=0.0 if x >= y else math.pi / 2,
                yaw_period=math.pi / 2 if math.isclose(x, y) else math.pi, box=(x, y, z)))
        elif shape == 'cylinder':
            d, length = 2 * spec['radius'], spec['length']
            models.append(ShapeModel(name, length, d, d, math.pi / 4, _FLAT_TOP, 0.0, 0.0,
                                     box=(d, d, length)))
        elif shape == 'triangular_prism':
            side, length = spec['side'], spec['length']
            ridge = side * _TAN_60 / 2
            # the floor threshold cuts the foot of both slanted faces
            base = side - 2 * threshold / _TAN_60
            top = threshold + _TOP_PERCENTILE / 100.0 * (ridge - threshold)
            layer = ridge - max(top - tolerance, threshold)
            models.append(ShapeModel(
                name, _top_layer_height(threshold, ridge, tolerance),
                max(length, base), min(length, base), 1.0, 2 * layer / _TAN_60 / base,
                yaw_offset=0.0 if length >= base else math.pi / 2,     # object x = ridge
                yaw_period=math.pi, box=(length, side, ridge)))
        else:
            raise ValueError(f'unknown shape {shape!r}')
    return models


def wrap_to_period(angle, period):
    """Angle reduced to (-period/2, period/2]; unchanged for period 0."""
    if period <= 0:
        return angle
    angle = math.fmod(angle, period)
    if angle > period / 2:
        angle -= period
    elif angle <= -period / 2:
        angle += period
    return angle


def measure(points, cell_heights, cell, flat_tolerance):
    """Features of one cluster.

    `points`: N x 3 in the bin frame; `cell_heights`: mean height of every occupied footprint
    cell of the top-view grid; `cell`: grid cell size.
    """
    xy = np.ascontiguousarray(points[:, :2], dtype=np.float32)
    corners = cv2.boxPoints(cv2.minAreaRect(xy)).astype(np.float64)
    edges = [corners[1] - corners[0], corners[2] - corners[1]]
    lengths = [float(np.hypot(*edge)) for edge in edges]
    major = 0 if lengths[0] >= lengths[1] else 1
    long_axis = wrap_to_period(math.atan2(edges[major][1], edges[major][0]), math.pi)

    top = np.percentile(points[:, 2], _TOP_PERCENTILE) - flat_tolerance
    height = float(np.median(points[points[:, 2] > top, 2]))
    area = len(cell_heights) * cell * cell
    rectangle = max(lengths[0] * lengths[1], 1e-9)
    return Features(
        centre=tuple(corners.mean(axis=0)),
        long=lengths[major], short=lengths[1 - major], long_axis=long_axis, height=height,
        fill=area / rectangle, flat=float(np.mean(cell_heights > top)), n_points=len(points))


def residual(features, model, sigma):
    """Mean squared normalised difference between measured and expected features."""
    terms = (
        (features.height - model.height) / sigma['height'],
        (features.long - model.long) / sigma['size'],
        (features.short - model.short) / sigma['size'],
        (features.fill - model.fill) / sigma['fill'],
        (features.flat - model.flat) / sigma['flat'],
    )
    return sum(term * term for term in terms) / len(terms)


def classify(features, models, params):
    """Best catalog hypothesis for a cluster; `unknown` when no shape explains it well enough.

    score = exp(-residual / 2) of the best shape, in (0, 1].
    """
    costs = {model.name: residual(features, model, params['sigma']) for model in models}
    best = min(models, key=lambda model: costs[model.name])
    score = math.exp(-costs[best.name] / 2.0)
    x, y = features.centre
    if score < params['min_score']:
        size = (features.long, features.short, features.height)
        return Detection(UNKNOWN, score, (x, y, features.height / 2), features.long_axis,
                         math.pi, size, features, costs)
    yaw = 0.0 if best.yaw_period == 0 else wrap_to_period(
        features.long_axis + best.yaw_offset, best.yaw_period)
    return Detection(best.name, score, (x, y, best.box[2] / 2), yaw, best.yaw_period, best.box,
                     features, costs)
