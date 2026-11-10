"""Object catalog geometry (openfactory_description/config/objects.yaml), shared by the
manipulation code and the simulation harness (scripts/scene_objects.py)."""

import math

SHAPES = ('box', 'cylinder', 'triangular_prism')


def prism_height(spec):
    """Height of the equilateral cross-section of a triangular prism."""
    return spec['side'] * math.sqrt(3.0) / 2.0


def bounding_box(spec):
    """Axis-aligned size [x, y, z] in the object frame (centred on the object origin)."""
    shape = spec['shape']
    if shape == 'box':
        return list(spec['size'])
    if shape == 'cylinder':
        return [2 * spec['radius'], 2 * spec['radius'], spec['length']]
    if shape == 'triangular_prism':
        return [spec['length'], spec['side'], prism_height(spec)]
    raise ValueError(f'unknown shape {shape!r}')
