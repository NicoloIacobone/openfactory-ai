#!/usr/bin/env python3
"""Seeded scene randomization: which catalog objects rest where in the input bin.

The sampler is ROS-free and deterministic: the same seed, scene index, catalog, cell layout and
parameters give the same scene. Phase 3 scenes hold separate objects resting in their catalog
orientation (random position and yaw); clutter and tumbled objects belong to phase 4.

Run in the VM with the cell launched to spawn a scene by hand:
    python3 scripts/randomize_scene.py --seed 1 --index 0 --objects 3 [--spawn]
"""

import argparse
import json
import math
import random
import sys

from openfactory_manipulation.core.objects import bounding_box

MAX_TRIES = 2000         # positions tried for one object
MAX_LAYOUTS = 20         # whole layouts tried for one scene


def footprint_radius(spec):
    """Radius of the circle around the object's footprint, about its frame origin."""
    x, y, _ = bounding_box(spec)
    if spec['shape'] == 'cylinder':
        return x / 2
    return math.hypot(x, y) / 2


def scene_rng(seed, index):
    """Generator for scene `index` of a run: independent of every other scene of the run."""
    return random.Random(f'{seed}:{index}')


def sample_scene(seed, index, n_objects, catalog, cell, params, types=None):
    """Objects of one scene: [{'name', 'type', 'x', 'y', 'yaw'}, ...] in the input_bin frame.

    `params`: wall_margin (footprint to the bin walls), min_gap (between footprints) and
    optionally region (half extents [x, y] that the object centres stay inside). `types`
    fixes the object types (one per object); by default they are drawn from the catalog.
    """
    rng = scene_rng(seed, index)
    inner = cell['bins']['input']['inner_size']
    names = sorted(catalog['objects'])
    if types is None:
        types = [rng.choice(names) for _ in range(n_objects)]
    elif len(types) != n_objects:
        raise ValueError('one type per object')
    region = params.get('region')

    limits = []
    for object_type in types:
        radius = footprint_radius(catalog['objects'][object_type])
        half_x = inner[0] / 2 - params['wall_margin'] - radius
        half_y = inner[1] / 2 - params['wall_margin'] - radius
        if region is not None:
            half_x, half_y = min(half_x, region[0]), min(half_y, region[1])
        if half_x < 0 or half_y < 0:
            raise ValueError(f'{object_type} does not fit the bin with these margins')
        limits.append((radius, half_x, half_y))

    # Objects are placed one after the other. In a crowded bin the first ones can leave no room
    # for the last: the layout is then drawn again from the same generator, so the scene is
    # still a function of the seed. A scene that fits at the first layout is unchanged.
    for _ in range(MAX_LAYOUTS):
        placed, stuck = [], None
        for number, (object_type, (radius, half_x, half_y)) in enumerate(zip(types, limits)):
            for _ in range(MAX_TRIES):
                x, y = rng.uniform(-half_x, half_x), rng.uniform(-half_y, half_y)
                if all(math.hypot(x - other['x'], y - other['y'])
                       >= radius + other['radius'] + params['min_gap'] for other in placed):
                    break
            else:
                stuck = (number, object_type)
                break
            placed.append({'name': f'{object_type}_{number}', 'type': object_type, 'x': x,
                           'y': y, 'yaw': rng.uniform(-math.pi, math.pi), 'radius': radius})
        if stuck is None:
            return [{key: value for key, value in item.items() if key != 'radius'}
                    for item in placed]
    raise ValueError(f'no room for object {stuck[0]} ({stuck[1]}) in scene {index}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--index', type=int, default=0, help='scene number within the seed')
    parser.add_argument('--objects', type=int, default=3)
    parser.add_argument('--wall-margin', type=float, default=0.012)
    parser.add_argument('--min-gap', type=float, default=0.02)
    parser.add_argument('--spawn', action='store_true', help='spawn the scene in Gazebo')
    args = parser.parse_args()

    # simulator access only here: the sampler above stays importable without ROS or Gazebo
    import scene_objects as so
    from gz_scene import GzScene, load_config

    catalog, cell = load_config()
    params = {'wall_margin': args.wall_margin, 'min_gap': args.min_gap}
    scene = sample_scene(args.seed, args.index, args.objects, catalog, cell, params)
    print(json.dumps(scene, indent=2))
    if args.spawn:
        gz = GzScene()
        for item in scene:
            spec = catalog['objects'][item['type']]
            xyz, yaw = so.spawn_pose(cell, 'input', spec, item['x'], item['y'], item['yaw'])
            gz.spawn(item['name'], so.model_sdf(item['name'], spec, catalog['friction']), xyz,
                     (0.0, 0.0, yaw))
    return 0


if __name__ == '__main__':
    sys.exit(main())
