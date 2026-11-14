#!/usr/bin/env python3
"""The Rubik's cube in the running cell (harness tool, phase 7): spawn, remove, ground truth.

The cube is spawned in the nest in a given state and orientation (`rubik_model.cube_sdf`) and
then never touched: only the arm moves it. Its state, where it sits and how square it is are
read from the simulator's link poses (`rubik_truth`), for evaluation only (rule 5).

Run in the VM with `openfactory_bringup rubik.launch.py` launched:
    python3 scripts/rubik_scene.py spawn [--seed 3 | --moves "R U F'"] [--orientation 0..23]
    python3 scripts/rubik_scene.py truth
    python3 scripts/rubik_scene.py remove
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import rotations as rot

import rubik_model as rm
import rubik_truth as rt
from gz_scene import GzScene

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def load_config():
    share = Path(get_package_share_directory('openfactory_description')) / 'config'
    return {name: yaml.safe_load((share / f'{name}.yaml').read_text())
            for name in ('rubik', 'rubik_cell', 'cell')}


def nest_xyz(cell, layout):
    """Origin of the nest frame in the world (the table is not rotated)."""
    table, pedestal = cell['table'], layout['pedestal']
    return (table['xyz'][0] + pedestal['xy'][0], table['xyz'][1] + pedestal['xy'][1],
            table['xyz'][2] + table['size'][2] + pedestal['height'])


def seat(config):
    """Centre of a cube seated in the nest, in the nest frame."""
    return (0.0, 0.0, 1.5 * config['cube']['pitch'])


def spawn_pose(cell, layout, config, orientation, drop=0.002):
    """(xyz, rpy) of the cube model in the world for a cube seated in `orientation` (the
    rotation from the cube frame to the nest frame), released `drop` above its seat."""
    nest, centre = nest_xyz(cell, layout), seat(config)
    xyz = (nest[0] + centre[0], nest[1] + centre[1], nest[2] + centre[2] + drop)
    return xyz, rot.rpy_of(orientation)


def cube_in_nest(poses, cell, layout, config):
    """Where the cube is: {orientation, tilt_rad, offset_m} from its link poses and the model
    pose, or None if they are missing. `orientation` is the nearest of the 24 (the cube frame
    in the nest frame), `tilt_rad` the angle from it, `offset_m` the cube centre minus the
    seat, in the nest frame."""
    if rm.MODEL_NAME not in poses or rt.CORE not in poses:
        return None
    core = g.compose(poses[rm.MODEL_NAME], poses[rt.CORE])
    xyz, quat = g.relative((nest_xyz(cell, layout), IDENTITY), core)
    orientation, tilt = rot.nearest_rotation(rot.from_quaternion(quat))
    return {'orientation': orientation, 'tilt_rad': tilt,
            'offset_m': tuple(a - b for a, b in zip(xyz, seat(config)))}


class CubeScene:
    """The cube of the running cell."""

    def __init__(self, configs=None, world='cell'):
        self.configs = configs or load_config()
        self.scene = GzScene(world)

    def spawn(self, state=c.SOLVED, orientation=rot.IDENTITY):
        self.remove()
        cfg = self.configs
        xyz, rpy = spawn_pose(cfg['cell'], cfg['rubik_cell'], cfg['rubik'], orientation)
        self.scene.spawn(rm.MODEL_NAME, rm.cube_sdf(cfg['rubik'], state), xyz, rpy,
                         timeout_ms=30000)

    def remove(self):
        try:
            self.scene.remove(rm.MODEL_NAME)
        except RuntimeError:
            pass                               # not in the world

    def truth(self):
        """(rubik_truth.Truth, cube_in_nest dict or None)."""
        cfg = self.configs
        poses = self.scene.model_poses(set(rt.LINKS) | {rm.MODEL_NAME})
        return (rt.read(poses, cfg['rubik']['cube']['pitch']),
                cube_in_nest(poses, cfg['cell'], cfg['rubik_cell'], cfg['rubik']))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    spawn = sub.add_parser('spawn', help='spawn a cube in the nest')
    spawn.add_argument('--seed', type=int, help='scramble with 25 random moves of this seed')
    spawn.add_argument('--moves', default='', help='scramble with these moves instead')
    spawn.add_argument('--orientation', type=int, default=None,
                       help='index into the 24 orientations (default: cube frame = nest frame)')
    sub.add_parser('truth', help='print the ground truth of the cube')
    sub.add_parser('remove', help='remove the cube')
    args = parser.parse_args()

    scene = CubeScene()
    if args.command == 'remove':
        scene.remove()
        return 0
    if args.command == 'spawn':
        moves = c.random_scramble(random.Random(args.seed)) if args.seed is not None \
            else c.parse_moves(args.moves)
        orientation = rot.IDENTITY if args.orientation is None else rot.ROTATIONS[args.orientation]
        scene.spawn(c.apply_moves(c.SOLVED, moves), orientation)
        print(f'spawned {rm.MODEL_NAME}: scramble {" ".join(moves) or "(none)"}')
        return 0
    truth, where = scene.truth()
    print(json.dumps({
        'state': truth.state, 'error': truth.error, 'solved': truth.state == c.SOLVED,
        'position_error_mm': round(truth.position_error * 1000, 4),
        'angle_error_deg': round(math.degrees(truth.angle_error), 4),
        'orientation': where and [list(row) for row in where['orientation']],
        'tilt_deg': where and round(math.degrees(where['tilt_rad']), 3),
        'offset_mm': where and [round(v * 1000, 3) for v in where['offset_m']],
    }, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
