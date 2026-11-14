#!/usr/bin/env python3
"""Benchmark of the Rubik's cube cell (phase 7, E-007 parts D, E, F): one cube per run.

A cube scrambled with seeded random moves is spawned in the nest in a seeded orientation and
the cell works on it alone (`openfactory_manipulation.rubik.RubikSolver`). The harness reads
the simulator after every flip and turn, and at the end, for evaluation only.

    --part scan       the cell looks at the six faces: the state and the orientation it reads
                      must be the simulator's
    --part solve      the cell scans, solves and scans again: the cube must be solved in the
                      simulator too (rubik_eval.judge_cube)
    --part scramble   the cube starts solved; the arm scrambles it with the seeded moves, then
                      solves it

One cube is one invocation, `--index` among the cubes of the seed: the Gazebo server leaks
memory at every change of axis of the cube (docs/environment.md), so the cell is relaunched
between cubes (scripts/vm/run_rubik.sh). Every invocation appends to the same run directory;
`--summarize` writes its summary.

Run in the VM with `openfactory_bringup rubik.launch.py` launched:
    python3 scripts/rubik_benchmark.py --part solve --set dev --index 0 --run-id p7_dev
    python3 scripts/rubik_benchmark.py --summarize --run-id p7_dev
Logs: ~/openfactory_data/trials/<run_id>/ (cubes.jsonl, operations.jsonl, summary.json).
"""

import argparse
import json
import random
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.srv import GetStateValidity
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import rotations as rot
from openfactory_manipulation.moveit_interface import ManipulationError
from openfactory_manipulation.rubik import RubikSolver, Stop, load_rubik_configs
from rclpy.node import Node
from rclpy.parameter import Parameter

import rubik_eval as ev
import trial_log as tl
from phase2_pick_place_test import VALIDITY_SERVICE, StateRecorder
from rubik_cell_test import count_collisions
from rubik_scene import CubeScene

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / 'scripts' / 'config' / 'phase7_rubik.yaml'


def cube_of(seed, index, moves):
    """(scramble, start orientation) of cube `index` of a seed."""
    rng = random.Random(f'{seed}:{index}')
    return c.random_scramble(rng, moves), rot.ROTATIONS[rng.randrange(24)]


def server_memory_mb():
    """Resident memory of the Gazebo server, MB (None if it is not found)."""
    out = subprocess.run(['ps', '-eo', 'rss,args'], capture_output=True, text=True).stdout
    sizes = [int(line.split()[0]) for line in out.splitlines() if 'gz sim' in line]
    return max(sizes) / 1024 if sizes else None


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def summarize(run_dir):
    cubes, operations = read(run_dir / 'cubes.jsonl'), read(run_dir / 'operations.jsonl')
    summary = {**(cubes[0]['meta'] if cubes else {}), **ev.summarize(cubes, operations),
               'indices': [cube['index'] for cube in cubes]}
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--part', choices=('scan', 'solve', 'scramble'), default='solve')
    parser.add_argument('--set', default='dev', dest='seed_set')
    parser.add_argument('--index', type=int, default=0)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--summarize', action='store_true')
    parser.add_argument('--dev-solver', choices=('inverse',),
                        help='development aid, never for a result: instead of kociemba, undo '
                             'the scramble move by move (only from the state as spawned)')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    run_dir = args.out / args.run_id
    if args.summarize:
        print(json.dumps(summarize(run_dir), indent=2))
        return 0
    run_dir.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(args.config.read_text())
    seed, tol = cfg['seeds'][args.seed_set], cfg['evaluation']
    scramble, orientation = cube_of(seed, args.index, cfg['scramble_moves'])
    by_robot = args.part == 'scramble'
    start_state = c.SOLVED if by_robot else c.apply_moves(c.SOLVED, scramble)
    meta = {'run_id': args.run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'part': args.part, 'set': args.seed_set}

    scene = CubeScene()
    rclpy.init()
    node = Node('rubik_benchmark',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    validity = node.create_client(GetStateValidity, VALIDITY_SERVICE)
    recorder = StateRecorder(node, tol['record_period_s'])
    operations = []

    def on_start(operation):
        recorder.start()

    def on_done(record):
        """Ground truth after an operation, next to what the cell believes."""
        states = recorder.stop()
        cell.moveit.spin_for(tol['settle_s'])
        truth, where = scene.truth()
        invalid, contacts = count_collisions(node, validity, states)
        square = truth.square(tol['square_position_m'], tol['square_angle_rad'])
        truth_ok = square and ev.seated(where, tol)
        operations.append({
            **meta, 'cube': args.index, 'index': record['index'], 'phase': record['phase'],
            'kind': record['operation'][0], 'operation': record['operation'],
            'cell_success': record['cell_success'],
            'cell_failure_reason': record['cell_failure_reason'],
            'failed_stage': record['failed_stage'], 'view': record['view'],
            'view_problem': record['view_problem'], 'total_s': record['total_s'],
            'planning_s': record['planning_s'], 'execution_s': record['execution_s'],
            'truth_state': truth.state, 'truth_square': square,
            'truth_orientation': where and [list(row) for row in where['orientation']],
            'truth_offset_m': where and list(where['offset_m']),
            'truth_tilt_rad': where and where['tilt_rad'], 'truth_ok': truth_ok,
            'states_checked': len(states), 'states_in_collision': invalid,
            'contacts': sorted(contacts), 'stages': record['stages'],
        })

    def undo_scramble(state):
        if state != start_state:
            raise ValueError('the development solver only knows the state as spawned')
        return [c.format_move(face, -turns) for face, turns in
                (c.parse_move(move) for move in reversed(scramble))]

    extra = {'solve': undo_scramble} if args.dev_solver else {}
    meta['solver'] = args.dev_solver or 'kociemba'
    try:
        cell = RubikSolver(node, *load_rubik_configs(), on_start=on_start, on_done=on_done,
                           **extra)
        cell.wait_ready()
        cell.moveit.clear_objects()
        if not cell.handler.go_ready():
            raise ManipulationError('the arm does not reach ready')
        scene.spawn(start_state, orientation)
        cell.moveit.spin_for(2 * tol['settle_s'])
        truth, where = scene.truth()
        if truth.state != start_state or where is None or where['orientation'] != orientation:
            print(f'the spawned cube is not where it should be: {truth.error} {where}')
            return 2
        sim_start, wall_start = node.get_clock().now(), time.monotonic()
        memory_start = server_memory_mb()

        if args.part == 'scan':
            try:
                cell.scan()
                end_reason = None
            except Stop as stop:
                end_reason = stop.reason
            result = {'solved': False, 'end_reason': end_reason, 'state': cell.state,
                      'solution': None, 'scans': cell.scans, 'log': cell.log,
                      'operations': len(operations)}
        else:
            result = cell.run(scramble if by_robot else None)
        cell.moveit.clear_objects()

        sim_s = (node.get_clock().now() - sim_start).nanoseconds * 1e-9
        truth, where = scene.truth()
        square = truth.square(tol['square_position_m'], tol['square_angle_rad'])
        collisions = sum(o['states_in_collision'] for o in operations)
        if args.part == 'scan':
            success, reason = ev.judge_scan(
                cell.state, cell.orientation, truth.state, where and where['orientation'])
        else:
            success, reason = ev.judge_cube(result, truth.state, square, where, collisions, tol)
        # What the cell believed after every operation it took as done, against the
        # simulator: a difference is a failure the cell did not detect. During a scan the
        # cell does not know the state yet, and counts its orientation from where it began.
        log = {r['index']: r for r in result['log'] if 'verified' in r}
        for operation in operations:
            entry = log.get(operation['index'], {})
            operation['verified'] = entry.get('verified')
            if not entry.get('verified'):
                operation['belief_ok'] = None
                continue
            believed = entry.get('believed_state')
            agree = bool(operation['truth_ok']) and (
                believed is None or believed == operation['truth_state'])
            if operation['phase'] != 'scan':
                agree = agree and entry.get('believed_orientation') == \
                    operation['truth_orientation']
            operation['belief_ok'] = agree
        record = {
            'meta': meta, 'index': args.index, 'scramble': scramble,
            'orientation': [list(row) for row in orientation], 'scrambled_by_robot': by_robot,
            'success': success, 'failure_reason': reason,
            'cell_solved': result['solved'], 'end_reason': result['end_reason'],
            'cell_state': result['state'], 'truth_state': truth.state,
            'truth_solved': truth.state == c.SOLVED, 'truth_square': square,
            'truth_seated': ev.seated(where, tol),
            'position_error_m': truth.position_error, 'angle_error_rad': truth.angle_error,
            'solution': result['solution'], 'scans': result['scans'],
            'operations': len(operations),
            'turns': sum(o['kind'] == 'turn' for o in operations),
            'flips': sum(o['kind'] == 'flip' for o in operations),
            'cell_failures': sum(not o['cell_success'] for o in operations),
            'states_in_collision': collisions, 'sim_s': sim_s,
            'wall_s': time.monotonic() - wall_start,
            'server_memory_mb': [memory_start, server_memory_mb()],
            'log': [{k: v for k, v in r.items() if k not in ('stages', 'view_object')}
                    for r in result['log']],
        }
        with (run_dir / 'operations.jsonl').open('a') as out:
            for operation in operations:
                out.write(json.dumps(operation) + '\n')
        with (run_dir / 'cubes.jsonl').open('a') as out:
            out.write(json.dumps(record) + '\n')
        print(json.dumps({k: record[k] for k in (
            'index', 'success', 'failure_reason', 'cell_solved', 'end_reason', 'truth_solved',
            'truth_square', 'truth_seated', 'scans', 'operations', 'turns', 'flips',
            'cell_failures', 'states_in_collision', 'sim_s', 'wall_s', 'server_memory_mb')},
            indent=2))
        scene.remove()
    except ManipulationError as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
