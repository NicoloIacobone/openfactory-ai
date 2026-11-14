#!/usr/bin/env python3
"""Turns and flips of the Rubik's cube cell against ground truth (phase 7, E-007 parts B, C).

A scrambled cube is spawned in the nest; the arm then makes a seeded sequence of operations
(`CubeHandler.turn`, `CubeHandler.flip`) on its own. After each one the harness reads the
simulator: the cube's state and orientation must be those the logical model predicts
(`core/rubik/orientation.py`), the cube must sit in the nest with every layer square, and no
executed robot state may be in collision.

    --part turn     quarter and half turns of whatever face is down
    --part flip     flips of every kind and yaw
    --ops "turn:1 flip:over:0 turn:2"   an explicit sequence, for debugging

Run in the VM with `openfactory_bringup rubik.launch.py` launched:
    scripts/vm/run_benchmark.sh p7_turn --script rubik_cell_test.py --part turn --count 60
Logs: ~/openfactory_data/trials/<run_id>/ (attempts.jsonl, summary.json).
"""

import argparse
import json
import math
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetStateValidity
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import orientation as ori
from openfactory_manipulation.core.rubik import plan
from openfactory_manipulation.core.rubik import rotations as rot
from openfactory_manipulation.cube_handling import CubeHandler
from openfactory_manipulation.pick_place_once import load_configs
from rclpy.node import Node
from rclpy.parameter import Parameter

import trial_log as tl
from moveit_msgs.msg import RobotState
from openfactory_manipulation.cube_handling import BASE_ID, GRIPPER_FILE, TOP_ID
from phase2_pick_place_test import RECORDED_JOINTS, VALIDITY_SERVICE, StateRecorder
from sensor_msgs.msg import JointState
from rubik_scene import CubeScene

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / 'scripts' / 'config' / 'phase7_rubik.yaml'


# The fingers hold the cube: these contacts are the task. Anything else is a collision.
HOLDING = {tuple(sorted((finger, part)))
           for finger in ('left_finger', 'right_finger') for part in (BASE_ID, TOP_ID)}


def count_collisions(node, client, states, group='ur_manipulator'):
    """(recorded states in collision, pairs of bodies in contact that are not the fingers on
    the cube). The states are checked against the planning scene as the operation left it:
    fixtures and the seated cube."""
    invalid, pairs = 0, set()
    for state in states:
        robot_state = RobotState(joint_state=JointState(name=list(RECORDED_JOINTS),
                                                        position=state))
        future = client.call_async(
            GetStateValidity.Request(group_name=group, robot_state=robot_state))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            invalid += 1
            pairs.add(('no_answer', group))
            continue
        if future.result().valid:
            continue
        found = {tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
                 for contact in future.result().contacts} - HOLDING
        if found or not future.result().contacts:
            invalid += 1
            pairs.update(found or {('unknown', group)})
    return invalid, pairs


def predict(state, orientation, op, flips):
    """(state, orientation) the logical model expects after `op`."""
    return plan.apply(state, orientation, op, flips)


def draw_ops(part, count, rng, kinds):
    if part == 'turn':
        return [('turn', rng.choice((1, -1, 2))) for _ in range(count)]
    if part == 'flip':
        return [('flip', rng.choice(kinds), rng.choice((0, 1, 2, 3))) for _ in range(count)]
    # mixed: a flip brings another face down, then one or two turns
    ops = []
    while len(ops) < count:
        ops.append(('flip', rng.choice(kinds), rng.choice((0, 1, 2, 3))))
        ops += [('turn', rng.choice((1, -1, 2))) for _ in range(rng.choice((1, 2)))]
    return ops[:count]


def parse_ops(text):
    ops = []
    for word in text.split():
        parts = word.split(':')
        ops.append(('turn', int(parts[1])) if parts[0] == 'turn'
                   else ('flip', parts[1], int(parts[2]) if len(parts) > 2 else 0))
    return ops


def judge(result, truth, where, expected_state, expected_orientation, invalid, tol):
    """(success, failure reason) of one operation."""
    seated = (where is not None
              and math.dist(where['offset_m'], (0.0, 0.0, 0.0)) <= tol['seat_offset_m']
              and where['tilt_rad'] <= tol['seat_tilt_rad'])
    checks = (
        ('cell_failed', result['success']),
        ('collision', invalid == 0),
        ('not_seated', seated),
        ('wrong_orientation', where is not None
         and where['orientation'] == expected_orientation),
        ('not_square', truth.square(tol['square_position_m'], tol['square_angle_rad'])),
        ('wrong_state', truth.state == expected_state),
    )
    return next(((False, name) for name, ok in checks if not ok), (True, None))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--part', choices=('turn', 'flip', 'mixed'), default='turn')
    parser.add_argument('--ops', help='explicit operations instead of a seeded sequence')
    parser.add_argument('--count', type=int)
    parser.add_argument('--set', default='dev', dest='seed_set')
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--orientation', type=int, help='start orientation, 0..23')
    parser.add_argument('--stop-on-failure', action='store_true')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    seed = cfg['seeds'][args.seed_set]
    tol = cfg['evaluation']
    config, _, gripper = load_configs(gripper_file=GRIPPER_FILE)
    share = Path(get_package_share_directory('openfactory_manipulation')) / 'config'
    rubik = yaml.safe_load((share / 'rubik.yaml').read_text())
    scene = CubeScene()
    kinds = sorted(rubik['flip']['kinds'])
    flips = plan.kind_rotations(rubik['flip']['kinds'])

    rng = random.Random(seed)
    state = c.apply_moves(c.SOLVED, c.random_scramble(rng))
    orientation = rot.ROTATIONS[args.orientation if args.orientation is not None
                                else rng.randrange(24)]
    ops = parse_ops(args.ops) if args.ops else draw_ops(
        args.part, args.count or cfg['parts'][args.part]['count'], rng, kinds)

    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    run_id = f'{cfg["run_name"]}_{args.part}_{args.seed_set}_{stamp}_s{seed}'
    run_dir = args.out / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node('rubik_cell_test',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    validity = node.create_client(GetStateValidity, VALIDITY_SERVICE)
    recorder = StateRecorder(node, tol['record_period_s'])
    handler = CubeHandler(node, config, rubik, scene.configs['rubik'], gripper)
    handler.wait_ready()
    handler.moveit.clear_objects()

    scene.spawn(state, orientation)
    handler.moveit.spin_for(tol['settle_s'])
    truth, where = scene.truth()
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'part': args.part}
    if truth.state != state or where is None or where['orientation'] != orientation:
        print(f'the spawned cube is not where it should be: {truth.error} {where}')
        return 2

    records, wall = [], time.monotonic()
    for index, op in enumerate(ops):
        expected_state, expected_orientation = predict(state, orientation, op, flips)
        recorder.start()
        result = handler.turn(op[1]) if op[0] == 'turn' else handler.flip(op[1], op[2])
        states = recorder.stop()
        handler.moveit.spin_for(tol['settle_s'])
        truth, where = scene.truth()
        invalid, contacts = count_collisions(node, validity, states)
        success, reason = judge(result, truth, where, expected_state, expected_orientation,
                                invalid, tol)
        record = {
            **meta, 'trial_id': index, 'operation': ' '.join(str(v) for v in op),
            'down_face': ori.down_face(orientation),
            'move': ori.turn_move(orientation, op[1]) if op[0] == 'turn' else None,
            'success': success, 'failure_reason': reason,
            'cell_success': result['success'], 'cell_failure_reason': result['failure_reason'],
            'failed_stage': result['failed_stage'], 'recovered': result['recovered'],
            'state': truth.state, 'expected_state': expected_state,
            'position_error_m': truth.position_error, 'angle_error_rad': truth.angle_error,
            'offset_m': where and list(where['offset_m']),
            'tilt_rad': where and where['tilt_rad'],
            'orientation_ok': where is not None and where['orientation'] == expected_orientation,
            'states_checked': len(states), 'states_in_collision': invalid,
            'contacts': sorted(contacts),
            'planning_s': result['planning_s'], 'execution_s': result['execution_s'],
            'total_s': result['total_s'], 'stages': result['stages'],
        }
        records.append(record)
        with (run_dir / 'attempts.jsonl').open('a') as out:
            out.write(json.dumps(record) + '\n')
        print(f'{index:3d} {record["operation"]:16s} {"ok" if success else reason:18s} '
              f'cell {result["failure_reason"] or "ok":14s} '
              f'state {"ok" if truth.state == expected_state else "WRONG"} '
              f'orientation {"ok" if record["orientation_ok"] else "WRONG"} '
              f'offset {[round(v * 1000, 2) for v in where["offset_m"]] if where else None} mm '
              f'tilt {math.degrees(where["tilt_rad"]) if where else 0:.2f} deg '
              f'square {truth.position_error * 1000:.3f} mm {result["total_s"]:.1f} s',
              flush=True)
        if not success and (args.stop_on_failure or truth.state is None or where is None):
            break
        # go on from what is really there, so that one failure is counted once
        if truth.state is not None and where is not None:
            state, orientation = truth.state, where['orientation']

    done = len(records)
    good = sum(r['success'] for r in records)
    low, high = tl.wilson_interval(good, done) if done else (None, None)
    reasons = {}
    for r in records:
        if not r['success']:
            reasons[r['failure_reason']] = reasons.get(r['failure_reason'], 0) + 1
    summary = {
        **meta, 'operations': done, 'planned': len(ops), 'successes': good,
        'success_rate': good / done if done else None, 'wilson_95': [low, high],
        'failures': reasons,
        'undetected_failures': sum(1 for r in records if not r['success'] and r['cell_success']),
        'states_checked': sum(r['states_checked'] for r in records),
        'states_in_collision': sum(r['states_in_collision'] for r in records),
        'max_offset_m': max((math.dist(r['offset_m'], (0, 0, 0)) for r in records
                             if r['offset_m']), default=None),
        'max_position_error_m': max((r['position_error_m'] for r in records), default=None),
        'mean_total_s': sum(r['total_s'] for r in records) / done if done else None,
        'wall_s': time.monotonic() - wall,
    }
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    threshold = cfg['parts'].get(args.part, {}).get('min_successes')
    passed = (done == len(ops) and summary['states_in_collision'] == 0
              and (threshold is None or args.ops or good >= threshold * done / max(
                  1, cfg['parts'][args.part]['count'])))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
