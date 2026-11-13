#!/usr/bin/env python3
"""Relay-loop benchmark of the ring of N arms: phase 6 (E-006).

The harness spawns seeded catalog objects in every bin once, before the loop starts, and never
touches them again. The ring then works alone (openfactory_manipulation.relay): arm i picks
from bin i and places into bin (i + 1) mod N, one arm at a time, until the number of handovers
asked for was tried or the loop stops on its own. After every turn the harness reads simulator
ground truth, which neither perception nor manipulation sees, and re-checks the executed states
of every arm with MoveIt, once per arm group (an arm against everything else, the other arms
included).
Logs: ~/openfactory_data/trials/<run_id>/ (attempts.jsonl: one record per handover tried;
turns.jsonl: one per turn; scene.json; summary.json).

Run in the VM with the ring launched (ros2 launch openfactory_bringup ring.launch.py n:=N):
    python3 scripts/relay_benchmark.py --arms N [--config FILE] [--set dev|eval]
                                       [--handovers K]
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.srv import GetStateValidity
from openfactory_description import ring
from openfactory_manipulation.core import relay as policy
from openfactory_manipulation.pick_place_once import load_configs
from openfactory_manipulation.relay import RelayLoop
from rclpy.node import Node
from rclpy.parameter import Parameter

import phase3_common as common
import randomize_scene as rs
import relay_eval as re
import scene_objects as so
import trial_log as tl
from gz_scene import GzScene, load_config
from phase2_pick_place_test import VALIDITY_SERVICE, StateRecorder, count_invalid

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase6_relay.yaml'
TURN_FIELDS = ('run_id', 'turn', 'arm', 'source_bin', 'destination_bin', 'skipped', 'parked',
               'success', 'failure_reason', 'states_checked', 'states_invalid', 'disturbed',
               'lost', 'total_s')


def sample_ring(seed, n, per_bin, catalog, cell, params):
    """Objects of every bin, [{'name', 'bin', 'type', 'rest', 'x', 'y', 'yaw'}] in the frame
    of their bin: bin k holds scene k of the seed."""
    objects = []
    for index in range(n):
        objects += re.scene_names(
            index, rs.sample_scene(seed, index, per_bin, catalog, cell, params))
    return objects


def all_names(catalog, n, per_bin):
    """Every model name a run of this harness may have left in the world."""
    return [f'b{index}_{name}' for index in range(n)
            for name in common.object_names(catalog, per_bin)]


def spawn_ring(scene, objects, catalog, bin_poses):
    for item in objects:
        spec, rest = catalog['objects'][item['type']], item.get('rest', so.CATALOG_REST)
        xyz, rpy = re.spawn_pose(bin_poses[ring.bin_name(item['bin'])], spec, item['x'],
                                 item['y'], item['yaw'], rest)
        scene.spawn(item['name'], so.model_sdf(item['name'], spec, catalog['friction']), xyz, rpy)


def read_states(ctx):
    """Ground-truth state of every object, from two readings a moment apart."""
    names = [item['name'] for item in ctx['objects']]
    ev_cfg = ctx['cfg']['evaluation']
    first = ctx['scene'].model_poses(names)
    common.spin_for(ctx['node'], ev_cfg['rest_interval_s'])
    second = ctx['scene'].model_poses(names)
    return re.object_states(ctx['objects'], first, second, ctx['bin_poses'], ctx['inner_size'],
                            ev_cfg)


def spawned_at_rest(ctx, states):
    """True if every object rests in the bin it was spawned in."""
    return all(states[item['name']]['location'] == ring.bin_name(item['bin'])
               and states[item['name']]['at_rest'] for item in ctx['objects'])


def turn_records(ctx, turn, before, after, states):
    """(turn record, handover record or None) of one turn of the loop."""
    ev_cfg, attempt = ctx['cfg']['evaluation'], turn['attempt']
    contacts = set()
    invalid = count_invalid(ctx['node'], ctx['validity'], states, ctx['joints'], ctx['groups'],
                            contacts)
    target_name = None
    if attempt is not None:
        target = attempt['target']
        target_name = re.match_target(
            target['x'], target['y'], before, turn['source_bin'],
            ctx['bin_poses'][turn['source_bin']], ev_cfg['match_distance_m'])
    evaluation = re.evaluate_turn(target_name, before, after, turn['destination_bin'], ev_cfg)
    light = {'run_id': ctx['meta']['run_id'],
             **{key: turn[key] for key in ('turn', 'arm', 'source_bin', 'destination_bin',
                                           'skipped', 'parked', 'success', 'failure_reason',
                                           'survey_s', 'total_s')},
             'bins_seen': turn['bins_seen'], 'free_slots': turn['free_slots'],
             'target_waiting_since': turn['target_waiting_since'],
             'states_checked': len(states), 'states_invalid': invalid,
             'invalid_contacts': sorted(list(pair) for pair in contacts),
             'disturbed': evaluation['disturbed'], 'lost': evaluation['lost']}
    if attempt is None:
        return light, None

    manipulation, target = attempt['manipulation'], attempt['target']
    success, reason, detection = re.handover_result(
        turn['success'], turn['failure_reason'], evaluation, invalid)
    truth = None if target_name is None else before[target_name]
    perception_error = None
    if truth is not None:
        tx, ty, _ = re.in_bin(ctx['bin_poses'][turn['source_bin']], truth['xyz'])
        perception_error = {'truth_type': truth['type'],
                            'xy_error_m': ((tx - target['x']) ** 2
                                           + (ty - target['y']) ** 2) ** 0.5}
    record = {
        **ctx['meta'],
        'trial_id': turn['turn'],
        'attempt': ctx['tried'],
        'arm': turn['arm'],
        'source_bin': turn['source_bin'],
        'destination_bin': turn['destination_bin'],
        'object_category': evaluation['target_type'] or target['class_id'],
        'objects_seen': attempt['objects'],
        'ranking': attempt['strategy'],
        'selected_grasp': {**target, 'frame': turn['source_bin'], 'tcp': manipulation['grasp']},
        'candidates': attempt['candidates'],
        'candidates_total': attempt['candidates_total'],
        'candidates_rejected': attempt['candidates_rejected'],
        'place': attempt['place'],
        'planner_result': {s['stage']: s['moveit_error_code'] for s in manipulation['stages']
                           if 'moveit_error_code' in s},
        'execution_result': {
            'cell_success': turn['success'],
            'cell_failure': turn['failure_reason'],
            'pick_success': attempt['success'],
            'pick_failure': attempt['failure_reason'],
            'manipulation_success': manipulation['success'],
            'manipulation_failure': manipulation['failure_reason'],
            'failed_stage': manipulation['failed_stage'],
            'recovered': manipulation['recovered'],
            'parked': turn['parked'],
        },
        'verification': attempt['verification'],
        'arrival': turn['arrival'],
        'perception_error': perception_error,
        'evaluation': {**evaluation, 'states_checked': len(states), 'states_invalid': invalid,
                       'invalid_contacts': light['invalid_contacts']},
        'detection': detection,
        'success': success,
        'failure_reason': reason,
        'perception_s': attempt['perception_s'] + turn['survey_s'],
        'selection_s': attempt['selection_s'],
        'planning_s': attempt['planning_s'],
        'execution_s': attempt['execution_s'],
        'total_s': turn['total_s'],
        'stages': manipulation['stages'],
    }
    return light, record


def validate_turn(record):
    missing = [name for name in TURN_FIELDS if name not in record]
    if missing:
        raise ValueError(f'missing fields: {missing}')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--arms', type=int, required=True, help='arms of the launched ring')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--set', default='eval', dest='which', help='set of the config')
    parser.add_argument('--handovers', type=int, help='overrides the set (smoke runs)')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    n = ring.check_n(args.arms)
    chosen = cfg['sets'][args.which]
    seed, wanted = chosen['seed'], args.handovers or chosen['handovers']
    full = args.handovers is None
    pp_cfg, catalog, gripper = load_configs(cfg['pick_place_config'])
    _, cell = load_config()                  # installed openfactory_description, as the ring uses
    layout = ring.load_layout(n)

    run_id = (f"{cfg['run_name']}_n{n}_{args.which}_"
              f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{seed}")
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'strategy': cfg['strategy']}
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)

    rclpy.init()
    node = Node('relay_benchmark',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    handovers, turns = [], []
    try:
        loop = RelayLoop(node, pp_cfg, catalog, gripper, cell, n)
        loop.wait_ready()
        validity = node.create_client(GetStateValidity, VALIDITY_SERVICE)
        if not validity.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{VALIDITY_SERVICE} not available')
        joints = [name for arm in range(n)
                  for name in (*ring.arm_joints(arm), *ring.finger_joints(arm))]
        ctx = {'cfg': cfg, 'catalog': catalog, 'meta': meta, 'node': node, 'scene': GzScene(),
               'validity': validity, 'joints': joints, 'tried': 0,
               'groups': [ring.planning_group(arm) for arm in range(n)],
               'bin_poses': ring.bin_poses(layout), 'inner_size': layout['bin_inner_size'],
               'recorder': StateRecorder(node, cfg['evaluation']['record_period_s'], joints)}
        names = all_names(catalog, n, cfg['objects_per_bin'])
        common.clear_objects(ctx['scene'], names)
        loop.reset(seed=seed)                # empty planning scene, every arm parked
        ctx['objects'] = sample_ring(seed, n, cfg['objects_per_bin'], catalog, cell,
                                     cfg['scene'])
        spawn_ring(ctx['scene'], ctx['objects'], catalog, ctx['bin_poses'])
        common.spin_for(node, cfg['evaluation']['settle_s'])
        before = read_states(ctx)
        (out_dir / 'scene.json').write_text(json.dumps(
            {**meta, 'arms': n, 'objects': ctx['objects'], 'initial_states': before},
            indent=2) + '\n')

        end_reason, idle = None, 0
        if not spawned_at_rest(ctx, before):
            end_reason = 'spawn_failed'
        while end_reason is None:
            ctx['recorder'].start()
            result = loop.step()
            states = ctx['recorder'].stop()
            turn = result['turn']
            if turn is not None:
                common.spin_for(node, cfg['evaluation']['settle_s'])
                after = read_states(ctx)
                light, record = turn_records(ctx, turn, before, after, states)
                tl.append(out_dir / 'turns.jsonl', light, check=validate_turn)
                turns.append(light)
                if record is None:
                    idle += 1
                    what = f"skipped ({turn['skipped']})"
                else:
                    idle = 0
                    ctx['tried'] += 1
                    tl.append(out_dir / 'attempts.jsonl', record)
                    handovers.append(record)
                    what = (f"{record['object_category']} {record['selected_grasp']['label']}: "
                            f"{'ok' if record['success'] else record['failure_reason']}"
                            f"{'' if record['detection'] == 'agree' else ' [' + record['detection'] + ']'}")
                before = after
                node.get_logger().info(
                    f"turn {turn['turn']} arm {turn['arm']} {turn['source_bin']} -> "
                    f"{turn['destination_bin']} {what}, {turn['total_s']:.1f} s, invalid "
                    f"{light['states_invalid']}/{light['states_checked']}, handovers "
                    f"{sum(r['success'] for r in handovers)}/{ctx['tried']}")
            end_reason = policy.run_end(result['end_reason'], ctx['tried'], wanted, idle, n)
        final = before
        common.clear_objects(ctx['scene'], names)
        loop.moveit.clear_objects()          # leave no obstacles behind in move_group
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = {**meta, 'set': args.which, 'full_set': full,
               'ranking': pp_cfg['ranking']['strategy'],
               **re.summarize(handovers, turns, final, n, end_reason, wanted, cfg['thresholds'])}
    summary['passed'] = summary['passed'] and full
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
