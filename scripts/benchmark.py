#!/usr/bin/env python3
"""Bin-picking benchmark over seeded scenes: phase 4 (E-004) and phase 5 (E-005).

For every scene the harness spawns several catalog objects at seeded random poses in the input
bin (between scenes, never during one) and empties the output bin. The cell then works alone
(openfactory_manipulation.bin_picking): it looks at the bin, selects a feasible grasp, picks
and places, verifies with the next view, and repeats until it sees the bin empty or gives up.
After each attempt the harness reads simulator ground truth, which neither perception nor
manipulation sees, and re-checks the executed arm states with MoveIt.
Logs: ~/openfactory_data/trials/<run_id>/ (attempts.jsonl, scenes.jsonl, summary.json).
A config with `strategies` (phase 5) runs the cell with the strategy given by --strategy: which
objects it may try and how it ranks the feasible grasps (D-018, D-019).

Run in the VM with the cell launched (camera and perception on, RViz not needed):
    python3 scripts/benchmark.py [--config FILE] [--set dev|eval] [--strategy NAME]
                                 [--scenes N] [--first N]
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.srv import GetStateValidity
from openfactory_manipulation import constants as c
from openfactory_manipulation.bin_picking import BinPicking
from openfactory_manipulation.pick_place_once import load_configs
from rclpy.node import Node
from rclpy.parameter import Parameter

import bin_picking_eval as be
import phase3_common as common
import randomize_scene as rs
import scene_objects as so
import trial_log as tl
from gz_scene import GzScene, load_config
from phase2_pick_place_test import VALIDITY_SERVICE, StateRecorder, count_invalid

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase4_bin_picking.yaml'
SCENE_FIELDS = ('run_id', 'trial_id', 'git_commit', 'seed', 'config', 'strategy', 'scene',
                'n_objects', 'deposited', 'completion', 'end_reason', 'attempts', 'total_s')


def read_states(ctx, objects):
    """Ground-truth state of the scene's objects, from two readings a moment apart."""
    names = [item['name'] for item in objects]
    if not names:
        return {}
    ev_cfg = ctx['cfg']['evaluation']
    first = ctx['scene'].model_poses(names)
    common.spin_for(ctx['node'], ev_cfg['rest_interval_s'])
    second = ctx['scene'].model_poses(names)
    return be.object_states(objects, first, second, ctx['cell'], ctx['catalog'], ev_cfg)


def perception_error(target, name, before, cell):
    """How far the cell's idea of the object it aimed at is from ground truth. A tumbled
    object is not in the catalog's rest state, so `unknown` is the right class for it."""
    if name is None:
        return None
    origin, truth = so.bin_origin(cell, 'input'), before[name]
    expected = truth['type'] if truth['rest'] == so.CATALOG_REST else 'unknown'
    return {'class_ok': target['class_id'] == expected, 'truth_type': truth['type'],
            'truth_rest': truth['rest'],
            'xy_error_m': math.hypot(truth['xyz'][0] - origin[0] - target['x'],
                                     truth['xyz'][1] - origin[1] - target['y'])}


def attempt_record(ctx, trial_id, attempt, before, after, states):
    ev_cfg, cell = ctx['cfg']['evaluation'], ctx['cell']
    manipulation, target = attempt['manipulation'], attempt['target']
    name = be.match_target(target['x'], target['y'], before, cell, ev_cfg['match_distance_m'])
    evaluation = be.evaluate_attempt(name, before, after, ev_cfg)
    invalid = count_invalid(ctx['node'], ctx['validity'], states)
    success, reason, detection = be.attempt_result(
        attempt['success'], attempt['failure_reason'], evaluation, invalid)
    return {
        **ctx['meta'],
        'trial_id': trial_id,
        'attempt': attempt['attempt'],
        'object_category': evaluation['target_type'] or target['class_id'],
        'objects_seen': attempt['objects'],
        'ranking': attempt['strategy'],
        'selected_grasp': {**target, 'frame': c.INPUT_BIN_FRAME, 'tcp': manipulation['grasp']},
        'candidates': attempt['candidates'],
        'candidates_total': attempt['candidates_total'],
        'candidates_rejected': attempt['candidates_rejected'],
        'place': attempt['place'],
        'planner_result': {s['stage']: s['moveit_error_code'] for s in manipulation['stages']
                           if 'moveit_error_code' in s},
        'execution_result': {
            'cell_success': attempt['success'],
            'cell_failure': attempt['failure_reason'],
            'manipulation_success': manipulation['success'],
            'manipulation_failure': manipulation['failure_reason'],
            'failed_stage': manipulation['failed_stage'],
            'recovered': manipulation['recovered'],
        },
        'verification': attempt['verification'],
        'perception_error': perception_error(target, name, before, cell),
        'evaluation': {**evaluation, 'states_checked': len(states), 'states_invalid': invalid},
        'detection': detection,
        'success': success,
        'failure_reason': reason,
        'perception_s': attempt['perception_s'],
        'selection_s': attempt['selection_s'],
        'planning_s': attempt['planning_s'],
        'execution_s': attempt['execution_s'],
        'total_s': attempt['total_s'],
        'stages': manipulation['stages'],
    }


def run_scene(ctx, trial_id, n_objects, log_path):
    """One scene from spawn to the cell's own end: (scene record, attempt records)."""
    cfg, cell, catalog, node = ctx['cfg'], ctx['cell'], ctx['catalog'], ctx['node']
    ev_cfg, picker = cfg['evaluation'], ctx['picker']
    start = time.monotonic()

    common.clear_objects(ctx['scene'], ctx['names'])
    picker.reset(seed=f"{ctx['meta']['seed']}:{trial_id}")
    objects = [] if n_objects == 0 else rs.sample_scene(
        ctx['meta']['seed'], trial_id, n_objects, catalog, cell, cfg['scene'])
    common.spawn_scene(ctx['scene'], objects, catalog, cell)
    common.spin_for(node, ev_cfg['settle_s'])
    _, resting = common.read_truth(ctx['scene'], objects, cell, ev_cfg, catalog)

    records, before = [], read_states(ctx, objects)
    if not resting:
        end_reason = 'spawn_failed'
    else:
        # the cell's own budgets end the scene; the cap only guards the harness
        for _ in range(4 * n_objects + 4):
            ctx['recorder'].start()
            result = picker.step()
            states = ctx['recorder'].stop()
            if result['done']:
                break
            common.spin_for(node, ev_cfg['settle_s'])
            after = read_states(ctx, objects)
            record = attempt_record(ctx, trial_id, result['attempt'], before, after, states)
            tl.append(log_path, record)
            records.append(record)
            before = after
            node.get_logger().info(
                f"scene {trial_id} attempt {record['attempt']} {record['object_category']} "
                f"{record['selected_grasp']['label']}: "
                f"{'ok' if record['success'] else record['failure_reason']}"
                f"{'' if record['detection'] == 'agree' else ' [' + record['detection'] + ']'}, "
                f"{record['total_s']:.1f} s, rejected {record['candidates_rejected']}, invalid "
                f"{record['evaluation']['states_invalid']}/{record['evaluation']['states_checked']}")
        end_reason = result['end_reason'] if result['done'] else 'harness_cap'

    outcome = be.scene_result(objects, before, end_reason, len(records))
    scene = {**ctx['meta'], 'trial_id': trial_id, 'scene': objects, **outcome,
             'final_states': before, 'total_s': time.monotonic() - start}
    node.get_logger().info(
        f"scene {trial_id}: {outcome['deposited']}/{outcome['n_objects']} deposited in "
        f"{outcome['attempts']} attempts, end {end_reason}")
    return scene, records


def validate_scene(record):
    missing = [name for name in SCENE_FIELDS if name not in record]
    if missing:
        raise ValueError(f'missing fields: {missing}')
    return record


def apply_strategy(cfg, pp_cfg, name):
    """(strategy name for the log, pick_place config) for a benchmark config. Phase 4 configs
    have one fixed strategy; phase 5 configs name several (D-019) and pick one."""
    if 'strategies' not in cfg:
        if name is not None:
            raise ValueError('this config has a single strategy')
        return cfg['strategy'], pp_cfg
    name = name or cfg['default_strategy']
    chosen = cfg['strategies'][name]
    pp_cfg = {**pp_cfg, 'selection': {**pp_cfg['selection'],
                                      'unknown_grasps': chosen['unknown_grasps']},
              'ranking': {**pp_cfg['ranking'], 'strategy': chosen['ranking'],
                          'model': chosen.get('model', pp_cfg['ranking']['model'])}}
    return name, pp_cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--set', default='eval', dest='which', help='scene set of the config')
    parser.add_argument('--strategy', help='strategy of a phase 5 config (default: its default)')
    parser.add_argument('--scenes', type=int, help='overrides the number of scenes (smoke runs)')
    parser.add_argument('--first', type=int, default=0, help='index of the first scene')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    chosen = cfg['sets'][args.which]
    seed, n_scenes = chosen['seed'], chosen['scenes']
    # filled scenes first, then the empty bins; a smoke run takes a slice of that schedule
    schedule = [(index, cfg['objects_per_scene']) for index in range(n_scenes)]
    schedule += [(n_scenes + index, 0) for index in range(chosen['empty_scenes'])]
    full = args.scenes is None and args.first == 0
    schedule = schedule[args.first:None if args.scenes is None else args.first + args.scenes]
    pp_cfg, catalog, gripper = load_configs(cfg['pick_place_config'])
    strategy, pp_cfg = apply_strategy(cfg, pp_cfg, args.strategy)
    _, cell = load_config()                  # installed openfactory_description, as the cell uses

    label = args.which if 'strategies' not in cfg else f'{args.which}_{strategy}'
    run_id = (f"{cfg.get('run_name', 'phase4_bin_picking')}_{label}_"
              f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{seed}")
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'strategy': strategy}
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)

    rclpy.init()
    node = Node('benchmark',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    attempts, scenes = [], []
    try:
        picker = BinPicking(node, pp_cfg, catalog, gripper, cell)
        picker.wait_ready()
        validity = node.create_client(GetStateValidity, VALIDITY_SERVICE)
        if not validity.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{VALIDITY_SERVICE} not available')
        ctx = {'cfg': cfg, 'cell': cell, 'catalog': catalog, 'meta': meta, 'node': node,
               'scene': GzScene(), 'picker': picker, 'validity': validity,
               'names': common.object_names(catalog, cfg['objects_per_scene']),
               'recorder': StateRecorder(node, cfg['evaluation']['record_period_s'])}
        for trial_id, n_objects in schedule:
            scene, records = run_scene(ctx, trial_id, n_objects, out_dir / 'attempts.jsonl')
            tl.append(out_dir / 'scenes.jsonl', scene, check=validate_scene)
            scenes.append(scene)
            attempts += records
        common.clear_objects(ctx['scene'], ctx['names'])
        picker.reset()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = {**meta, 'set': args.which, 'scenes_run': len(scenes), 'full_set': full,
               'ranking': pp_cfg['ranking']['strategy'],
               'unknown_grasps': pp_cfg['selection'].get('unknown_grasps', False),
               **be.summarize(attempts, scenes, cfg['thresholds'])}
    summary['passed'] = summary['passed'] and full
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
