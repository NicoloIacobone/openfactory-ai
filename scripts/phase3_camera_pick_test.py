#!/usr/bin/env python3
"""Phase 3 closing test, part C: pick and place of one object located by the camera.

For every attempt the harness spawns one catalog object at a seeded random pose in the input
bin (between attempts, never during one). The cell then looks at the bin with the arm at
ready, takes the most confident perception hypothesis as object type and pose, and runs the
same pick-and-place cycle as the phase 2 baseline with that pose instead of a known one. The
harness evaluates with simulator ground truth, which neither perception nor manipulation sees:
object resting upright in the output bin, perception error, executed arm states re-checked by
MoveIt. Logs: ~/openfactory_data/trials/<run_id>/.

Run in the VM with the cell launched (camera and perception on, RViz not needed):
    python3 scripts/phase3_camera_pick_test.py [--config FILE] [--seed N] [--trials N]
"""

import argparse
import json
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.srv import GetStateValidity
from openfactory_manipulation import constants as c
from openfactory_manipulation.camera_pick import perceive
from openfactory_manipulation.perception_client import PerceptionClient
from openfactory_manipulation.pick_place import PickPlace
from openfactory_manipulation.pick_place_once import load_configs
from rclpy.node import Node
from rclpy.parameter import Parameter

import perception_eval as pe
import phase3_common as common
import pick_place_eval as ev
import randomize_scene as rs
import trial_log as tl
from gz_scene import GzScene, load_config
from phase2_pick_place_test import VALIDITY_SERVICE, StateRecorder, count_invalid

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase3_camera_pick.yaml'
STRATEGY = 'camera_top_down'
NOT_RUN = {'success': False, 'failed_stage': None, 'recovered': None, 'grasp': None,
           'stages': [], 'planning_s': 0.0, 'execution_s': 0.0, 'total_s': 0.0}


def perception_error(seen, truth, catalog):
    """How far the selected hypothesis is from ground truth (evaluation only)."""
    if not seen['ok']:
        return None
    hypothesis = {'class_id': seen['object_type'], 'score': seen['selected']['score'],
                  'x': seen['x'], 'y': seen['y'], 'yaw': seen['yaw']}
    result = pe.score_scene([truth], [hypothesis], catalog, max_distance=float('inf'))
    match = result['matches'][0]
    return {key: match[key] for key in ('class_ok', 'score', 'xy_error_m', 'yaw_error_rad')}


def run_attempt(ctx, trial_id, object_type):
    cfg, cell, catalog = ctx['cfg'], ctx['cell'], ctx['catalog']
    ev_cfg, scene, runner, node = cfg['evaluation'], ctx['scene'], ctx['runner'], ctx['node']
    spec = catalog['objects'][object_type]

    common.clear_objects(scene, ctx['names'])
    objects = rs.sample_scene(ctx['meta']['seed'], trial_id, 1, catalog, cell, cfg['scene'],
                              types=[object_type])
    name = objects[0]['name']
    common.spawn_scene(scene, objects, catalog, cell)
    common.spin_for(node, ev_cfg['settle_s'])
    truth, resting = common.read_truth(scene, objects, cell, ev_cfg)

    seen, states = None, []
    if not resting:
        manipulation = {**NOT_RUN, 'failure_reason': 'spawn_failed'}
    else:
        ctx['recorder'].start()
        seen = perceive(node, runner, ctx['client'])
        if seen['ok']:
            manipulation = runner.run(seen['object_type'], seen['object_type'], seen['x'],
                                      seen['y'], seen['yaw'], source='perception')
        else:
            manipulation = {**NOT_RUN, 'failure_reason': seen['failure_reason']}
        states = ctx['recorder'].stop()

    common.spin_for(node, ev_cfg['settle_s'])
    final = scene.model_poses([name]).get(name)
    evaluation = ev.evaluate_place(final, cell, spec, runner.cfg['place']['xy'], ev_cfg)
    invalid = count_invalid(node, ctx['validity'], states)
    success, reason = ev.outcome(manipulation, evaluation, invalid)

    planner = {s['stage']: s['moveit_error_code'] for s in manipulation['stages']
               if 'moveit_error_code' in s}
    perception = None if seen is None else {
        'hypotheses': common.hypotheses_in_bin(seen['hypotheses'], cell),
        'selected': None if not seen['ok'] else {
            'class_id': seen['object_type'], 'score': seen['selected']['score'],
            'frame': c.INPUT_BIN_FRAME, 'x': seen['x'], 'y': seen['y'], 'yaw': seen['yaw']},
        'stamp': seen['stamp'],
        'error': perception_error(seen, truth[0], catalog),
    }
    return {
        **ctx['meta'],
        'trial_id': trial_id,
        'attempt': 0,
        'object_category': object_type,
        'scene': objects,
        'truth': truth,
        'perception': perception,
        'selected_grasp': manipulation['grasp'],
        'candidates_total': 0 if perception is None else len(perception['hypotheses']),
        'candidates_rejected': {},
        'planner_result': planner,
        'execution_result': {
            'manipulation_success': manipulation['success'],
            'manipulation_failure': manipulation['failure_reason'],
            'failed_stage': manipulation['failed_stage'],
            'recovered': manipulation['recovered'],
        },
        'evaluation': {**evaluation, 'states_checked': len(states), 'states_invalid': invalid},
        'success': success,
        'failure_reason': reason,
        'perception_s': None if seen is None else seen['perception_s'],
        'planning_s': manipulation['planning_s'],
        'execution_s': manipulation['execution_s'],
        'total_s': manipulation['total_s'] + ((seen or {}).get('perception_s') or 0.0),
        'stages': manipulation['stages'],
    }


def _spread(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {'mean': statistics.fmean(values), 'median': statistics.median(values),
            'max': max(values), 'n': len(values)}


def summarize(records, cfg):
    summary = tl.summarize(records)
    limits, by_type = cfg['thresholds'], summary['by_object_category']
    errors = [r['perception']['error'] for r in records
              if r['perception'] and r['perception']['error']]
    summary['perception_error'] = {
        'class_ok': sum(e['class_ok'] for e in errors), 'n': len(errors),
        'xy_error_m': _spread([e['xy_error_m'] for e in errors]),
        'yaw_error_rad': _spread([e['yaw_error_rad'] for e in errors]),
        'score': _spread([e['score'] for e in errors]),
    }
    summary['place_error_m'] = _spread(
        [r['evaluation']['place_error_m'] for r in records if r['success']])
    summary['states_checked'] = sum(r['evaluation']['states_checked'] for r in records)
    summary['states_invalid'] = sum(r['evaluation']['states_invalid'] for r in records)
    # an attempt the manipulation reported as done while ground truth says otherwise
    summary['undetected_failures'] = sum(
        r['execution_result']['manipulation_success'] and not r['evaluation']['in_output_bin']
        for r in records)
    full = len(records) == cfg['trials_per_object'] * len(cfg['objects'])
    summary['passed'] = (
        full
        and summary['overall']['successes'] >= limits['min_successes']
        and all(by_type.get(name, {}).get('successes', 0) >= limits['min_successes_per_object']
                for name in cfg['objects'])
        and summary['states_invalid'] <= limits['max_invalid_states'])
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--seed', type=int, help='overrides the config seed')
    parser.add_argument('--trials', type=int, help='overrides trials_per_object (smoke runs)')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    trials = cfg['trials_per_object'] if args.trials is None else args.trials
    seed = cfg['seed'] if args.seed is None else args.seed
    pp_cfg, catalog, gripper = load_configs(cfg['pick_place_config'])
    _, cell = load_config()                  # installed openfactory_description, as the cell uses

    run_id = f"phase3_camera_pick_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{seed}"
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'strategy': STRATEGY}
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)
    log_path = out_dir / 'attempts.jsonl'

    rclpy.init()
    node = Node('phase3_camera_pick_test',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    records = []
    try:
        runner = PickPlace(node, pp_cfg, catalog, gripper)
        runner.wait_ready()
        validity = node.create_client(GetStateValidity, VALIDITY_SERVICE)
        if not validity.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{VALIDITY_SERVICE} not available')
        ctx = {'cfg': cfg, 'cell': cell, 'catalog': catalog, 'meta': meta, 'node': node,
               'scene': GzScene(), 'runner': runner, 'validity': validity,
               'client': PerceptionClient(node), 'names': common.object_names(catalog),
               'recorder': StateRecorder(node, cfg['evaluation']['record_period_s'])}
        schedule = [name for _ in range(trials) for name in cfg['objects']]
        for trial_id, object_type in enumerate(schedule):
            record = run_attempt(ctx, trial_id, object_type)
            tl.append(log_path, record)
            records.append(record)
            error = (record['perception'] or {}).get('error')
            seen = ('no hypothesis' if not error else
                    f"{'class ok' if error['class_ok'] else 'WRONG CLASS'}, "
                    f"xy {error['xy_error_m'] * 1000:.1f} mm")
            node.get_logger().info(
                f"trial {trial_id} {object_type}: "
                f"{'ok' if record['success'] else record['failure_reason']}, "
                f"{record['total_s']:.1f} s, perception {seen}, invalid states "
                f"{record['evaluation']['states_invalid']}/{record['evaluation']['states_checked']}")
        common.clear_objects(ctx['scene'], ctx['names'])
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = {**meta, 'trials_per_object': trials, **summarize(records, cfg)}
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
