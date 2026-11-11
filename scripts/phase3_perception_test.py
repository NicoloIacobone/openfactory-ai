#!/usr/bin/env python3
"""Phase 3 closing test, part B: perception accuracy against simulator ground truth.

For every seeded scene the harness spawns 0-3 separate catalog objects in the input bin, waits
for them to rest, reads their ground-truth poses from Gazebo, takes the first perception output
stamped after that, and scores it: detections matched to objects, class, position and yaw
errors, false positives. Ground truth is read by this harness only and never reaches the
perception node (rule 5). The robot does not move. Logs: ~/openfactory_data/trials/<run_id>/.

Run in the VM with the cell launched (camera and perception on, RViz not needed):
    python3 scripts/phase3_perception_test.py [--set eval|dev] [--config FILE] [--scenes N]
"""

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from openfactory_manipulation.perception_client import PerceptionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

import perception_eval as pe
import phase3_common as common
import randomize_scene as rs
import trial_log as tl
from gz_scene import GzScene, load_config

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase3_perception.yaml'


def schedule(cfg, scene_set, n_scenes=None):
    """[(scene index, objects)]: `scenes` scenes cycling through objects_cycle, then the empty
    ones."""
    spec = cfg['sets'][scene_set]
    count = spec['scenes'] if n_scenes is None else n_scenes
    cycle = cfg['objects_cycle']
    plan = [(index, cycle[index % len(cycle)]) for index in range(count)]
    return plan + [(count + index, 0) for index in range(spec['empty_scenes'])]


def run_scene(ctx, index, n_objects):
    cfg, catalog, cell, node = ctx['cfg'], ctx['catalog'], ctx['cell'], ctx['node']
    ev_cfg = cfg['evaluation']
    common.clear_objects(ctx['scene'], ctx['names'])
    objects = rs.sample_scene(ctx['meta']['seed'], index, n_objects, catalog, cell, cfg['scene'])
    common.spawn_scene(ctx['scene'], objects, catalog, cell)
    common.spin_for(node, ev_cfg['settle_s'])
    truth, resting = common.read_truth(ctx['scene'], objects, cell, ev_cfg)

    start = time.monotonic()
    view = ctx['client'].wait_fresh(node.get_clock().now(), ev_cfg['view_timeout_s'])
    wait_s = time.monotonic() - start
    if view is None:
        detections, stamp, latency = [], None, None
    else:
        hypotheses, stamp = view
        detections = common.hypotheses_in_bin(hypotheses, cell)
        latency = max(0.0, node.get_clock().now().nanoseconds * 1e-9 - stamp)

    result = pe.score_scene(truth, detections, catalog, ev_cfg['match_distance_m'])
    if not resting:
        reason = 'spawn_failed'
    elif view is None:
        reason = 'no_view'
    else:
        reason = pe.scene_failure(result)
    return {
        **ctx['meta'], 'trial_id': index, 'scene': objects, 'truth': truth,
        'detections': detections, **result, 'success': reason is None, 'failure_reason': reason,
        'perception_s': latency, 'wait_s': wait_s, 'stamp': stamp,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--set', dest='scene_set', default='eval', choices=['eval', 'dev'])
    parser.add_argument('--scenes', type=int, help='overrides the number of scenes (smoke runs)')
    parser.add_argument('--label', default='', help='suffix of the run id, e.g. ideal')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    catalog, cell = load_config()
    seed = cfg['sets'][args.scene_set]['seed']
    label = f'_{args.label}' if args.label else ''
    run_id = (f"phase3_perception_{args.scene_set}{label}_"
              f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{seed}")
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'strategy': cfg['strategy']}
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)

    rclpy.init()
    node = Node('phase3_perception_test',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    records = []
    try:
        ctx = {'cfg': cfg, 'catalog': catalog, 'cell': cell, 'node': node, 'meta': meta,
               'scene': GzScene(), 'client': PerceptionClient(node),
               'names': common.object_names(catalog)}
        for index, n_objects in schedule(cfg, args.scene_set, args.scenes):
            record = run_scene(ctx, index, n_objects)
            tl.append(out_dir / 'scenes.jsonl', record, tl.validate_perception)
            records.append(record)
            errors = [f"{m['type']}->{m['class_id']} {m['xy_error_m'] * 1000:.1f} mm"
                      for m in record['matches']]
            node.get_logger().info(
                f"scene {index} ({n_objects} objects): "
                f"{'ok' if record['success'] else record['failure_reason']}; {', '.join(errors)}")
        common.clear_objects(ctx['scene'], ctx['names'])
    finally:
        node.destroy_node()
        rclpy.shutdown()

    scored = [r for r in records if r['failure_reason'] not in ('spawn_failed', 'no_view')]
    summary = {**meta, 'set': args.scene_set, 'label': args.label,
               'scenes_run': len(records), 'scenes_ok': sum(r['success'] for r in records),
               'scenes_not_scored': len(records) - len(scored),
               'failures_by_reason': {
                   reason: sum(r['failure_reason'] == reason for r in records)
                   for reason in sorted({r['failure_reason'] for r in records} - {None})},
               **pe.summarize(scored, cfg['thresholds'])}
    for name, hits, total in (('recall', summary['matched'], summary['objects']),
                              ('precision', summary['matched'], summary['detections'])):
        summary[f'{name}_ci95'] = list(tl.wilson_interval(hits, total))
    latencies = [r['perception_s'] for r in records if r['perception_s'] is not None]
    summary['perception_s'] = ({'median': pe.percentile(latencies, 50),
                                'max': max(latencies)} if latencies else None)
    # a scene that could not be scored is a failed scene, not a skipped one
    summary['passed'] = summary['passed'] and len(scored) == len(records)
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
