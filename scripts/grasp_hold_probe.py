#!/usr/bin/env python3
"""Phase 4 diagnostic: how much does a held object turn between the pads? (D-015)

Each attempt spawns one catalog object at a known pose and yaw, runs the unchanged known-pose
pick-and-place cycle (D-010) and meanwhile samples the object's ground-truth pose a few times
per second. The rotation about the finger axis while the object is in the air, and the face
it rests on afterwards, are compared between variants of the pick-and-place parameters.

Ground truth is read by this harness only, for measurement: manipulation gets the known pose
and nothing else. Logs: ~/openfactory_data/trials/<run_id>/.

Run in the VM with the cell launched (camera and perception are not needed):
    python3 scripts/grasp_hold_probe.py [--config FILE] [--variants NAME ...] [--repeats N]
"""

import argparse
import json
import math
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from openfactory_manipulation.pick_place import PickPlace
from openfactory_manipulation.pick_place_once import load_configs
from rclpy.node import Node
from rclpy.parameter import Parameter

import hold_probe_eval as hp
import phase3_common as common
import pick_place_eval as ev
import scene_objects as so
import trial_log as tl
from gz_scene import GzScene, load_config

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase4_hold_probe.yaml'
STRATEGY = 'known_pose_top_down'
# rotational symmetry about the object x axis, the axis the fingers close along
ROLL_PERIOD = {'triangular_prism': 2 * math.pi / 3, 'box': math.pi / 2}


class PoseSampler:
    """Ground-truth pose of one model, sampled from a thread while the cycle runs.

    One reading at a time through GzScene.model_poses: a continuous subscription to the 1 kHz
    pose topic would starve this process.
    """

    def __init__(self, period_s):
        self.period_s = period_s
        self.scene = GzScene()
        self.samples, self.name = [], None
        self.halt, self.thread = threading.Event(), None

    def read(self):
        try:
            pose = self.scene.model_poses([self.name]).get(self.name)
        except RuntimeError:
            return
        if pose is not None:
            self.samples.append((time.monotonic(), pose[0], pose[1]))

    def _loop(self):
        while not self.halt.is_set():
            self.read()
            self.halt.wait(self.period_s)

    def start(self, name):
        self.name, self.samples = name, []
        self.halt.clear()
        self.read()                              # the object at rest, before the cycle
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.halt.set()
        self.thread.join()
        self.read()                              # the object at rest, after the cycle
        return self.samples


def run_attempt(ctx, trial_id, variant, yaw_deg):
    cfg, cell, catalog = ctx['cfg'], ctx['cell'], ctx['catalog']
    ev_cfg, scene, runner, node = cfg['evaluation'], ctx['scene'], ctx['runner'], ctx['node']
    name = cfg['object']
    spec = catalog['objects'][name]
    x, y = cfg['position']
    yaw = math.radians(yaw_deg)

    common.clear_objects(scene, ctx['names'])
    xyz, _ = so.spawn_pose(cell, 'input', spec, x, y, yaw)
    scene.spawn(name, so.model_sdf(name, spec, catalog['friction']), xyz, (0.0, 0.0, yaw))
    common.spin_for(node, ev_cfg['settle_s'])

    ctx['sampler'].start(name)
    manipulation = runner.run(name, name, x, y, yaw)
    common.spin_for(node, ev_cfg['settle_s'])
    samples = ctx['sampler'].stop()

    hold = hp.summarize_hold(samples, ROLL_PERIOD[spec['shape']], ev_cfg['air_margin_m'])
    final = samples[-1][1:] if samples else None
    evaluation = ev.evaluate_place(final, cell, spec, runner.cfg['place']['xy'], ev_cfg)
    deposited = final is not None and ev.in_bin(final[0], cell, 'output')
    if not manipulation['success']:
        reason = manipulation['failure_reason']
    else:
        reason = None if deposited else 'place_failed'

    start = samples[0][0] if samples else 0.0
    quat0 = samples[0][2] if samples else None
    return {
        **ctx['meta'],
        'trial_id': trial_id,
        'attempt': 0,
        'object_category': name,
        'variant': variant,
        'yaw_deg': yaw_deg,
        'selected_grasp': manipulation['grasp'],
        'candidates_total': 1,
        'candidates_rejected': {},
        'planner_result': {s['stage']: s['moveit_error_code'] for s in manipulation['stages']
                           if 'moveit_error_code' in s},
        'execution_result': {
            'manipulation_success': manipulation['success'],
            'manipulation_failure': manipulation['failure_reason'],
            'failed_stage': manipulation['failed_stage'],
            'recovered': manipulation['recovered'],
        },
        'evaluation': {**evaluation, 'deposited': deposited},
        'hold': hold,
        # [seconds since the first sample, y, z, roll about the finger axis]
        'trace': [[round(t - start, 2), round(p[1], 4), round(p[2], 4),
                   round(hp.roll_about_x(quat0, q), 4)] for t, p, q in samples],
        'success': reason is None,
        'failure_reason': reason,
        'perception_s': None,
        'planning_s': manipulation['planning_s'],
        'execution_s': manipulation['execution_s'],
        'total_s': manipulation['total_s'],
        'stages': manipulation['stages'],
    }


def summarize_variant(records, warning_rad):
    rolls = [r['hold']['max_roll_held_rad'] for r in records
             if r['hold'] and r['hold']['max_roll_held_rad'] is not None]
    return {
        'attempts': len(records),
        'deposited': sum(r['evaluation']['deposited'] for r in records),
        'resting_upright': sum(r['evaluation']['in_output_bin'] for r in records),
        'manipulation_failures': tl.summarize_group(records)['failures_by_reason'],
        'face_changes': sum(bool(r['hold'] and r['hold']['face_changed']) for r in records),
        'rolls_above_warning': sum(roll > warning_rad for roll in rolls),
        'max_roll_held_deg': None if not rolls else {
            'median': math.degrees(statistics.median(rolls)),
            'max': math.degrees(max(rolls)), 'n': len(rolls)},
        'worst_yaws_deg': [r['yaw_deg'] for r in sorted(
            (r for r in records if r['hold'] and r['hold']['max_roll_held_rad'] is not None),
            key=lambda r: -r['hold']['max_roll_held_rad'])[:3]],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--variants', nargs='+', help='subset of the config variants')
    parser.add_argument('--repeats', type=int, help='overrides the config repeats')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    repeats = cfg['repeats'] if args.repeats is None else args.repeats
    variants = args.variants or list(cfg['variants'])
    unknown = [name for name in variants if name not in cfg['variants']]
    if unknown:
        parser.error(f'unknown variants {unknown}; the config has {list(cfg["variants"])}')
    base_cfg, catalog, gripper = load_configs(cfg['pick_place_config'])
    _, cell = load_config()

    run_id = f"phase4_hold_probe_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{cfg['seed']}"
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': cfg['seed'],
            'config': tl.config_ref(args.config), 'strategy': STRATEGY}
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)
    log_path = out_dir / 'attempts.jsonl'

    rclpy.init()
    node = Node('grasp_hold_probe',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    records = []
    try:
        # the runner and its MoveIt interface share this dict: a variant rewrites it in place
        live_cfg = dict(base_cfg)
        runner = PickPlace(node, live_cfg, catalog, gripper)
        runner.wait_ready()
        ctx = {'cfg': cfg, 'cell': cell, 'catalog': catalog, 'meta': meta, 'node': node,
               'scene': GzScene(), 'runner': runner, 'names': common.object_names(catalog),
               'sampler': PoseSampler(cfg['evaluation']['sample_period_s'])}
        trial_id = 0
        for variant in variants:
            live_cfg.clear()
            live_cfg.update(hp.deep_merge(base_cfg, cfg['variants'][variant] or {}))
            for _ in range(repeats):
                for yaw_deg in cfg['yaws_deg']:
                    record = run_attempt(ctx, trial_id, variant, yaw_deg)
                    tl.append(log_path, record)
                    records.append(record)
                    trial_id += 1
                    hold = record['hold'] or {}
                    roll = hold.get('max_roll_held_rad')
                    node.get_logger().info(
                        f"{variant} yaw {yaw_deg}: "
                        f"{'ok' if record['success'] else record['failure_reason']}, max roll "
                        f"{'n/a' if roll is None else f'{math.degrees(roll):.1f} deg'}, "
                        f"face {hold.get('final_face')}")
        common.clear_objects(ctx['scene'], ctx['names'])
    finally:
        node.destroy_node()
        rclpy.shutdown()

    warning = cfg['evaluation']['roll_warning_rad']
    by_variant = {name: summarize_variant([r for r in records if r['variant'] == name], warning)
                  for name in variants}
    limit = cfg['thresholds']['max_face_changes']
    summary = {**meta, 'object': cfg['object'], 'repeats': repeats, 'by_variant': by_variant,
               'variants_meeting_goal': [name for name, s in by_variant.items()
                                         if s['face_changes'] <= limit
                                         and s['deposited'] == s['attempts']]}
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
