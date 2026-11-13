#!/usr/bin/env python3
"""Phase 2 closing test: deterministic pick and place of each catalog object at a known pose.

For every attempt the harness spawns the object at its known pose in the input bin (between
attempts, never during one), checks that it rests there, runs one openfactory_manipulation
cycle, then evaluates with simulator ground truth: object resting upright in the output bin,
place error, and every executed arm state re-checked by MoveIt for collisions. Ground truth is
read from Gazebo by this harness only (D-009). Logs: ~/openfactory_data/trials/<run_id>/ in the
VM (copy to the repo's data/ with scp from the Mac).

Run in the VM with the cell launched (openfactory_bringup cell.launch.py, RViz not needed):
    python3 scripts/phase2_pick_place_test.py [--config FILE] [--seed N] [--trials N]
"""

import argparse
import json
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from openfactory_manipulation import constants as c
from openfactory_manipulation.pick_place import PickPlace
from openfactory_manipulation.pick_place_once import load_configs
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

import pick_place_eval as ev
import scene_objects as so
import trial_log as tl
from gz_scene import GzScene, load_config

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase2_pick_place.yaml'
VALIDITY_SERVICE = '/check_state_validity'
STRATEGY = 'known_pose_top_down'
RECORDED_JOINTS = (*c.ARM_JOINTS, 'left_finger_joint', 'right_finger_joint')


class StateRecorder:
    """Samples executed joint states from /joint_states on the node the cycle spins."""

    def __init__(self, node, period_s, joints=RECORDED_JOINTS):
        self.period_s = period_s
        self.joints = tuple(joints)
        self.states, self.active, self.last = [], False, 0.0
        self.sub = node.create_subscription(JointState, c.JOINT_STATES_TOPIC, self._on_state, 50)

    def _on_state(self, msg):
        if not self.active:
            return
        now = time.monotonic()
        if now - self.last < self.period_s:
            return
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joints):
            self.states.append([positions[name] for name in self.joints])
            self.last = now

    def start(self):
        self.states, self.active, self.last = [], True, 0.0

    def stop(self):
        self.active = False
        return self.states


def count_invalid(node, client, states, joints=RECORDED_JOINTS, groups=(c.ARM_GROUP,),
                  contacts=None):
    """Recorded states that MoveIt finds in collision.

    A state is checked once per planning group of `groups` (ring of N arms: one per arm) and
    is invalid if any of them is: a group's check covers the pairs that involve its links, so
    an arm against everything else, the other arms included. The whole robot is not checked
    as one: the fixtures are robot links, and a planning-scene object next to a bin wall
    would then count as a collision of the bin. `contacts`, if given, is a set that collects
    the pairs of bodies in contact.
    """
    invalid = 0
    for state in states:
        robot_state = RobotState(joint_state=JointState(name=list(joints), position=state))
        valid = True
        for group in groups:
            future = client.call_async(
                GetStateValidity.Request(group_name=group, robot_state=robot_state))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done():
                valid = False
                if contacts is not None:
                    contacts.add(('no_answer', group))
            elif not future.result().valid:
                valid = False
                if contacts is not None:
                    contacts.update(tuple(sorted((contact.contact_body_1,
                                                  contact.contact_body_2)))
                                    for contact in future.result().contacts)
        invalid += not valid
    return invalid


def clear_objects(scene, catalog):
    for name in catalog['objects']:
        try:
            scene.remove(name)
        except RuntimeError:
            pass                               # not in the world


def run_attempt(ctx, trial_id, object_type):
    cfg, cell, catalog = ctx['cfg'], ctx['cell'], ctx['catalog']
    spec, known = catalog['objects'][object_type], cfg['objects'][object_type]
    ev_cfg = ctx['cfg']['evaluation']
    scene, runner, moveit = ctx['scene'], ctx['runner'], ctx['runner'].moveit

    clear_objects(scene, catalog)
    xyz, yaw = so.spawn_pose(cell, 'input', spec, known['x'], known['y'], known['yaw'])
    scene.spawn(object_type, so.model_sdf(object_type, spec, catalog['friction']), xyz,
                (0.0, 0.0, yaw))
    moveit.spin_for(ev_cfg['settle_s'])
    resting = scene.model_poses([object_type]).get(object_type)
    expected = (xyz[0], xyz[1], xyz[2] - 0.002)  # spawned 2 mm above its resting height
    spawn_error = None if resting is None else ev.distance(resting[0], expected)

    if spawn_error is None or spawn_error > ev_cfg['spawn_tolerance_m']:
        manipulation = {'success': False, 'failure_reason': 'spawn_failed', 'failed_stage': None,
                        'recovered': None, 'grasp': None, 'stages': [], 'planning_s': 0.0,
                        'execution_s': 0.0, 'total_s': 0.0}
        states = []
    else:
        ctx['recorder'].start()
        manipulation = runner.run(object_type, object_type, known['x'], known['y'], known['yaw'])
        states = ctx['recorder'].stop()

    moveit.spin_for(ev_cfg['settle_s'])
    final = scene.model_poses([object_type]).get(object_type)
    place_xy = runner.cfg['place']['xy']
    evaluation = ev.evaluate_place(final, cell, spec, place_xy, ev_cfg)
    invalid = count_invalid(ctx['node'], ctx['validity'], states)
    success, reason = ev.outcome(manipulation, evaluation, invalid)

    planner = {s['stage']: s['moveit_error_code'] for s in manipulation['stages']
               if 'moveit_error_code' in s}
    return {
        **ctx['meta'],
        'trial_id': trial_id,
        'attempt': 0,
        'object_category': object_type,
        'known_pose': {'frame': c.INPUT_BIN_FRAME, **known},
        'selected_grasp': manipulation['grasp'],
        'candidates_total': 1,                  # one deterministic grasp, no candidate search
        'candidates_rejected': {},
        'planner_result': planner,
        'execution_result': {
            'manipulation_success': manipulation['success'],
            'manipulation_failure': manipulation['failure_reason'],
            'failed_stage': manipulation['failed_stage'],
            'recovered': manipulation['recovered'],
        },
        'evaluation': {**evaluation, 'spawn_error_m': spawn_error,
                       'states_checked': len(states), 'states_invalid': invalid},
        'success': success,
        'failure_reason': reason,
        'perception_s': None,                   # no perception in phase 2
        'planning_s': manipulation['planning_s'],
        'execution_s': manipulation['execution_s'],
        'total_s': manipulation['total_s'],
        'stages': manipulation['stages'],
    }


def summarize(records, cfg):
    summary = tl.summarize(records)
    errors = [r['evaluation']['place_error_m'] for r in records
              if r['success'] and r['evaluation']['place_error_m'] is not None]
    by_type = summary['by_object_category']
    n = cfg['trials_per_object']
    summary['place_error_m'] = (
        {'mean': statistics.fmean(errors), 'max': max(errors)} if errors else None)
    summary['states_checked'] = sum(r['evaluation']['states_checked'] for r in records)
    summary['states_invalid'] = sum(r['evaluation']['states_invalid'] for r in records)
    summary['passed'] = (
        all(by_type.get(name, {}).get('successes') == n == by_type[name]['attempts']
            for name in cfg['objects'])
        and summary['states_invalid'] == 0)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--seed', type=int, help='overrides the config seed')
    parser.add_argument('--trials', type=int, help='overrides trials_per_object (smoke runs)')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    if args.trials is not None:
        cfg['trials_per_object'] = args.trials
    seed = cfg['seed'] if args.seed is None else args.seed
    pp_cfg, catalog, gripper = load_configs(cfg['pick_place_config'])
    _, cell = load_config()                  # installed openfactory_description, as the cell uses

    run_id = f"phase2_pick_place_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{seed}"
    meta = {'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': seed,
            'config': tl.config_ref(args.config), 'strategy': STRATEGY}
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)
    log_path = out_dir / 'attempts.jsonl'

    rclpy.init()
    node = Node('phase2_pick_place_test',
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
               'recorder': StateRecorder(node, cfg['evaluation']['record_period_s'])}
        schedule = [name for _ in range(cfg['trials_per_object']) for name in cfg['objects']]
        for trial_id, object_type in enumerate(schedule):
            record = run_attempt(ctx, trial_id, object_type)
            tl.append(log_path, record)
            records.append(record)
            ev_ = record['evaluation']
            node.get_logger().info(
                f"trial {trial_id} {object_type}: "
                f"{'ok' if record['success'] else record['failure_reason']}, "
                f"{record['total_s']:.1f} s, place error "
                f"{ev_['place_error_m'] if ev_['place_error_m'] is None else round(ev_['place_error_m'], 4)}"
                f", invalid states {ev_['states_invalid']}/{ev_['states_checked']}")
        clear_objects(ctx['scene'], catalog)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = {**meta, 'trials_per_object': cfg['trials_per_object'], **summarize(records, cfg)}
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
