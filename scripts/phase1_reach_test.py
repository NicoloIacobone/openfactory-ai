#!/usr/bin/env python3
"""Phase 1 closing test: seeded joint goals, planned by MoveIt and executed in Gazebo.

Each goal is sampled from config ranges and accepted only if MoveIt reports it collision-free
and tool0 clears the table. It is then planned and executed through move_group and the
joint trajectory controller. States executed in Gazebo are recorded from /joint_states and
re-checked by MoveIt afterwards. Logs go to data/trials/<run_id>/.

Run in the VM with the cell launched (openfactory_bringup cell.launch.py):
    python3 scripts/phase1_reach_test.py [--config FILE] [--seed N]
"""

import argparse
import json
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

import reach_test_core as core

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase1_reach.yaml'
JOINT_STATES_TOPIC = '/joint_states'
VALIDITY_SERVICE = '/check_state_validity'
FK_SERVICE = '/compute_fk'
MOVE_ACTION = '/move_action'
SERVICE_TIMEOUT_S = 10.0


class ReachTest(Node):

    def __init__(self, cfg):
        super().__init__(
            'phase1_reach_test',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cfg = cfg
        self.joints = cfg['joints']
        self.latest = None
        self.recording = None
        self.last_record = 0.0
        self.joint_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.on_joint_state, 10)
        self.validity_client = self.create_client(GetStateValidity, VALIDITY_SERVICE)
        self.fk_client = self.create_client(GetPositionFK, FK_SERVICE)
        self.move_client = ActionClient(self, MoveGroup, MOVE_ACTION)

    def on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joints):
            return
        self.latest = [positions[name] for name in self.joints]
        now = time.monotonic()
        if self.recording is not None and now - self.last_record >= self.cfg['record_period_s']:
            self.recording.append(self.latest)
            self.last_record = now

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f'no response within {timeout} s')
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self):
        for client, name in ((self.validity_client, VALIDITY_SERVICE), (self.fk_client, FK_SERVICE)):
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
                raise RuntimeError(f'{name} not available: is the cell launched?')
        if not self.move_client.wait_for_server(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f'{MOVE_ACTION} not available')
        end = time.monotonic() + SERVICE_TIMEOUT_S
        while self.latest is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError(f'no {JOINT_STATES_TOPIC}')

    def robot_state(self, positions):
        return RobotState(joint_state=JointState(name=self.joints, position=list(positions)))

    def is_valid(self, positions):
        request = GetStateValidity.Request(
            group_name=self.cfg['group'], robot_state=self.robot_state(positions))
        return self.wait(self.validity_client.call_async(request), SERVICE_TIMEOUT_S).valid

    def tool_z(self, positions):
        request = GetPositionFK.Request(
            fk_link_names=[self.cfg['tip_link']], robot_state=self.robot_state(positions))
        request.header.frame_id = self.cfg['frame']
        response = self.wait(self.fk_client.call_async(request), SERVICE_TIMEOUT_S)
        if response.error_code.val != core.MOVEIT_SUCCESS:
            return None
        return response.pose_stamped[0].pose.position.z

    def sample_goal(self, rng, min_tool_z):
        for samples in range(1, self.cfg['max_samples_per_goal'] + 1):
            goal = core.sample_joint_goal(rng, self.joints, self.cfg['joint_ranges'])
            if not self.is_valid(goal):
                continue
            z = self.tool_z(goal)
            if z is not None and z >= min_tool_z:
                return goal, z, samples
        return None, None, self.cfg['max_samples_per_goal']

    def move_goal(self, goal):
        cfg = self.cfg
        msg = MoveGroup.Goal()
        request = msg.request
        request.group_name = cfg['group']
        request.pipeline_id = cfg['pipeline']
        request.num_planning_attempts = cfg['planning_attempts']
        request.allowed_planning_time = cfg['planning_time_s']
        request.max_velocity_scaling_factor = cfg['velocity_scaling']
        request.max_acceleration_scaling_factor = cfg['acceleration_scaling']
        request.start_state.is_diff = True
        request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(
                joint_name=name, position=position,
                tolerance_above=cfg['goal_joint_tolerance'],
                tolerance_below=cfg['goal_joint_tolerance'], weight=1.0)
            for name, position in zip(self.joints, goal)])]
        msg.planning_options.plan_only = False
        return msg

    def run_goal(self, rng, min_tool_z):
        goal, tool_z, samples = self.sample_goal(rng, min_tool_z)
        record = {
            'goal': goal, 'goal_tool_z': tool_z, 'goal_samples': samples,
            'moveit_error_code': None, 'planning_s': None, 'execution_s': None, 'total_s': None,
            'final': None, 'final_max_abs_error': None,
            'executed_states_checked': 0, 'executed_states_invalid': 0,
        }
        if goal is None:
            return {**record, 'success': False, 'failure_reason': 'no_valid_goal'}

        start = time.monotonic()
        self.recording, self.last_record = [], 0.0
        try:
            handle = self.wait(self.move_client.send_goal_async(self.move_goal(goal)),
                               SERVICE_TIMEOUT_S)
            if not handle.accepted:
                return {**record, 'success': False, 'failure_reason': 'goal_rejected'}
            result = self.wait(handle.get_result_async(), self.cfg['action_timeout_s']).result
        except TimeoutError:
            return {**record, 'success': False, 'failure_reason': 'timeout'}
        finally:
            executed, self.recording = self.recording, None
        total = time.monotonic() - start

        self.spin_for(self.cfg['settle_s'])
        final = list(self.latest)
        error = core.max_abs_error(goal, final)
        invalid = sum(not self.is_valid(state) for state in executed)
        code = result.error_code.val
        reason = core.failure_reason(code, error, self.cfg['final_error_tolerance'], invalid)
        record.update({
            'moveit_error_code': code,
            'planning_s': result.planning_time,
            'execution_s': total - result.planning_time,
            'total_s': total,
            'final': final,
            'final_max_abs_error': error,
            'executed_states_checked': len(executed),
            'executed_states_invalid': invalid,
        })
        return {**record, 'success': reason is None, 'failure_reason': reason}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--seed', type=int, help='overrides the config seed')
    parser.add_argument('--out', type=Path, default=REPO / 'data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    seed = cfg['seed'] if args.seed is None else args.seed
    cell_file = Path(get_package_share_directory('openfactory_description')) / 'config' / 'cell.yaml'
    cell = yaml.safe_load(cell_file.read_text())
    min_tool_z = cell['table']['xyz'][2] + cell['table']['size'][2] + cfg['min_tool_height']

    run_id = f"phase1_reach_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_s{seed}"
    meta = {
        'run_id': run_id,
        'git_commit': core.git_commit(REPO),
        'seed': seed,
        'config': {'path': str(args.config), 'sha256_12': core.config_hash(args.config)},
        'strategy': 'moveit_joint_goal',
    }
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)

    rclpy.init()
    node = ReachTest(cfg)
    try:
        node.wait_ready()
        rng = random.Random(seed)
        records = []
        with open(out_dir / 'attempts.jsonl', 'w') as log:
            for trial_id in range(cfg['goals']):
                record = {**meta, 'trial_id': trial_id, 'attempt': 0, **node.run_goal(rng, min_tool_z)}
                log.write(json.dumps(record) + '\n')
                log.flush()
                records.append(record)
                node.get_logger().info(
                    f"goal {trial_id}: {'ok' if record['success'] else record['failure_reason']}"
                    f", error {record['final_max_abs_error']}")
        summary = {**meta, **core.summarize(records, cfg['goals'])}
        (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
