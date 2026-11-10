"""One deterministic pick-and-place cycle at a known object pose (phase 2 baseline).

The object pose is fixed infrastructure input (D-009): a catalog type and (x, y, yaw) in the
input_bin frame, given by the caller. Nothing here reads simulator ground truth.

Sequence (docs/agent-guides/manipulation.md): ready -> open -> scene object -> pre-grasp (free
plan) -> approach (line) -> pre-close -> close + contact check -> attach -> lift (line) + hold
check -> transfer (line) + hold check -> descend (line) -> release -> detach -> retreat (line) ->
ready (free plan). On a failure the cycle stops,
records the stage and reason, and tries to return to a usable state (open, clear the scene
object, back to ready).
"""

import time

from . import constants as c
from .core import pick_place as core
from .moveit_interface import ManipulationError, MoveItInterface


class StageFailed(Exception):

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class PickPlace:

    def __init__(self, node, config, catalog, gripper):
        self.cfg = config
        self.catalog = catalog
        self.gripper_cfg = gripper
        self.moveit = MoveItInterface(node, config)
        self.stages = []

    def wait_ready(self):
        self.moveit.wait_ready()

    # -- stage helpers --------------------------------------------------------------------

    def _record(self, name, reason=None, planning_s=0.0, execution_s=0.0, **extra):
        self.stages.append({'stage': name, 'ok': reason is None, 'reason': reason,
                            'planning_s': planning_s, 'execution_s': execution_s, **extra})
        if reason is not None:
            raise StageFailed(reason)

    def _tool0(self, tcp_pose):
        return core.tool0_from_tcp(tcp_pose, self.gripper_cfg['tcp_offset'])

    def _execute(self, name, code, trajectory, planning_s, **extra):
        reason = core.moveit_failure(code)
        if reason is not None:
            self._record(name, reason, planning_s, moveit_error_code=code, **extra)
        code, execution_s = self.moveit.execute(trajectory)
        if code != core.MOVEIT_SUCCESS:
            reason = 'timeout' if core.moveit_failure(code) == 'timeout' else 'execution_failed'
        self._record(name, reason, planning_s, execution_s, moveit_error_code=code, **extra)

    def _free_move(self, name, tcp_pose):
        code, trajectory, planning_s = self.moveit.plan_pose(self._tool0(tcp_pose))
        self._execute(name, code, trajectory, planning_s)

    def _joint_move(self, name, joints):
        code, trajectory, planning_s = self.moveit.plan_joints(joints)
        self._execute(name, code, trajectory, planning_s)

    def _line(self, name, tcp_pose):
        code, trajectory, fraction, planning_s = self.moveit.plan_cartesian(self._tool0(tcp_pose))
        self._execute(name, code, trajectory, planning_s, cartesian_fraction=fraction)

    def _gripper(self, name, position):
        result, seconds = self.moveit.gripper(position, self.cfg['grasp']['max_effort'])
        finger = self.moveit.finger_position()
        extra = {'finger_position': finger, 'stalled': result.stalled,
                 'reached_goal': result.reached_goal}
        return finger, extra, seconds

    def _check_fingers(self, name, check, plan, extra=None):
        finger = self.moveit.finger_position()
        reason = check(finger, plan, self.cfg['grasp']['width_tolerance'])
        self._record(name, reason, finger_position=finger, **(extra or {}))

    # -- cycle ----------------------------------------------------------------------------

    def run(self, object_type, object_id, x, y, yaw):
        """Pick `object_type` resting at (x, y, yaw) in input_bin, place it in output_bin."""
        self.stages = []
        spec = self.catalog['objects'][object_type]
        start = time.monotonic()
        failure, recovered = None, None
        try:
            in_bin = self.moveit.frame_pose(c.INPUT_BIN_FRAME)
            out_bin = self.moveit.frame_pose(c.OUTPUT_BIN_FRAME)
            params = {**self.cfg['grasp'], **self.cfg['place']}
            object_pose = core.object_pose_in_bin(in_bin, x, y, yaw, spec)
            plan = core.plan_pick_place(object_pose, out_bin, self.cfg['place']['xy'], spec,
                                        self.gripper_cfg, params)
            self._cycle(plan, spec, object_id)
        except StageFailed as error:
            failure = error.reason
        except (ManipulationError, ValueError) as error:
            failure = 'timeout' if isinstance(error, ManipulationError) else 'planning_failed'
            self.stages.append({'stage': 'error', 'ok': False, 'reason': failure,
                                'planning_s': 0.0, 'execution_s': 0.0, 'message': str(error)})
        if failure is not None:
            recovered = self._recover(object_id)
        return {
            'object_type': object_type,
            'object_id': object_id,
            'known_pose': {'frame': c.INPUT_BIN_FRAME, 'x': x, 'y': y, 'yaw': yaw},
            'success': failure is None,
            'failure_reason': failure,
            'failed_stage': next((s['stage'] for s in self.stages if not s['ok']), None),
            'recovered': recovered,
            'planning_s': sum(s['planning_s'] for s in self.stages),
            'execution_s': sum(s['execution_s'] for s in self.stages),
            'total_s': time.monotonic() - start,
            'stages': self.stages,
        }

    def _cycle(self, plan, spec, object_id):
        ready = self.cfg['ready_joints']
        self.moveit.remove_object(object_id)          # leftovers of an interrupted cycle

        self._joint_move('ready', ready)
        finger, extra, seconds = self._gripper('open', plan.open_position)
        self._record('open', None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)

        lifted = core.raised(plan.object_pose, self.cfg['scene']['object_clearance'])
        ok = self.moveit.add_object(object_id, spec, lifted)
        self._record('scene_add', None if ok else 'scene_failed')

        self._free_move('pre_grasp', plan.pre_grasp)
        self._line('approach', plan.grasp)

        finger, extra, seconds = self._gripper('preclose', plan.preclose_position)
        self._record('preclose', None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)
        finger, extra, seconds = self._gripper('close', plan.close_position)
        reason = core.check_grasp(finger, plan, self.cfg['grasp']['width_tolerance'])
        self._record('close', reason, execution_s=seconds, **extra)

        ok = self.moveit.attach_object(object_id)
        self._record('attach', None if ok else 'scene_failed')

        self._line('lift', plan.lift)
        self._check_fingers('hold_after_lift', core.check_hold, plan)
        # straight line at lift height with a constant, vertical tool: a free-space plan may tilt
        # the gripper, and a tilted parallel grasp lets the object pivot out between the pads
        self._line('transfer', plan.pre_place)
        self._check_fingers('hold_after_transfer', core.check_hold, plan)
        self._line('descend', plan.place)

        finger, extra, seconds = self._gripper('release', plan.open_position)
        self._record('release', None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)
        ok = self.moveit.remove_object(object_id)
        self._record('detach', None if ok else 'scene_failed')

        self._line('retreat', plan.retreat)
        self._joint_move('return', ready)

    def _recover(self, object_id):
        """Best effort back to a usable state: open, clear the scene object, rise straight up
        (the gripper may be inside a bin, where free-space planning grazes the walls), then
        go to ready.

        Returns {'ok': bool, <step>: outcome} so a failed recovery can be diagnosed.
        """
        steps = {}
        try:
            result, _ = self.moveit.gripper(self.gripper_cfg['finger']['stroke'],
                                            self.cfg['grasp']['max_effort'])
            steps['open'] = bool(result.reached_goal)
            steps['scene_clear'] = self.moveit.remove_object(object_id)
            tool0 = self.moveit.frame_pose(c.TOOL_LINK)
            up = core.raised(tool0, self.cfg['grasp']['lift_height'])
            code, trajectory, fraction, _ = self.moveit.plan_cartesian(up)
            steps['rise_fraction'] = fraction
            if code == core.MOVEIT_SUCCESS:
                steps['rise_execute_code'], _ = self.moveit.execute(trajectory)
            code, trajectory, _ = self.moveit.plan_joints(self.cfg['ready_joints'])
            steps['ready_plan_code'] = code
            if code == core.MOVEIT_SUCCESS:
                code, _ = self.moveit.execute(trajectory)
                steps['ready_execute_code'] = code
            steps['ok'] = code == core.MOVEIT_SUCCESS
        except ManipulationError as error:
            steps.update(ok=False, error=str(error))
        return steps
