"""One pick-and-place cycle of a catalog object resting in the input bin.

Two entry points share the cycle:
- `run`: the object is a catalog type at (x, y, yaw) in the input_bin frame, a known pose given
  by the caller as fixed infrastructure (phase 2, D-009) or one perception hypothesis (phase 3,
  camera_pick.perceive). The grasp is the single top-down grasp of D-010.
- `run_grasp`: the grasp was chosen among candidates by grasp_selector and the object is
  already in the planning scene with the other objects seen in the bin (phase 4, D-016).
Nothing here reads simulator state.

Sequence (docs/agent-guides/manipulation.md): ready -> open -> (scene object) -> pre-grasp ->
approach (line) -> (pre-close) -> close + contact check -> attach -> lift (line) + hold check ->
transfer (line) + hold check -> descend (line) -> release -> detach -> retreat (line) -> ready
(free plan). On a failure the cycle stops, records the stage and reason, and tries to return to
a usable state (open, clear the scene object, back to ready).
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
        self.place_index = None
        self.released = None

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
        opening = self.moveit.jaw_opening()
        extra = {'finger_position': self.moveit.finger_position(), 'jaw_opening': opening,
                 'stalled': result.stalled, 'reached_goal': result.reached_goal}
        return opening, extra, seconds

    def _check_fingers(self, name, check, plan):
        opening = self.moveit.jaw_opening()
        reason = check(opening, plan, self.cfg['grasp']['width_tolerance'])
        self._record(name, reason, jaw_opening=opening,
                     finger_position=self.moveit.finger_position())

    # -- cycle ----------------------------------------------------------------------------

    def go_ready(self):
        """Move to the ready configuration outside a cycle (arm clear of the input bin, e.g.
        before a camera view); False if planning or execution fails."""
        try:
            code, trajectory, _ = self.moveit.plan_joints(self.cfg['ready_joints'])
            if code != core.MOVEIT_SUCCESS:
                return False
            code, _ = self.moveit.execute(trajectory)
            return code == core.MOVEIT_SUCCESS
        except ManipulationError:
            return False

    def run(self, object_type, object_id, x, y, yaw, source='known_pose'):
        """Pick `object_type` resting at (x, y, yaw) in input_bin, place it in output_bin.

        `source` records where the pose comes from: 'known_pose' or 'perception'.
        """
        spec = self.catalog['objects'][object_type]

        def plans():
            in_bin = self.moveit.frame_pose(c.INPUT_BIN_FRAME)
            out_bin = self.moveit.frame_pose(c.OUTPUT_BIN_FRAME)
            params = {**self.cfg['grasp'], **self.cfg['place']}
            object_pose = core.object_pose_in_bin(in_bin, x, y, yaw, spec)
            return [core.plan_pick_place(object_pose, out_bin, self.cfg['place']['xy'], spec,
                                         self.gripper_cfg, params)]

        result = self._run(object_id, plans, spec)
        return {
            'object_type': object_type,
            'object_id': object_id,
            'object_pose': {'frame': c.INPUT_BIN_FRAME, 'x': x, 'y': y, 'yaw': yaw,
                            'source': source},
            **result,
        }

    def run_grasp(self, object_id, grasp_tcp, object_pose, width, height, place_xys,
                  pre_grasp_joints):
        """Pick with a chosen grasp an object that is already in the planning scene.

        `grasp_tcp`: gripper_tcp pose (world) with the fingers closing on `width`;
        `object_pose`: world pose of the object, `height` its size along z; `place_xys`: place
        points in output_bin, tried in order until transfer and descent are feasible;
        `pre_grasp_joints`: the collision-free arm configuration above the grasp that the
        selector validated. The fingers stay pre-shaped to the object width plus the margin.
        """
        def plans():
            out_bin = self.moveit.frame_pose(c.OUTPUT_BIN_FRAME)
            params = {**self.cfg['grasp'], **self.cfg['place']}
            return [core.plan_grasp(grasp_tcp, object_pose, width, height, out_bin, xy,
                                    self.gripper_cfg, params, preshape=True)
                    for xy in place_xys]

        result = self._run(object_id, plans, None, pre_grasp_joints)
        result['object_id'] = object_id
        return result

    def _run(self, object_id, make_plans, spec, pre_grasp_joints=None):
        self.stages, self.place_index, self.released = [], None, None
        start = time.monotonic()
        failure, recovered, grasp = None, None, None
        try:
            plans = make_plans()
            plan = plans[0]                       # the grasp side is the same for every place
            grasp = {'type': 'top_down', 'frame': c.WORLD_FRAME, 'tcp_xyz': list(plan.grasp[0]),
                     'tcp_quat': list(plan.grasp[1]), 'width_m': plan.grasp_width}
            self._cycle(plans, spec, object_id, pre_grasp_joints)
        except StageFailed as error:
            failure = error.reason
        except (ManipulationError, ValueError) as error:
            failure = 'timeout' if isinstance(error, ManipulationError) else 'planning_failed'
            self.stages.append({'stage': 'error', 'ok': False, 'reason': failure,
                                'planning_s': 0.0, 'execution_s': 0.0, 'message': str(error)})
        if failure is not None:
            recovered = self._recover(object_id)
        return {
            'grasp': grasp,
            'success': failure is None,
            'failure_reason': failure,
            'failed_stage': next((s['stage'] for s in self.stages if not s['ok']), None),
            'recovered': recovered,
            'place_index': self.place_index,
            # where the object was released, None unless the cycle got that far
            'placed_object_pose': None if self.released is None else
            [list(self.released[0]), list(self.released[1])],
            'planning_s': sum(s['planning_s'] for s in self.stages),
            'execution_s': sum(s['execution_s'] for s in self.stages),
            'total_s': time.monotonic() - start,
            'stages': self.stages,
        }

    def _choose_place(self, plans):
        """The first place option whose transfer and descent lines are feasible, planned from
        the lifted state: (plan, transfer, descent), each (code, trajectory, planning s,
        fraction). With a single option (the known-pose cycle) it is returned even if
        infeasible and the stage that executes it records the failure. With several, None is
        returned when none is feasible: nothing has moved yet, so the object can go back."""
        skipped_s, rejected = 0.0, []
        for index, plan in enumerate(plans):
            code, trajectory, fraction, planning_s = self.moveit.plan_cartesian(
                self._tool0(plan.pre_place))
            transfer, descent = (code, trajectory, planning_s, fraction), None
            if code == core.MOVEIT_SUCCESS:
                after = self.moveit.robot_state(self.moveit.end_joints(trajectory))
                code, trajectory, fraction, planning_s = self.moveit.plan_cartesian(
                    self._tool0(plan.place), start_state=after)
                descent = (code, trajectory, planning_s, fraction)
            if code == core.MOVEIT_SUCCESS or len(plans) == 1:
                self.place_index = index
                if len(plans) > 1:
                    self._record('place_select', planning_s=skipped_s, chosen=index,
                                 rejected=rejected)
                return plan, transfer, descent
            skipped_s += transfer[2] + (descent[2] if descent else 0.0)
            rejected.append({'index': index, 'stage': 'descend' if descent else 'transfer',
                             'moveit_error_code': code,
                             'cartesian_fraction': (descent or transfer)[3]})
        self.stages.append({'stage': 'place_select', 'ok': True, 'reason': None,
                            'planning_s': skipped_s, 'execution_s': 0.0, 'chosen': None,
                            'rejected': rejected})
        return None

    def _put_back(self, plan, object_id):
        """No place is feasible: set the held object down where it was picked, then fail."""
        self._line('put_back', plan.grasp)
        _, extra, seconds = self._gripper('put_back_release', plan.release_position)
        self._record('put_back_release', None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)
        self.moveit.remove_object(object_id)
        self._line('put_back_retreat', plan.pre_grasp)
        self._record('place_blocked', 'place_blocked')

    def _cycle(self, plans, spec, object_id, pre_grasp_joints):
        plan = plans[0]
        ready = self.cfg['ready_joints']
        known_pose = spec is not None             # `run`: this cycle owns the scene object
        if known_pose:
            self.moveit.remove_object(object_id)  # leftovers of an interrupted cycle

        self._joint_move('ready', ready)
        _, extra, seconds = self._gripper('open', plan.approach_position)
        self._record('open', None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)

        if known_pose:
            lifted = core.raised(plan.object_pose, self.cfg['scene']['object_clearance'])
            ok = self.moveit.add_object(object_id, spec, lifted)
            self._record('scene_add', None if ok else 'scene_failed')

        if pre_grasp_joints is None:
            self._free_move('pre_grasp', plan.pre_grasp)
        else:
            self._joint_move('pre_grasp', pre_grasp_joints)
        self._line('approach', plan.grasp)

        if plan.approach_position > plan.preclose_position + 1e-6:
            _, extra, seconds = self._gripper('preclose', plan.preclose_position)
            self._record('preclose', None if extra['reached_goal'] else 'gripper_failed',
                         execution_s=seconds, **extra)
        opening, extra, seconds = self._gripper('close', plan.close_position)
        reason = core.check_grasp(opening, plan, self.cfg['grasp']['width_tolerance'])
        self._record('close', reason, execution_s=seconds, **extra)

        ok = self.moveit.attach_object(object_id)
        self._record('attach', None if ok else 'scene_failed')

        self._line('lift', plan.lift)
        self._check_fingers('hold_after_lift', core.check_hold, plan)
        # straight line at lift height with a constant, vertical tool: a free-space plan may tilt
        # the gripper, and a tilted parallel grasp lets the object pivot out between the pads
        chosen = self._choose_place(plans)
        if chosen is None:
            self._put_back(plan, object_id)
        plan, transfer, descent = chosen
        code, trajectory, planning_s, fraction = transfer
        self._execute('transfer', code, trajectory, planning_s, cartesian_fraction=fraction)
        self._check_fingers('hold_after_transfer', core.check_hold, plan)
        code, trajectory, planning_s, fraction = descent
        self._execute('descend', code, trajectory, planning_s, cartesian_fraction=fraction)

        _, extra, seconds = self._gripper('release', plan.release_position)
        self._record('release', None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)
        self.released = plan.place_object_pose
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
