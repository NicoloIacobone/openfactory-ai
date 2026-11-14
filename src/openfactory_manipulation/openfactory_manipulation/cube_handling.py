"""The two motions of the Rubik's cube cell (phase 7, D-024): turn and flip.

The cube sits in the nest, which holds its bottom layer. The gripper holds it from above by
the two layers on top of that one (`core/rubik/station.py`).

    turn(k)          grasp, turn the wrist k quarter turns about the vertical, let go: the
                     bottom layer stays in the nest, the rest of the cube turns
    flip(kind, yaw)  grasp, lift, turn the held cube `yaw` quarter turns about the vertical,
                     tip it a quarter turn about the finger axis, set it down with the tool
                     horizontal, let go

Every move is planned by MoveIt against the fixtures and the cube: free-space plans between
the ready configuration and the pre-grasp, straight lines and arcs (Cartesian paths) with the
cube. The cube is in the planning scene as two boxes, the bottom layer and the two layers
above it, slightly smaller than the real ones; the held part is attached to the gripper.
Nothing here reads simulator state: where the cube is comes from the nest, a fixture.
"""

import math
import time

from . import constants as c
from .core import pick_place as core
from .core.geometry import compose
from .core.rubik import station as st
from .moveit_interface import ManipulationError
from .pick_place import PickPlace, StageFailed

NEST_FRAME = c.NEST_FRAME
GRIPPER_FILE = 'rubik_gripper.yaml'      # in openfactory_description/config
BASE_ID, TOP_ID = 'cube_base', 'cube_top'
UP = (0.0, 0.0, 1.0)


class CubeHandler(PickPlace):

    def __init__(self, node, config, rubik, cube, gripper, moveit=None):
        """`config`: pick_place.yaml (planning, lines, timeouts); `rubik`: this package's
        rubik.yaml; `cube`: openfactory_description's rubik.yaml; `gripper`: gripper.yaml."""
        super().__init__(node, {**config, 'ready_joints': rubik['ready_joints']}, None, gripper,
                         moveit)
        self.rubik = rubik
        grasp = rubik['grasp']
        self.station = st.Station(cube['cube']['pitch'], gripper, grasp['tip_margin'])
        # finger joint value at which the pads touch the cube (the joint measures the inner
        # faces of the fingers, the pads stand out of them)
        pad = gripper['finger'].get('pad', {'size': [0.0, 0.0, 0.0]})['size'][1]
        self.held_opening = self.station.side + 2 * pad
        self.open_position = self.held_opening / 2 + grasp['preclose_margin']
        self.close_position = self.held_opening / 2 - grasp['squeeze_depth']
        self.width_tolerance = grasp['width_tolerance']
        self._nest = None
        # what this object believes about the cube: 'seated' (in the planning scene, in the
        # nest), 'held' (attached to the gripper) or 'unknown' (after a failure while held)
        self.cube = 'seated'
        self.upright = True       # while held: the tool still points down

    # -- geometry -------------------------------------------------------------------------

    def nest(self):
        if self._nest is None:
            self._nest = self.moveit.frame_pose(NEST_FRAME)
        return self._nest

    def _world(self, pose):
        return compose(self.nest(), pose)

    def _path(self, name, poses, scaling=None):
        """Cartesian path through gripper_tcp poses given in the nest frame."""
        code, trajectory, fraction, planning_s = self.moveit.plan_cartesian_path(
            [self._tool0(self._world(pose)) for pose in poses], scaling=scaling)
        self._execute(name, code, trajectory, planning_s, cartesian_fraction=fraction)

    @staticmethod
    def _turn_options(quarter_turns):
        """The wrist turns (rad about the vertical) that make `quarter_turns`: a half turn
        can be made either way, three quarters are one the other way."""
        return {0: [0.0], 1: [math.pi / 2], 2: [math.pi, -math.pi],
                3: [-math.pi / 2]}[quarter_turns % 4]

    def _pre_grasp(self, finger_axis, turns):
        """(collision-free joints above the grasp, the finger axis chosen, the turn chosen).

        `turns`: the turns about the vertical (rad) one of which follows the grasp;
        `finger_axis(turn)`: the axis the fingers must close along for that turn. The
        gripper is the same turned half a turn about its axis, so an axis and its opposite
        are the same grasp with the wrist half a turn apart. Of all these the choice that
        leaves wrist_3 the most room is taken.
        """
        s = self.station
        best = None
        for turn in turns:
            axis = finger_axis(turn)
            for candidate in (axis, tuple(-v for v in axis)):
                above = s.tcp_pose(s.lifted(self.rubik['grasp']['approach_height']), st.DOWN,
                                   candidate)
                code, joints = self.moveit.compute_ik(
                    self._tool0(self._world(above)), self.cfg['ready_joints'],
                    self.open_position, self.cfg['selection']['ik_timeout_s'] * 4)
                if joints is None:
                    continue
                # the tool points down: a turn about the vertical is the opposite turn of wrist_3
                room = 2 * math.pi - max(abs(joints[5]), abs(joints[5] - turn))
                if best is None or room > best[0]:
                    best = (room, joints, candidate, turn)
        if best is None or best[0] < 0.1:
            self._record('pre_grasp_ik', 'ik_failed')
        return best[1], best[2], best[3]

    # -- planning scene -------------------------------------------------------------------

    def _boxes(self, center, top_axis=UP):
        """The cube whose centre is `center` (nest frame) as two planning-scene boxes: the
        layer on the side opposite to `top_axis`, and the two layers the gripper holds."""
        pitch, shrink = self.station.pitch, self.rubik['scene']['shrink']
        side = self.station.side - 2 * shrink

        def box(offset, thickness):
            size = [side - (side - thickness) * abs(a) for a in top_axis]
            xyz = tuple(c + offset * a for c, a in zip(center, top_axis))
            return ({'shape': 'box', 'size': size},
                    self._world((xyz, (0.0, 0.0, 0.0, 1.0))))

        return {BASE_ID: box(-pitch, pitch - 2 * shrink), TOP_ID: box(pitch / 2, 2 * pitch - 2 * shrink)}

    def _scene(self, name, center=None):
        """Put the seated cube in the planning scene (after detaching what was held)."""
        for object_id in (BASE_ID, TOP_ID):
            self.moveit.remove_object(object_id)
        ok = self.moveit.set_objects(self._boxes(center or self.station.seat))
        self.cube = 'seated'
        self._record(name, None if ok else 'scene_failed')

    def _attach(self, name, ids):
        ok = all(self.moveit.attach_object(object_id) for object_id in ids)
        self.cube, self.upright = 'held', True
        self._record(name, None if ok else 'scene_failed')

    # -- shared steps ---------------------------------------------------------------------

    def _open(self, name):
        _, extra, seconds = self._gripper(name, self.open_position)
        self._record(name, None if extra['reached_goal'] else 'gripper_failed',
                     execution_s=seconds, **extra)

    def _held(self, name, reason):
        opening = self.moveit.jaw_opening()
        ok = opening is not None and abs(opening - self.held_opening) <= self.width_tolerance
        self._record(name, None if ok else reason, jaw_opening=opening,
                     finger_position=self.moveit.finger_position())

    def _grasp(self, finger_axis, turns):
        """From ready to the cube held from above; returns (finger axis used, the turn in rad
        chosen among `turns`). Arguments as `_pre_grasp`."""
        s = self.station
        self._joint_move('ready', self.cfg['ready_joints'])
        self._open('open')
        self._scene('scene')
        joints, axis, turn = self._pre_grasp(finger_axis, turns)
        self._joint_move('pre_grasp', joints)
        self._path('approach', [s.tcp_pose(s.seat, st.DOWN, axis)])
        opening, extra, seconds = self._gripper('close', self.close_position)
        ok = opening is not None and abs(opening - self.held_opening) <= self.width_tolerance
        self._record('close', None if ok else 'grasp_failed', execution_s=seconds, **extra)
        return axis, turn

    def _steps(self, angle, spacing):
        return max(1, math.ceil(abs(angle) / spacing - 1e-9))

    # -- operations -----------------------------------------------------------------------

    def turn(self, quarter_turns):
        """Turn the cube above its bottom layer `quarter_turns` x 90 degrees about the
        vertical (right-handed: positive is anticlockwise seen from above)."""
        return self._operation(
            {'operation': 'turn', 'quarter_turns': quarter_turns},
            lambda: self._turn(self._turn_options(quarter_turns), 0.0))

    def realign(self, yaw):
        """Turn back the part of the cube above its bottom layer, which a view saw `yaw` rad
        out of square about the vertical: grasp it as it stands, turn it by -yaw."""
        return self._operation({'operation': 'realign', 'yaw': yaw},
                               lambda: self._turn([-yaw], yaw))

    def _turn(self, turns, yaw):
        s = self.station
        fingers = st.rotate_about(UP, yaw, (0.0, 1.0, 0.0))
        axis, angle = self._grasp(lambda turn: fingers, turns)
        self._attach('attach', [TOP_ID])
        steps = self._steps(angle, self.rubik['turn']['waypoint_angle'])
        self._path('turn', s.spin(s.seat, axis, angle, steps), self.rubik['turn']['scaling'])
        self._held('hold_after_turn', 'grasp_slipped')
        self._open('release')
        self._scene('detach')
        end_axis = st.rotate_about(UP, angle, axis)
        self._path('retreat', [s.tcp_pose(
            s.lifted(self.rubik['grasp']['approach_height']), st.DOWN, end_axis)])
        self._joint_move('return', self.cfg['ready_joints'])

    def flip(self, kind, yaw=0):
        """Tip the cube a quarter turn as `kind` of rubik.yaml says, after turning it `yaw`
        quarter turns about the vertical in the air (positive anticlockwise from above)."""
        spec = self.rubik['flip']['kinds'][kind]
        finger = tuple(float(v) for v in spec['finger_axis'])
        tool = tuple(float(v) for v in spec['tool_axis'])
        cross = st._cross(st.DOWN, tool)
        tip = math.copysign(math.pi / 2, sum(a * b for a, b in zip(cross, finger)))

        def body():
            s, cfg = self.station, self.rubik['flip']
            lifted = s.lifted(cfg['lift_height'])
            # the fingers must end on `finger` after the turn in the air
            axis, spin = self._grasp(lambda turn: st.rotate_about(UP, -turn, finger),
                                     self._turn_options(yaw))
            sign = 1.0 if sum(a * b for a, b in zip(st.rotate_about(UP, spin, axis), finger)) > 0 else -1.0
            end_finger = tuple(sign * v for v in finger)
            self._attach('attach', [BASE_ID, TOP_ID])
            self._path('lift', [s.tcp_pose(lifted, st.DOWN, axis)])
            self._held('hold_after_lift', 'grasp_slipped')
            if spin:
                self._path('yaw', s.spin(lifted, axis, spin, self._steps(
                    spin, self.rubik['turn']['waypoint_angle'])), cfg['scaling'])
            # tipping about -f by -a is tipping about f by a: the sign of the axis the fingers
            # ended on does not change where the tool goes
            self.upright = False
            self._path('tip', s.arc(lifted, end_finger, sign * tip,
                                    self._steps(tip, cfg['waypoint_angle'])), cfg['scaling'])
            self._held('hold_after_tip', 'grasp_slipped')
            down = s.tcp_pose(s.lifted(cfg['release_height']), tool, end_finger)
            self._path('descend', [down])
            self._open('release')
            self._scene('detach')
            back = s.retreat(down, cfg['retreat'])
            self._path('retreat', [back])
            # straight up into open space before the free plan back to ready
            self._path('rise', [((back[0][0], back[0][1], back[0][2] + cfg['rise']), back[1])])
            self._joint_move('return', self.cfg['ready_joints'])

        return self._operation({'operation': 'flip', 'kind': kind, 'yaw': yaw}, body)

    def _operation(self, record, body):
        self.stages = []
        self.recovery_position = self.open_position
        start = time.monotonic()
        failure, recovered = None, None
        try:
            body()
        except StageFailed as error:
            failure = error.reason
        except (ManipulationError, ValueError) as error:
            failure = 'timeout' if isinstance(error, ManipulationError) else 'planning_failed'
            self.stages.append({'stage': 'error', 'ok': False, 'reason': failure,
                                'planning_s': 0.0, 'execution_s': 0.0, 'message': str(error)})
        if failure is not None:
            recovered = self._recover_cube()
        return {
            **record,
            'cube': self.cube,
            'success': failure is None,
            'failure_reason': failure,
            'failed_stage': next((s['stage'] for s in self.stages if not s['ok']), None),
            'recovered': recovered,
            'planning_s': sum(s['planning_s'] for s in self.stages),
            'execution_s': sum(s['execution_s'] for s in self.stages),
            'total_s': time.monotonic() - start,
            'stages': self.stages,
        }

    def _recover_cube(self):
        """Best effort back to ready after a failure.

        A cube that is seated stays in the planning scene, so that the way back goes round
        it. A cube that is held with the tool still pointing down is set down where it was
        taken from. Otherwise it is let go where it is and taken out of the planning scene:
        where it ends is not known, and the next view has to say.
        """
        steps = {'cube': self.cube}
        cfg = self.rubik['flip']
        try:
            if self.cube == 'held' and self.upright:
                tcp = self.moveit.frame_pose(self.moveit.names.tcp_link)
                seat = self._world(self.station.tcp_pose(self.station.seat, st.DOWN,
                                                         (0.0, 1.0, 0.0)))
                down = ((tcp[0][0], tcp[0][1], seat[0][2] + cfg['release_height']), tcp[1])
                code, trajectory, fraction, _ = self.moveit.plan_cartesian(self._tool0(down))
                steps['put_back_fraction'] = fraction
                if code == core.MOVEIT_SUCCESS:
                    self.moveit.execute(trajectory)
            result, _ = self.moveit.gripper(self.open_position, self.cfg['grasp']['max_effort'])
            steps['open'] = bool(result.reached_goal)
            if self.cube == 'held':
                put_back = self.upright and steps.get('put_back_fraction', 0.0) >= 0.99
                for object_id in (BASE_ID, TOP_ID):
                    self.moveit.remove_object(object_id)
                if put_back:
                    self.moveit.set_objects(self._boxes(self.station.seat))
                self.cube = 'seated' if put_back else 'unknown'
            tool0 = self.moveit.frame_pose(self.moveit.names.tool_link)
            back = self.station.retreat(tool0, cfg['retreat'])
            code, trajectory, fraction, _ = self.moveit.plan_cartesian(back)
            steps['back_fraction'] = fraction
            if code == core.MOVEIT_SUCCESS:
                self.moveit.execute(trajectory)
            steps['ok'] = self.go_ready() or self.escape()
        except ManipulationError as error:
            steps.update(ok=False, error=str(error))
        steps['cube_after'] = self.cube
        return steps
