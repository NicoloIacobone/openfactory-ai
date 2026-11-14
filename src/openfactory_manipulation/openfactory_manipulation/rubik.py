"""Rubik's cube cell (phase 7, D-027): look at the cube, work out its state, solve it.

    scan      the cube stands in the nest in an unknown state and orientation. The arm looks
              at the face that is up, flips the cube, looks again, until the six faces have
              been seen (core/rubik/scan.py)
    solve     the moves that solve that state (kociemba) become flips, which bring a face
              down, and turns of that face (core/rubik/plan.py)
    verify    after every operation the camera must show a cube seated and square in the nest,
              and the face that the believed state says is up. A view that disagrees ends the
              plan: the cube is scanned again and solved from what is there
    done      a last scan must show the solved cube

What the cell knows comes from the views of cube_perception, its own actions and the fixed
infrastructure (nest, cube size and colours). Nothing here reads camera data or simulator
state.

    ros2 run openfactory_manipulation rubik [--scramble 25 --seed 1]
"""

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter

from . import constants as c
from .core.rubik import cube, plan, scan, solver
from .core.rubik import rotations as rot
from .core.rubik import view as views
from .cube_handling import GRIPPER_FILE, CubeHandler
from .cube_view_client import CubeViewClient
from .moveit_interface import ManipulationError
from .pick_place_once import load_configs

END_REASONS = ('solved', 'scan_failed', 'cube_lost', 'not_square', 'no_view', 'failure_budget',
               'operation_budget', 'solver_failed')


def load_rubik_configs():
    """(pick_place.yaml, this package's rubik.yaml, the cube's rubik.yaml, the gripper)."""
    config, _, gripper = load_configs(gripper_file=GRIPPER_FILE)
    own = Path(get_package_share_directory(c.PACKAGE)) / 'config'
    description = Path(get_package_share_directory(c.DESCRIPTION_PACKAGE)) / 'config'
    return (config, yaml.safe_load((own / 'rubik.yaml').read_text()),
            yaml.safe_load((description / 'rubik.yaml').read_text()), gripper)


class Stop(Exception):
    """The loop cannot go on; `reason` is one of END_REASONS."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class Rescan(Exception):
    """A view is not what the believed state predicts: scan again."""


class RubikSolver:

    def __init__(self, node, config, rubik, cube_config, gripper, on_start=None, on_done=None,
                 solve=solver.solve):
        """`on_start(operation)` and `on_done(record)` are called around every flip and turn:
        the harness records the executed states and reads ground truth there. `solve`: state
        -> moves (kociemba by default)."""
        self.solve = solve
        self.node = node
        self.rubik = rubik
        self.handler = CubeHandler(node, config, rubik, cube_config, gripper)
        self.moveit = self.handler.moveit
        self.client = CubeViewClient(node)
        self.reference = {face: tuple(255.0 * v for v in rgba[:3])
                          for face, rgba in cube_config['cube']['colors'].items()}
        self.kinds = plan.kind_rotations(rubik['flip']['kinds'])
        self.options = plan.flip_options(self.kinds, rubik['plan'])
        self.on_start = on_start or (lambda operation: None)
        self.on_done = on_done or (lambda record: None)
        self.reset()

    def wait_ready(self):
        self.handler.wait_ready()

    def reset(self):
        """Forget the cube: a new one is in the nest."""
        self.state, self.orientation, self.centers = None, None, None
        self.log, self.scans, self.operations = [], 0, 0

    # -- views ----------------------------------------------------------------------------

    def look(self):
        """A view of the cube with the arm at ready, or None."""
        if not self.handler.go_ready():
            return None
        cfg = self.rubik['view']
        self.moveit.spin_for(cfg['settle_s'])
        return self.client.wait_fresh(self.node.get_clock().now(), cfg['timeout_s'],
                                      self.handler.nest(), self.handler.station.seat)

    def _usable(self, view):
        """The view, once it shows a cube seated and square; a cube that stands in its seat
        with its top turned about the vertical is turned back first. Raises Stop otherwise."""
        limits = self.rubik['view']
        realigns = 0
        while True:
            problem = views.problem(view, limits)
            if problem is None:
                return view
            if problem == 'no_view':
                raise Stop('no_view')
            if problem == 'no_cube':
                raise Stop('cube_lost')
            in_seat = (math.hypot(view.offset[0], view.offset[1]) <= limits['seat_offset']
                       and abs(view.offset[2]) <= limits['seat_height']
                       and view.tilt <= limits['seat_tilt'])
            if (not in_seat or abs(view.yaw) <= limits['seat_yaw']
                    or realigns >= self.rubik['task']['realigns']):
                raise Stop('cube_lost' if not in_seat else 'not_square')
            realigns += 1
            view = self._run(('realign', view.yaw), 'realign')['view_object']

    # -- operations -----------------------------------------------------------------------

    def _run(self, operation, phase):
        """Make one operation and look. Returns its record (with the view as `view_object`)."""
        if self.operations >= self.rubik['task']['max_operations']:
            raise Stop('operation_budget')
        self.operations += 1
        self.on_start(operation)
        if operation[0] == 'turn':
            result = self.handler.turn(operation[1])
        elif operation[0] == 'flip':
            result = self.handler.flip(operation[1], operation[2])
        else:
            result = self.handler.realign(operation[1])
        view = self.look()
        record = {
            'index': len(self.log), 'phase': phase, 'operation': list(operation),
            'cell_success': result['success'], 'cell_failure_reason': result['failure_reason'],
            'failed_stage': result['failed_stage'], 'recovered': result['recovered'],
            'cube': result['cube'], 'planning_s': result['planning_s'],
            'execution_s': result['execution_s'], 'total_s': result['total_s'],
            'stages': result['stages'],
            'view': None if view is None else {
                'present': view.present, 'offset': list(view.offset), 'yaw': view.yaw,
                'tilt': view.tilt, 'score': view.score},
            'view_problem': views.problem(view, self.rubik['view']),
            'view_object': view,
        }
        self.log.append(record)
        self.on_done(record)
        return record

    def _step(self, operation, phase):
        """Make `operation` and bring the belief up to date; returns the view after it.

        While the state is known the view is checked against it: the face that must be up
        after the operation, or, if the cell reported a failure, the face that was up before
        it (nothing moved: try again). Anything else raises Rescan.
        """
        retries = self.rubik['task']['retries']
        for attempt in range(retries + 1):
            record = self._run(operation, phase)
            view = self._usable(record['view_object'])
            before = (self.state, self.orientation)
            after = self._after(operation)
            if self.centers is None:
                # scanning: nothing to compare with yet. A reported failure with the cube
                # still seated means nothing was moved.
                done = record['cell_success']
            else:
                seen = scan.read_face(view.stickers, self.centers)
                expected = [scan.expected_face(*belief) for belief in (before, after)]
                record.update(seen_face=seen, expected_face=expected[1])
                if seen == expected[1] and (record['cell_success'] or seen != expected[0]):
                    done = True
                elif seen == expected[0] and not record['cell_success']:
                    done = False
                else:
                    record['verified'] = False
                    raise Rescan()
            record['verified'] = done
            if done:
                self.state, self.orientation = after
                record.update(believed_state=self.state,
                              believed_orientation=[list(row) for row in self.orientation])
                return view
        raise Stop('failure_budget')

    def _after(self, operation):
        """Believed (state, orientation) after an operation; the state may be unknown."""
        orientation = self.orientation
        if orientation is None:
            return None, None
        if self.state is None:
            return None, rot.mat_mul(plan.operation_rotation(operation, self.kinds), orientation)
        return plan.apply(self.state, orientation, operation, self.kinds)

    # -- phases ---------------------------------------------------------------------------

    def scan(self):
        """Look at the six faces and set the believed state and orientation. Raises Stop."""
        for _ in range(self.rubik['task']['rescans'] + 1 - self.scans):
            self.scans += 1
            self.state, self.centers = None, None
            # during a scan the orientation is counted from where the cube stands now
            self.orientation = rot.IDENTITY
            view = self._usable(self.look())
            seen = [(rot.IDENTITY, view.stickers)]
            shown = {views.up_direction(rot.IDENTITY)}
            for operation in plan.scan_plan(self.options):
                view = self._step(operation, 'scan')
                up = views.up_direction(self.orientation)
                if up not in shown:
                    shown.add(up)
                    seen.append((self.orientation, view.stickers))
            turned = self.orientation
            result = scan.assemble(seen, self.reference)
            self.log.append({'index': len(self.log), 'phase': 'scan', 'operation': ['assemble'],
                             'error': result.get('error'), 'state': result.get('state'),
                             'doubtful': result.get('doubtful')})
            if 'error' not in result:
                self.state, self.centers = result['state'], result['centers']
                self.orientation = rot.mat_mul(turned, result['orientation'])
                return
        raise Stop('scan_failed')

    def make(self, moves, phase):
        """Make `moves` on the cube with flips and turns."""
        operations, _ = plan.solve_plan(moves, self.orientation, self.options,
                                        self.rubik['plan']['turn'])
        for operation in operations:
            self._step(operation, phase)

    def run(self, scramble=None):
        """Scan, (scramble with the given moves,) solve, scan again. Returns
        {'end_reason', 'solved', 'state', 'operations', 'scans', 'log', ...}."""
        start = time.monotonic()
        solution, reason, scanned = None, None, None
        try:
            self.scan()
            scanned = self.state
            if scramble:
                try:
                    self.make(scramble, 'scramble')
                except Rescan:
                    self.scan()
            while True:
                try:
                    if self.state == cube.SOLVED:
                        # believed solved: only a fresh look at the six faces counts
                        if self.log and self.log[-1].get('operation') == ['assemble']:
                            reason = 'solved'
                            break
                        self.scan()
                        continue
                    try:
                        solution = self.solve(self.state)
                    except (ImportError, ValueError) as error:
                        self.log.append({'index': len(self.log), 'phase': 'solve',
                                         'operation': ['solve'], 'error': str(error)})
                        raise Stop('solver_failed') from error
                    self.log.append({'index': len(self.log), 'phase': 'solve',
                                     'operation': ['solve'], 'state': self.state,
                                     'moves': solution})
                    self.make(solution, 'solve')
                except Rescan:
                    self.scan()
        except Stop as stop:
            reason = stop.reason
        except ManipulationError as error:
            reason = 'failure_budget'
            self.log.append({'index': len(self.log), 'phase': 'error', 'operation': ['error'],
                             'error': str(error)})
        for record in self.log:
            record.pop('view_object', None)
        done = [r for r in self.log if r['operation'][0] in ('turn', 'flip', 'realign')]
        return {
            'end_reason': reason, 'solved': reason == 'solved', 'state': self.state,
            'scanned_state': scanned, 'solution': solution,
            'operations': len(done), 'scans': self.scans,
            'turns': sum(r['operation'][0] == 'turn' for r in done),
            'flips': sum(r['operation'][0] == 'flip' for r in done),
            'cell_failures': sum(not r['cell_success'] for r in done),
            'total_s': time.monotonic() - start, 'log': self.log,
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scramble', type=int, default=0,
                        help='random moves the arm makes on the cube before solving it')
    parser.add_argument('--seed', type=int, default=0, help='seed of those moves')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])

    rclpy.init(args=argv)
    node = Node('rubik',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        cell = RubikSolver(node, *load_rubik_configs(), on_done=lambda record: node.get_logger(
        ).info(f"{record['phase']} {record['operation']}: "
               f"{'ok' if record['cell_success'] else record['cell_failure_reason']}, "
               f"view {record['view_problem'] or 'ok'}, {record['total_s']:.1f} s"))
        cell.wait_ready()
        cell.moveit.clear_objects()
        moves = cube.random_scramble(random.Random(args.seed), args.scramble) \
            if args.scramble else None
        result = cell.run(moves)
        cell.moveit.clear_objects()
    except ManipulationError as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result.pop('log')
    print(json.dumps(result, indent=2))
    return 0 if result['solved'] else 1


if __name__ == '__main__':
    sys.exit(main())
