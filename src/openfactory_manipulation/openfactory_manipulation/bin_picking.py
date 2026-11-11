"""Autonomous bin picking: empty the input bin into the output bin (phase 4, D-017).

One step of the loop: look at the input bin from the ready configuration -> put every object of
the view in the planning scene -> select a grasp (grasp_selector) -> pick and place -> look
again. The second view verifies the pick (the object must be gone from the input bin) and is
the first view of the next step. The cell stops when the bin looks empty or when it gives up,
and says which.

What the cell knows comes from perception hypotheses, TF, its own actions and the fixed
infrastructure (cell layout, object catalog). The output bin is not observed: the cell
remembers what it placed where. Nothing here reads camera data or simulator state.

    ros2 run openfactory_manipulation bin_picking
"""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter

from . import constants as c
from .core import grasps, task
from .core import pick_place as core
from .grasp_selector import GraspSelector, candidate_record
from .moveit_interface import ManipulationError
from .perception_client import PerceptionClient
from .pick_place import PickPlace
from .pick_place_once import load_configs


def load_cell():
    """Cell layout (fixed infrastructure) from openfactory_description."""
    description = Path(get_package_share_directory(c.DESCRIPTION_PACKAGE)) / 'config'
    return yaml.safe_load((description / 'cell.yaml').read_text())


def object_record(obj):
    return {'index': obj.index, 'class_id': obj.class_id, 'score': obj.score, 'x': obj.x,
            'y': obj.y, 'yaw': obj.yaw, 'size': list(obj.size)}


class BinPicking:

    def __init__(self, node, config, catalog, gripper, cell):
        self.node = node
        self.cfg = config
        self.catalog = catalog
        self.inner_size = cell['bins'][c.INPUT_BIN]['inner_size']
        self.runner = PickPlace(node, config, catalog, gripper)
        self.moveit = self.runner.moveit
        self.selector = GraspSelector(self.moveit, config, catalog, gripper)
        self.client = PerceptionClient(node)
        self.state = task.TaskState()
        self.view = None                         # latest view of the input bin, see _look

    def wait_ready(self):
        self.runner.wait_ready()

    def reset(self):
        """Start a new scene: a new input bin and an empty output bin."""
        self.state = task.TaskState()
        self.view = None
        self.moveit.clear_objects()

    # -- perception ----------------------------------------------------------------------

    def _look(self):
        """View of the input bin with the arm at ready, clear of it: {'objects', 'hypotheses',
        'stamp', 'perception_s'} or None (the arm could not get there, or no view came)."""
        cfg = self.cfg['perception']
        start = time.monotonic()
        try:
            if not self.runner.go_ready():
                return None
            self.moveit.spin_for(cfg['settle_s'])
            fresh = self.client.wait_fresh(self.node.get_clock().now(), cfg['timeout_s'])
            if fresh is None:
                return None
            hypotheses, stamp = fresh
            objects = grasps.scene_objects(hypotheses, self.moveit.frame_pose(c.INPUT_BIN_FRAME))
        except ManipulationError:
            return None
        return {'objects': objects, 'hypotheses': hypotheses, 'stamp': stamp,
                'perception_s': time.monotonic() - start}

    def _spec(self, obj):
        return self.catalog['objects'].get(obj.class_id) or task.unknown_spec(obj.size)

    def _sync_scene(self, view):
        """Planning scene = every object of the view, plus what the cell placed before."""
        lift = self.cfg['scene']['object_clearance']
        by_index = {h_index: h for h_index, h in enumerate(view['hypotheses'])}
        scene = {}
        for obj in view['objects']:
            h = by_index[obj.index]
            scene[f'in_{obj.index}'] = (self._spec(obj), core.raised((h['xyz'], h['quat']), lift))
        padding = self.cfg['place']['obstacle_padding']
        for slot, placed in self.state.placed.items():
            scene[f'out_{slot}'] = (task.obstacle_spec(placed['spec'], padding), placed['pose'])
        return self.moveit.set_objects(scene)

    # -- one attempt ---------------------------------------------------------------------

    def step(self):
        """One look-select-pick-place-verify attempt.

        Returns {'done', 'end_reason', 'attempt'}: `attempt` is the record of the pick that
        was tried (None when the scene ended before one), `end_reason` is set when `done`.
        """
        limits = self.cfg['task']
        view = self.view or self._look()
        self.view = None
        if view is None:
            return self._end('no_view')
        objects = view['objects']
        pickable = [obj for obj in objects if obj.class_id in self.catalog['objects']
                    and obj.score >= self.cfg['perception']['min_score']]
        if self.state.max_attempts is None:
            self.state.max_attempts = task.attempt_budget(len(objects), limits)
        slots = task.free_slots(self.cfg['place']['slots'], self.state.placed)
        reason = task.end_reason(objects, pickable, self.state, slots, limits)
        if reason is not None:
            return self._end(reason, view)

        start = time.monotonic()
        try:
            if not self._sync_scene(view):
                return self._end('no_view', view)
            selection = self.selector.select(
                objects, self.moveit.frame_pose(c.INPUT_BIN_FRAME), self.inner_size,
                excluded=lambda grasp: task.excluded(grasp, self.state.failures, limits))
        except ManipulationError:
            return self._end('no_view', view)
        selection_s = time.monotonic() - start
        if not selection['ranked']:
            return self._end('no_feasible_grasp', view, selection)

        chosen = selection['ranked'][0]
        grasp = chosen['grasp']
        hypothesis = view['hypotheses'][grasp.object_index]
        target = next(obj for obj in objects if obj.index == grasp.object_index)
        manipulation = self.runner.run_grasp(
            f'in_{grasp.object_index}', chosen['tcp'], (hypothesis['xyz'], hypothesis['quat']),
            grasp.width, target.size[2], [xy for _, xy in slots], chosen['pre_grasp_joints'])

        # the next view verifies this pick and is the input of the next step
        self.view = self._look()
        removed = None if self.view is None else not task.still_there(
            target.x, target.y, self.view['objects'], limits['match_radius'])
        success, reason = task.attempt_outcome(manipulation['failure_reason'], removed)

        slot = None if manipulation['place_index'] is None else slots[manipulation['place_index']]
        # The place point is taken as soon as the object left the input bin on its way there,
        # also when the cycle then failed: an object lost over the output bin lies somewhere
        # around that point, and the next one must not be set down on it.
        if manipulation['target_place_pose'] is not None and removed is not False:
            xyz, quat = manipulation['target_place_pose']
            self.state.placed[slot[0]] = {'spec': self._spec(target),
                                          'pose': (tuple(xyz), tuple(quat))}
        self.state.attempts += 1
        if success:
            self.state.consecutive_failures = 0
        else:
            self.state.consecutive_failures += 1
            self.state.failures.append((grasp.x, grasp.y, grasp.finger_yaw))

        attempt = {
            'attempt': self.state.attempts - 1,
            'view_stamp': view['stamp'],
            'objects': [object_record(obj) for obj in objects],
            'target': candidate_record(chosen),
            'candidates': [candidate_record(x) for x in selection['candidates']],
            'candidates_total': selection['candidates_total'],
            'candidates_rejected': selection['candidates_rejected'],
            'place': None if slot is None else {'slot': slot[0], 'xy': list(slot[1])},
            'manipulation': manipulation,
            'verification': {
                'view_stamp': None if self.view is None else self.view['stamp'],
                'object_removed': removed},
            'success': success,
            'failure_reason': reason,
            'perception_s': view['perception_s'],
            'selection_s': selection_s,
            'planning_s': selection_s + manipulation['planning_s'],
            'execution_s': manipulation['execution_s'],
            'total_s': view['perception_s'] + selection_s + manipulation['total_s'],
        }
        return {'done': False, 'end_reason': None, 'attempt': attempt}

    def _end(self, reason, view=None, selection=None):
        end = {'done': True, 'end_reason': reason, 'attempt': None,
               'attempts': self.state.attempts, 'placed': len(self.state.placed),
               'objects_left': None if view is None else
               [object_record(obj) for obj in view['objects']]}
        if selection is not None:
            end['candidates'] = [candidate_record(x) for x in selection['candidates']]
            end['candidates_rejected'] = selection['candidates_rejected']
        return end

    def run(self, on_attempt=None):
        """Steps until the scene ends; `on_attempt(record)` is called after each attempt.
        Returns the end record with the attempts in `log`."""
        log = []
        while True:
            result = self.step()
            if result['done']:
                return {**result, 'log': log}
            log.append(result['attempt'])
            if on_attempt is not None:
                on_attempt(result['attempt'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, help='pick_place.yaml override')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])

    config, catalog, gripper = load_configs(args.config)
    rclpy.init(args=argv)
    node = Node('bin_picking',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        cell = BinPicking(node, config, catalog, gripper, load_cell())
        cell.wait_ready()
        cell.reset()

        def report(attempt):
            target = attempt['target']
            node.get_logger().info(
                f"attempt {attempt['attempt']}: {target['class_id']} {target['label']} "
                f"{'ok' if attempt['success'] else attempt['failure_reason']}, "
                f"{attempt['total_s']:.1f} s, rejected {attempt['candidates_rejected']}")

        result = cell.run(report)
        cell.moveit.clear_objects()          # leave no obstacles behind in move_group
    except ManipulationError as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    successes = sum(a['success'] for a in result['log'])
    print(json.dumps({'end_reason': result['end_reason'], 'attempts': result['attempts'],
                      'successes': successes, 'placed': result['placed'],
                      'objects_left': result['objects_left'],
                      'failures': [a['failure_reason'] for a in result['log']
                                   if not a['success']]}, indent=2))
    return 0 if result['end_reason'] == 'bin_empty' else 1


if __name__ == '__main__':
    sys.exit(main())
