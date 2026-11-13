"""Relay loop of the ring of N arms (phase 6, D-020, D-021): arm i picks from bin i and places
into bin (i + 1) mod N, for ever.

One arm moves at a time. The others stand parked, upright above their base, so that MoveIt
checks every plan of the working arm against arms that do not move, and every camera sees its
bin. One turn: look at every bin -> the next arm with something to do goes to ready -> one
bin-picking attempt (bin_picking.BinPicking.step) from its source bin to a free place point of
its destination bin -> park -> look at the destination bin: the handover counts when the object
is gone from the source bin and is seen at its place point. Among the objects of its source bin
an arm takes first the one that has been there longest, so that every object goes round.

What the cell knows comes from perception hypotheses, TF, its own actions and the fixed
infrastructure (ring layout, object catalog). Nothing here reads camera data or simulator state.

    ros2 run openfactory_manipulation relay --arms 3 [--handovers 20]
"""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from openfactory_description import ring
from rclpy.node import Node
from rclpy.parameter import Parameter

from . import constants as c
from . import names
from .bin_picking import BinPicking, load_cell, object_record
from .core import grasps, task
from .core import pick_place as core
from .core import relay as policy
from .moveit_interface import ManipulationError, MoveItInterface
from .perception_client import PerceptionClient
from .pick_place_once import load_configs


class RelayLoop:

    def __init__(self, node, config, catalog, gripper, cell, n):
        self.node = node
        self.cfg = config
        self.n = ring.check_n(n)
        self.bins = [ring.bin_name(index) for index in range(self.n)]
        self.clients = [PerceptionClient(node, ring.bin_topic(index, c.OBJECTS_TOPIC))
                        for index in range(self.n)]
        self.moveit = MoveItInterface(node, config, names.ring_arm(0))
        self.workers = []
        for arm in range(self.n):
            moveit = self.moveit if arm == 0 else self.moveit.for_arm(names.ring_arm(arm))
            self.workers.append(BinPicking(
                node, config, catalog, gripper, cell, moveit=moveit,
                source_frame=self.bins[policy.source_bin(arm)],
                destination_frame=self.bins[policy.destination_bin(arm, self.n)],
                client=self.clients[policy.source_bin(arm)]))
        self.last_arm = None
        self.turns = 0
        self.consecutive_failures = 0
        # per bin, where and in which turn the objects handed over arrived: [(x, y, turn)]
        self.arrivals = [[] for _ in range(self.n)]

    def wait_ready(self):
        for worker in self.workers:
            worker.wait_ready()

    def reset(self, seed=None):
        """Start a new run: empty planning scene, no memory of failed grasps, every arm
        parked. `seed` fixes the random order of the uniform_random ranking. Raises
        ManipulationError if an arm cannot be parked."""
        self.last_arm, self.turns, self.consecutive_failures = None, 0, 0
        self.arrivals = [[] for _ in range(self.n)]
        self.moveit.clear_objects()
        for arm, worker in enumerate(self.workers):
            worker.state = task.TaskState()
            worker.view = None
            if seed is not None:
                worker.selector.reseed(f'{seed}:{arm}')
            if not self._park(worker):
                raise ManipulationError(f'arm {arm} cannot reach its parked configuration')

    # -- views and parking ----------------------------------------------------------------

    def _views(self, bins=None):
        """A view of each of `bins` (default: all) newer than now: {bin index: {'objects',
        'hypotheses', 'stamp'}}, or None if one of them does not come. Taken while no arm is
        over a bin: every arm parked, or the working arm at ready."""
        cfg = self.cfg['perception']
        wanted = range(self.n) if bins is None else bins
        views = {}
        try:
            self.moveit.spin_for(cfg['settle_s'])
            after = self.node.get_clock().now()
            for index in wanted:
                fresh = self.clients[index].wait_fresh(after, cfg['timeout_s'])
                if fresh is None:
                    return None
                hypotheses, stamp = fresh
                objects = grasps.scene_objects(
                    hypotheses, self.moveit.frame_pose(self.bins[index]))
                views[index] = {'objects': objects, 'hypotheses': hypotheses, 'stamp': stamp}
        except ManipulationError:
            return None
        return views

    def _park(self, worker):
        """Move an arm to its parked configuration; False if it does not get there. An arm
        that cannot plan its way (it stands inside a bin after a failed recovery) first
        leaves straight up, as the bin-picking loop does."""
        park = self.cfg['relay']['park_joints']
        if worker.runner.go_joints(park):
            return True
        try:
            worker.moveit.clear_objects()
        except ManipulationError:
            return False
        return worker.runner.escape() and worker.runner.go_joints(park)

    def _destination(self, worker, view):
        """What a view of the destination bin gives the attempt: the free place points and
        the objects standing there, as obstacles grown by the padding of placed objects."""
        relay, place = self.cfg['relay'], self.cfg['place']
        lift = self.cfg['scene']['object_clearance']
        obstacles = {}
        for obj in view['objects']:
            hypothesis = view['hypotheses'][obj.index]
            obstacles[f'dst_{obj.index}'] = (
                task.obstacle_spec(worker._spec(obj), place['obstacle_padding']),
                core.raised((hypothesis['xyz'], hypothesis['quat']), lift))
        return {'slots': policy.free_slots(place['slots'], view['objects'],
                                           relay['place_margin']),
                'obstacles': obstacles}

    # -- one turn -------------------------------------------------------------------------

    def step(self):
        """One turn of the loop.

        Returns {'done', 'end_reason', 'turn'}: `turn` is the record of what the arm whose
        turn it was did (None when the loop ended before), `end_reason` is set when `done`.
        """
        relay, limits = self.cfg['relay'], self.cfg['task']
        start = time.monotonic()
        views = self._views()
        if views is None:
            return self._end('no_view')
        self.arrivals = [policy.prune_arrivals(arrivals, views[index]['objects'],
                                               limits['match_radius'])
                         for index, arrivals in enumerate(self.arrivals)]
        destinations, ready = [], []
        for arm, worker in enumerate(self.workers):
            source = views[policy.source_bin(arm)]
            destination = self._destination(worker, views[policy.destination_bin(arm, self.n)])
            destinations.append(destination)
            ready.append(policy.can_work(worker.selector.pickable(source['objects']),
                                         destination['slots']))
        arm = policy.next_arm(self.last_arm, ready)
        if arm is None:
            return self._end('no_work', views)
        survey_s = time.monotonic() - start

        worker = self.workers[arm]
        source_index = policy.source_bin(arm)
        destination_index = policy.destination_bin(arm, self.n)
        # a new attempt for this arm: its budgets start again, and it forgets the failed
        # grasps whose object is no longer there
        worker.state = task.TaskState(failures=policy.prune_failures(
            worker.state.failures, views[source_index]['objects'], limits['match_radius']))
        worker.view = None                       # the arm looks again once it is at ready
        # the object that has been in the source bin longest goes first
        arrivals = self.arrivals[source_index]
        result = worker.step(
            destinations[arm],
            priority=lambda obj: policy.waiting_since(obj, arrivals, limits['match_radius']))
        parked = self._park(worker)
        attempt = result['attempt']
        waited = None
        if attempt is not None:
            target = policy.nearest((attempt['target']['x'], attempt['target']['y']),
                                    views[source_index]['objects'], limits['match_radius'])
            if target is not None:
                waited = policy.waiting_since(target, arrivals, limits['match_radius'])

        arrival, arrival_view = None, None
        if attempt is not None and attempt['success']:
            after = self._views([destination_index]) if parked else None
            if after is not None:
                arrival_view = after[destination_index]
                seen = policy.nearest(attempt['place']['xy'], arrival_view['objects'],
                                      relay['arrival_radius'])
                arrival = seen is not None
                if arrival:
                    self.arrivals[destination_index].append((seen.x, seen.y, self.turns))
        if attempt is None:
            success, reason = False, None        # nothing was tried: not a handover
        else:
            success, reason = policy.handover_outcome(
                attempt['success'], attempt['failure_reason'], arrival)
            self.consecutive_failures = 0 if success else self.consecutive_failures + 1

        turn = {
            'turn': self.turns,
            'arm': arm,
            'source_bin': self.bins[source_index],
            'destination_bin': self.bins[destination_index],
            'bins_seen': {self.bins[index]: [object_record(obj) for obj in view['objects']]
                          for index, view in views.items()},
            'free_slots': [index for index, _ in destinations[arm]['slots']],
            # why the arm did nothing, when its own view at ready gave it nothing to try
            'skipped': result['end_reason'] if attempt is None else None,
            'attempt': attempt,
            # turn in which the chosen object arrived in the source bin, -1: there from the start
            'target_waiting_since': waited,
            'arrival': {
                'seen': arrival,
                'view_stamp': None if arrival_view is None else arrival_view['stamp'],
                'objects': None if arrival_view is None else
                [object_record(obj) for obj in arrival_view['objects']]},
            'parked': parked,
            'success': success,
            'failure_reason': reason,
            'survey_s': survey_s,
            'total_s': time.monotonic() - start,
        }
        self.last_arm = arm
        self.turns += 1
        if not parked:
            return {'done': True, 'end_reason': 'park_failed', 'turn': turn}
        if self.consecutive_failures >= relay['max_consecutive_failures']:
            return {'done': True, 'end_reason': 'failure_budget', 'turn': turn}
        return {'done': False, 'end_reason': None, 'turn': turn}

    def _end(self, reason, views=None):
        return {'done': True, 'end_reason': reason, 'turn': None,
                'bins_seen': None if views is None else {
                    self.bins[index]: [object_record(obj) for obj in view['objects']]
                    for index, view in views.items()}}

    def run(self, handovers=None, on_turn=None):
        """Turns until `handovers` of them were tried (None: for ever) or the loop ends on
        its own; `on_turn(record)` is called after each turn. Returns {'end_reason', 'log',
        ...counts of core.relay.summarize}."""
        log, tried, idle = [], 0, 0
        while True:
            result = self.step()
            turn = result['turn']
            if turn is not None:
                log.append(turn)
                tried += turn['attempt'] is not None
                # an arm that had work in the survey and found none at ready
                idle = 0 if turn['attempt'] is not None else idle + 1
                if on_turn is not None:
                    on_turn(turn)
            reason = policy.run_end(result['end_reason'], tried, handovers, idle, self.n)
            if reason is not None:
                return {'end_reason': reason, 'log': log, **policy.summarize(log, self.n)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--arms', type=int, required=True, help='number of arms of the ring')
    parser.add_argument('--handovers', type=int, help='stop after this many (default: never)')
    parser.add_argument('--config', type=Path, help='pick_place.yaml override')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])

    config, catalog, gripper = load_configs(args.config)
    rclpy.init(args=argv)
    node = Node('relay',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        loop = RelayLoop(node, config, catalog, gripper, load_cell(), args.arms)
        loop.wait_ready()
        loop.reset()

        def report(turn):
            attempt = turn['attempt']
            what = (f"skipped ({turn['skipped']})" if attempt is None else
                    f"{attempt['target']['class_id']} {attempt['target']['label']} "
                    f"{'ok' if turn['success'] else turn['failure_reason']}")
            node.get_logger().info(
                f"turn {turn['turn']}: arm {turn['arm']} {turn['source_bin']} -> "
                f"{turn['destination_bin']} {what}, {turn['total_s']:.1f} s")

        result = loop.run(args.handovers, report)
        loop.moveit.clear_objects()          # leave no obstacles behind in move_group
    except ManipulationError as error:
        node.get_logger().error(str(error))
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({key: value for key, value in result.items() if key != 'log'}, indent=2))
    return 0 if result['end_reason'] == 'handovers_done' else 1


if __name__ == '__main__':
    sys.exit(main())
