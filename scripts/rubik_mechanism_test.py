#!/usr/bin/env python3
"""Test bench of the Rubik's cube mechanism (phase 7, E-007 part A): no robot.

A cube whose core is fixed to the world is spawned in the bench world; its faces are turned
by torques applied to their centre cubies through Gazebo's ApplyLinkWrench system, and the
cube's state is read from the simulator's link poses after every turn and compared with the
logical model. This drive exists only on this bench: in the cell a layer turns under the
forces of the gripper and of the nest.

    faces      every face by a quarter turn clockwise, a half turn and a quarter turn back
    below      a torque below the breakaway torque turns nothing, on a free and on a locked axis
    locking    with a layer left 30 degrees out of square the other axes do not turn
    drift      N seeded random turns: state after every turn, cubies still on the grid

Run in the VM with the bench launched (docs/environment.md):
    scripts/vm/launch.sh bench openfactory_simulation rubik_bench.launch.py
    scripts/vm/run_benchmark.sh p7_bench --script rubik_mechanism_test.py [--turns 400]
Logs: ~/openfactory_data/trials/<run_id>/ (turns.jsonl, summary.json).
"""

import argparse
import json
import math
import random
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import cubies as cb

import rubik_model as rm
import rubik_truth as rt
import trial_log as tl
from gz_scene import GzScene

REPO = Path(__file__).resolve().parents[1]
WORLD = 'rubik_bench'
SPAWN_XYZ = (0.0, 0.0, 0.5)
DRIVE_TORQUE = 0.065               # N m, above the breakaway torque of rubik.yaml
STOP_MARGIN = math.radians(6.0)    # stop pushing this far from the target: the detent finishes
TURN_ANGLE = {1: -math.pi / 2, 2: -math.pi, 3: math.pi / 2}   # about the outward normal


class Bench:
    """The cube in the bench world: spawn, torque on a face centre, ground truth."""

    def __init__(self, config):
        self.config = config
        self.pitch = config['cube']['pitch']
        self.scene = GzScene(WORLD)
        self.node = self.scene.node
        self._lock = threading.Lock()
        self._poses, self._stamp, self._status = {}, 0.0, {}
        self._wrench = self.node.advertise(f'/world/{WORLD}/wrench/persistent', EntityWrench)
        self._clear = self.node.advertise(f'/world/{WORLD}/wrench/clear', Entity)
        self._topics = {f'/world/{WORLD}/dynamic_pose/info': (Pose_V, self._on_poses),
                        f'/model/{rm.MODEL_NAME}/mechanism': (StringMsg, self._on_status)}

    def _listen(self, on):
        """Subscriptions are off during a service request: a request holds the interpreter
        while it waits, the callbacks then block the transport thread and the reply is never
        read (the request times out although the world served it)."""
        for topic, (msg_type, callback) in self._topics.items():
            if not on:
                self.node.unsubscribe(topic)
            elif not self.node.subscribe(msg_type, topic, callback):
                raise RuntimeError(f'cannot subscribe to {topic}')

    def _on_poses(self, msg):
        poses = {}
        for pose in msg.pose:
            if pose.name in rt.LINKS:
                p, q = pose.position, pose.orientation
                poses[pose.name] = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        if len(poses) == len(rt.LINKS):
            with self._lock:
                self._poses = poses
                self._stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec

    def _on_status(self, msg):
        with self._lock:
            self._status = json.loads(msg.data)

    def poses(self):
        with self._lock:
            return dict(self._poses), self._stamp

    def status(self):
        with self._lock:
            return dict(self._status)

    def sim_time(self):
        return self.poses()[1]

    def wait(self, seconds):
        """Wait `seconds` of simulated time."""
        end = self.sim_time() + seconds
        limit = time.monotonic() + 60.0 + 20.0 * seconds
        while self.sim_time() < end:
            if time.monotonic() > limit:
                raise RuntimeError('the simulation clock does not advance')
            time.sleep(0.002)

    def spawn(self, state):
        self.remove()
        self.scene.spawn(rm.MODEL_NAME, rm.cube_sdf(self.config, state, fixed_core=True),
                         SPAWN_XYZ, timeout_ms=60000)
        self._listen(True)
        limit = time.monotonic() + 20.0
        while not self.poses()[0]:
            if time.monotonic() > limit:
                raise RuntimeError('no poses of the cube links after the spawn')
            time.sleep(0.01)
        self.wait(0.2)
        # The plugin of a spawned model starts one step after its links: the cubies fall
        # g dt^2 (9.8 um) before they are welded. A free cube falls as one; here the core is
        # fixed, so the whole grid sits that much lower. Measured once, taken out of the drift.
        offsets = self.grid_offsets()
        self.spawn_offset = tuple(sum(o[i] for o in offsets.values()) / len(offsets)
                                  for i in range(3))

    def grid_offsets(self):
        """{home: position of the cubie minus the centre of its slot}, in the core frame."""
        poses, truth = self.poses()[0], self.truth()
        result = {}
        for home, (slot, _) in truth.cubies.items():
            xyz = g.relative(poses[rt.CORE], poses[cb.name_of(home)])[0]
            result[home] = tuple(a - self.pitch * b for a, b in zip(xyz, slot))
        return result

    def remove(self):
        self._listen(False)
        with self._lock:
            self._poses, self._status = {}, {}
        try:
            self.scene.remove(rm.MODEL_NAME)
        except RuntimeError:
            pass                               # not in the world

    def truth(self):
        return rt.read(self.poses()[0], self.pitch)

    def angle(self, face):
        return rt.layer_angle(self.poses()[0], face)

    def _entity(self, face):
        name = cb.name_of(c.NORMAL_OF[face])
        return Entity(name=f'{rm.MODEL_NAME}::{name}', type=Entity.LINK)

    def push(self, face, torque):
        """Persistent torque (N m, about the outward normal of `face`) on its centre cubie.
        Persistent wrenches add up: `release` before another push on the same face."""
        msg = EntityWrench()
        msg.entity.CopyFrom(self._entity(face))
        normal = c.NORMAL_OF[face]
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (
            torque * n for n in normal)
        self._wrench.publish(msg)

    def release(self, face):
        self._clear.publish(self._entity(face))

    def drive(self, face, angle, torque=DRIVE_TORQUE, timeout_s=3.0):
        """Push `face` until it is within STOP_MARGIN of `angle` (rad about its outward
        normal, relative to where it is now), then let go. Returns the angle turned so far,
        or None on a timeout."""
        last, turned = self.angle(face), 0.0
        end = self.sim_time() + timeout_s
        self.push(face, math.copysign(torque, angle))
        try:
            while abs(angle - turned) > STOP_MARGIN:
                if self.sim_time() > end:
                    return None
                time.sleep(0.001)
                now = self.angle(face)
                turned += g.wrap_angle(now - last)
                last = now
        finally:
            self.release(face)
        return turned

    def settle(self, face, timeout_s=2.0):
        """Wait until `face` has stopped; returns its angle in (-pi, pi]."""
        end = self.sim_time() + timeout_s
        last = self.angle(face)
        while self.sim_time() < end:
            self.wait(0.1)
            now = self.angle(face)
            if abs(g.wrap_angle(now - last)) < 1e-6:
                return now
            last = now
        return last

    def turn(self, move):
        """One move on the bench. Returns a record with what the simulator shows after it."""
        face, turns = c.parse_move(move)
        started = self.sim_time()
        turned = self.drive(face, TURN_ANGLE[turns])
        self.settle(face)
        truth = self.truth()
        status = self.status()
        return {'move': move, 'drive_completed': turned is not None, 'state': truth.state,
                'error': truth.error, 'position_error_m': truth.position_error,
                'angle_error_rad': truth.angle_error, 'sim_s': self.sim_time() - started,
                'axis': status.get('axis'), 'switches': status.get('switches')}


def check_turn(bench, state, move, records, part):
    """Turn, compare with the logical model, log. Returns (new expected state, ok)."""
    expected = c.apply_move(state, move)
    record = bench.turn(move)
    record.update(part=part, expected=expected, success=record['state'] == expected)
    records.append(record)
    return expected, record['success']


def part_faces(bench, state, records):
    ok = True
    for face in c.FACES:
        for suffix in ('', '2', "'"):
            state, good = check_turn(bench, state, face + suffix, records, 'faces')
            ok = ok and good
    return state, ok


def part_below(bench, state, records, mech):
    """80 % of the breakaway torque for a second: nothing may turn, whatever the axis."""
    ok = True
    for face in ('U', 'R', 'F'):
        before, status = bench.angle(face), bench.status()
        bench.push(face, 0.8 * mech['breakaway_torque'])
        bench.wait(1.0)
        bench.release(face)
        bench.wait(0.2)
        moved = abs(g.wrap_angle(bench.angle(face) - before))
        truth = bench.truth()
        good = moved < 0.01 and truth.state == state
        records.append({'part': 'below', 'face': face, 'axis_before': status.get('axis'),
                        'moved_rad': moved, 'state': truth.state, 'expected': state,
                        'success': good})
        ok = ok and good
    return state, ok


def part_locking(bench, state, records, mech):
    """U left 30 degrees out of square: a strong torque on R, then on F, turns nothing; once
    U is square again R turns."""
    ok = True
    reached = bench.drive('U', math.radians(30.0) + STOP_MARGIN)
    bench.wait(0.3)
    u_angle = bench.angle('U')
    for face in ('R', 'F'):
        before = bench.angle(face)
        bench.push(face, 5.0 * mech['breakaway_torque'])
        bench.wait(1.0)
        blocked = bench.status().get('blocked')
        bench.release(face)
        bench.wait(0.2)
        moved = abs(g.wrap_angle(bench.angle(face) - before))
        u_moved = abs(g.wrap_angle(bench.angle('U') - u_angle))
        good = reached is not None and moved < 1e-3 and u_moved < 1e-3 and bool(blocked)
        records.append({'part': 'locking', 'face': face, 'u_angle_rad': u_angle,
                        'moved_rad': moved, 'u_moved_rad': u_moved, 'blocked': blocked,
                        'success': good})
        ok = ok and good
    # back to square, then the other axis is free again
    out_of_square = bench.angle('U') - round(bench.angle('U') / (math.pi / 2)) * math.pi / 2
    bench.drive('U', -out_of_square)
    bench.settle('U')
    truth = bench.truth()
    good = truth.state == state
    records.append({'part': 'locking', 'face': 'U', 'back_to_square': good,
                    'state': truth.state, 'expected': state, 'success': good})
    ok = ok and good
    state, good = check_turn(bench, state, 'R', records, 'locking')
    return state, ok and good


def part_drift(bench, state, records, rng, turns):
    ok = True
    for move in c.random_scramble(rng, turns):
        state, good = check_turn(bench, state, move, records, 'drift')
        ok = ok and good
        if not good:
            break
    # a last turn on another axis squares the layers that were free
    last_face = records[-1]['move'][0]
    other = next(f for f in 'URF' if c.NORMAL_OF[f] != c.NORMAL_OF[last_face]
                 and c.NORMAL_OF[f] != tuple(-x for x in c.NORMAL_OF[last_face]))
    state, good = check_turn(bench, state, other, records, 'drift')
    truth = bench.truth()
    # Cubies of the middle slab of the free axis are welded to the core: their distance from
    # the grid is what the welds hold. Each cubie carries the offset it was spawned with
    # (`Bench.spawn`), turned with the layers it has moved in; anything beyond that is error
    # the welds accumulated over the turns.
    truth, axis = bench.truth(), records[-1]['axis']
    offsets = bench.grid_offsets()
    welded = [home for home, (slot, _) in truth.cubies.items() if slot[axis] == 0]
    spawn = math.hypot(*bench.spawn_offset)
    carried = [tuple(spawn * sign * (i == k) for i in range(3))
               for k in range(3) for sign in (1, -1)]
    drift = max(min(math.dist(offsets[home], vector) for vector in carried) for home in welded)
    records.append({'part': 'drift', 'core_welded_cubies': len(welded),
                    'core_welded_drift_m': drift, 'spawn_offset_m': spawn,
                    'success': drift < 1e-6})
    return state, ok and good and drift < 1e-6


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, default=7000)
    parser.add_argument('--turns', type=int, default=400, help='random turns of the drift part')
    parser.add_argument('--parts', default='faces,below,locking,drift')
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    share = Path(get_package_share_directory('openfactory_description')) / 'config'
    config_path = share / 'rubik.yaml'
    config = yaml.safe_load(config_path.read_text())
    mech = config['mechanism']
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    run_id = f'phase7_mechanism_{stamp}_s{args.seed}'
    run_dir = args.out / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    rng = random.Random(args.seed)
    state = c.apply_moves(c.SOLVED, c.random_scramble(rng))
    bench = Bench(config)
    bench.spawn(state)
    first = bench.truth()
    if first.state != state:
        print(f'the spawned cube does not read as its state: {first.error}')
        return 1

    records, results = [], {}
    parts = {
        'faces': lambda s: part_faces(bench, s, records),
        'below': lambda s: part_below(bench, s, records, mech),
        'locking': lambda s: part_locking(bench, s, records, mech),
        'drift': lambda s: part_drift(bench, s, records, rng, args.turns),
    }
    wall = time.monotonic()
    for name in args.parts.split(','):
        state, results[name] = parts[name](state)
        done = [r for r in records if r['part'] == name]
        print(f'{name}: {"pass" if results[name] else "FAIL"} '
              f'({sum(r["success"] for r in done)}/{len(done)} checks)', flush=True)
    with (run_dir / 'turns.jsonl').open('w') as out:
        for record in records:
            out.write(json.dumps(record) + '\n')
    turns = [r for r in records if 'move' in r]
    summary = {
        'run_id': run_id, 'git_commit': tl.git_commit(REPO), 'seed': args.seed,
        'config': tl.config_ref(config_path), 'parts': results,
        'turns': len(turns), 'turns_correct': sum(r['success'] for r in turns),
        'axis_switches': bench.status().get('switches'),
        'max_position_error_m': max((r['position_error_m'] for r in turns), default=None),
        'max_angle_error_rad': max((r['angle_error_rad'] for r in turns), default=None),
        'mean_turn_sim_s': sum(r['sim_s'] for r in turns) / max(1, len(turns)),
        'core_welded_drift_m': next(
            (r['core_welded_drift_m'] for r in records if 'core_welded_drift_m' in r), None),
        'wall_s': time.monotonic() - wall,
    }
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    bench.remove()
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
