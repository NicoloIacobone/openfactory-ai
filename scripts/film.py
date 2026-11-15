#!/usr/bin/env python3
"""Film a real run of a system from a camera created in the running Gazebo world (D-029).

The film camera is a model with no body (film_shots.camera_sdf), standing or going round
the point it looks at (an orbit shot): it is spawned for the take and removed after it, it
is not bridged to ROS and nothing in perception or manipulation reads it. Gazebo's
CameraVideoRecorder system encodes its frames on simulated time, so the video plays at the
speed of the simulated system however slowly the VM ran it.

Takes are in scripts/config/film.yaml. Run in the VM:
    python3 scripts/film.py still TAKE [--position X Y Z] [--look-at X Y Z] [--hfov DEG]
                                       [--angle DEG]
        one frame of the shot from the running system, to frame it; --angle: from that far
        round its orbit
    python3 scripts/film.py take TAKE [--name NAME] [--set KEY=VALUE ...] [--fixed]
        the whole take: launches the system without a window, records, runs the scenario,
        stops. It refuses to start while something else runs in the VM, unless --replace.
        Detached, as any benchmark:
        scripts/vm/run_benchmark.sh NAME --script film.py take TAKE --name NAME
    python3 scripts/film.py start TAKE [--name NAME]   /   python3 scripts/film.py stop TAKE
        record a system that is already running
Output: ~/openfactory_data/videos/NAME/NAME.mp4 and take.json (seed, config, commit, simulated
times, exit codes of the scenario). scripts/film_edit.py, on the Mac, copies and edits it.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.video_record_pb2 import VideoRecord
from gz.msgs10.world_stats_pb2 import WorldStatistics

import film_shots as fs
import trial_log as tl
from gz_scene import GzScene

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / 'scripts' / 'config' / 'film.yaml'
OUT = Path.home() / 'openfactory_data' / 'videos'
LAUNCH_LOG = 'film_cell'            # ~/film_cell.log, the launch of a take
SERVICE_TIMEOUT_MS = 10000
STILL_SKIP_FRAMES = 3               # the first frames may come before the scene is complete


class Film:
    """The film camera of one shot in the running world."""

    def __init__(self, shot_name, video, shot):
        self.name, self.video, self.shot = shot_name, video, shot
        self.scene = GzScene()
        self.node = self.scene.node

    def spawn(self):
        xyz, rpy = fs.spawn_pose(self.shot)
        self.scene.spawn(fs.model_name(self.name), fs.camera_sdf(self.name, self.video, self.shot),
                         xyz, rpy, timeout_ms=SERVICE_TIMEOUT_MS)

    def remove(self):
        self.scene.remove(fs.model_name(self.name))

    def _record(self, request):
        ok, reply = self.node.request(
            fs.record_service(self.name), request, VideoRecord, Boolean, SERVICE_TIMEOUT_MS)
        if not ok or not reply.data:
            raise RuntimeError(f'{fs.record_service(self.name)} failed: {request}'.strip())

    def start(self, video_file):
        self._record(VideoRecord(start=True, format=fs.VIDEO_FORMAT,
                                 save_filename=str(video_file)))

    def stop(self):
        self._record(VideoRecord(stop=True))

    def _one(self, message_type, topic, timeout_s, skip=0):
        """One message of a topic. The subscription is dropped before returning: holding one
        while calling a service blocks Gazebo Transport from Python (docs/environment.md)."""
        received, box, count = threading.Event(), [], [0]

        def on_message(msg):
            count[0] += 1
            if count[0] > skip and not received.is_set():
                box.append(msg)
                received.set()

        if not self.node.subscribe(message_type, topic, on_message):
            raise RuntimeError(f'cannot subscribe to {topic}')
        try:
            if not received.wait(timeout_s):
                raise RuntimeError(f'no message on {topic} (is the system launched?)')
        finally:
            self.node.unsubscribe(topic)
        return box[0]

    def sim_time(self):
        stats = self._one(WorldStatistics, f'/world/{self.scene.world}/stats', 5.0)
        return stats.sim_time.sec + stats.sim_time.nsec * 1e-9

    def still(self, path, timeout_s=60.0):
        msg = self._one(Image, fs.image_topic(self.name), timeout_s, skip=STILL_SKIP_FRAMES)
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError(f'cannot write {path}')


def wait_for_file(path, timeout_s=60.0):
    """The recorder moves its file into place a moment after `stop`: wait until it is whole."""
    deadline, size = time.monotonic() + timeout_s, -1
    while time.monotonic() < deadline:
        time.sleep(1.0)
        now = path.stat().st_size if path.exists() else -1
        if now > 0 and now == size:
            return now
        size = now
    raise RuntimeError(f'{path} was not written')


def scenario_env(take):
    env = dict(os.environ)
    path_file = take.get('pythonpath_file')
    if path_file and Path(path_file).expanduser().exists():
        extra = Path(path_file).expanduser().read_text().strip()
        env['PYTHONPATH'] = f"{extra}:{env.get('PYTHONPATH', '')}"
    return env


def stop_cell():
    subprocess.run([str(REPO / 'scripts' / 'vm' / 'stop_cell.sh')], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def running():
    """What a take would stop (film_shots.busy), this process and its ancestors left out."""
    parents = dict(
        map(int, line.split()) for line in subprocess.run(
            ['ps', '-eo', 'pid=,ppid='], capture_output=True, text=True).stdout.splitlines())
    own, pid = set(), os.getpid()
    while pid in parents and pid not in own:        # this process and what started it: its
        own.add(pid)                                # runner, a chain of takes
        pid = parents[pid]
    out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
    return fs.busy(out.splitlines()[1:], own_pids=own)


def launch(arguments):
    """The system of a take, detached, without a window. From the home directory: the share
    is read-only for the VM and Gazebo writes there while it records."""
    subprocess.Popen(
        ['setsid', 'nohup', str(REPO / 'scripts' / 'vm' / 'launch.sh'), LAUNCH_LOG, *arguments],
        cwd=Path.home(), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)


def say(text):
    print(text, flush=True)


def run_take(config, take_name, name, overrides, replace=False, fixed=False):
    take, video = config['takes'][take_name], config['video']
    shot = {key: value for key, value in take['shot'].items() if not (fixed and key == 'orbit')}
    params = fs.take_params(take, overrides)
    commands = fs.scenario(take, overrides)
    commit = tl.git_commit(REPO)        # of the code that runs, not of what HEAD is at the end
    busy = running()
    if busy and not replace:
        say('something is running in the VM and a take would stop it (--replace to do so):\n  '
            + '\n  '.join(line[:110] for line in busy))
        return 2
    out_dir = OUT / name
    out_dir.mkdir(parents=True, exist_ok=True)
    video_file = out_dir / f'{name}.{fs.VIDEO_FORMAT}'
    video_file.unlink(missing_ok=True)

    stop_cell()
    arguments = fs.launch_arguments(take, overrides)
    say(f"take {take_name} -> {video_file}\nlaunch: ros2 launch {' '.join(arguments)}")
    launch(arguments)
    time.sleep(take['warmup_s'])

    film = Film(take_name, video, shot)
    results, sim_start, sim_end, error = [], None, None, None
    try:
        film.spawn()
        film.start(video_file)
        sim_start = film.sim_time()
        say(f'recording from simulated time {sim_start:.1f} s'
            + (f", orbit of {fs.orbit_rate(shot) and 2 * math.pi / abs(fs.orbit_rate(shot)):.0f} s"
               if fs.orbit_rate(shot) else ''))
        time.sleep(config.get('lead_in_s', 0.0))
        env = scenario_env(take)
        for command in commands:
            say(f'scenario: {command}')
            code = subprocess.run(['bash', '-c', command], cwd=REPO, env=env,
                                  stdin=subprocess.DEVNULL).returncode
            say(f'scenario: exit {code}')
            results.append((command, code))
            if code != 0:
                break
        time.sleep(config.get('lead_out_s', 0.0))
        sim_end = film.sim_time()
        film.stop()
        size = wait_for_file(video_file)
        say(f'recorded {sim_end - sim_start:.1f} s of simulated time, {size / 1e6:.1f} MB')
    except RuntimeError as exc:
        error = str(exc)
        say(f'error: {error}')
    finally:
        stop_cell()

    record = fs.take_record(name, take_name, params, video, shot, commit,
                            sim_start, sim_end, results, video_file.name)
    if error:
        record.update(success=False, error=error)
    (out_dir / 'take.json').write_text(json.dumps(record, indent=2) + '\n')
    say(json.dumps(record, indent=2))
    return 0 if record['success'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=CONFIG)
    sub = parser.add_subparsers(dest='command', required=True)
    still = sub.add_parser('still', help='one frame of a shot from the running system')
    still.add_argument('take')
    still.add_argument('--position', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    still.add_argument('--look-at', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    still.add_argument('--hfov', type=float, help='degrees')
    still.add_argument('--angle', type=float, default=0.0,
                       help='degrees round the orbit of the shot, to see what it will pass')
    still.add_argument('--out', type=Path, help='PNG file (default: videos/stills/TAKE.png)')
    whole = sub.add_parser('take', help='launch, record, run the scenario, stop')
    whole.add_argument('take')
    whole.add_argument('--name', help='name of the output (default: the take)')
    whole.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                       help='override a parameter of the take')
    whole.add_argument('--replace', action='store_true',
                       help='stop whatever is running in the VM first')
    whole.add_argument('--fixed', action='store_true',
                       help='keep the camera where the shot starts, without its orbit')
    start = sub.add_parser('start', help='start recording a running system')
    start.add_argument('take')
    start.add_argument('--name', help='name of the output (default: the take)')
    stop = sub.add_parser('stop', help='stop the recording started by `start`')
    stop.add_argument('take')
    args = parser.parse_args()

    config = fs.check_config(yaml.safe_load(args.config.read_text()))
    if args.take not in config['takes']:
        parser.error(f"unknown take {args.take!r}: {sorted(config['takes'])}")
    take, video = config['takes'][args.take], config['video']

    if args.command == 'take':
        return run_take(config, args.take, args.name or args.take,
                        fs.parse_overrides(args.set), args.replace, args.fixed)

    shot = dict(take['shot'])
    if args.command == 'still':
        shot.update({key: value for key, value in (
            ('position', args.position), ('look_at', args.look_at), ('hfov_deg', args.hfov))
            if value is not None})
        # a still is always taken from a camera that stands, at an angle of the orbit
        shot['position'] = fs.orbit_position(shot, math.radians(args.angle))
        shot.pop('orbit', None)
    film = Film(args.take, video, shot)
    if args.command == 'still':
        path = args.out or OUT / 'stills' / f'{args.take}.png'
        film.spawn()
        try:
            film.still(path)
        finally:
            film.remove()
        print(f"{path}: position {[round(v, 3) for v in shot['position']]} "
              f"look_at {shot['look_at']}")
    elif args.command == 'start':
        name = args.name or args.take
        video_file = OUT / name / f'{name}.{fs.VIDEO_FORMAT}'
        video_file.parent.mkdir(parents=True, exist_ok=True)
        film.spawn()
        film.start(video_file)
        print(f'recording {video_file} from simulated time {film.sim_time():.1f} s')
    else:
        end = film.sim_time()
        film.stop()
        time.sleep(3.0)
        film.remove()
        print(f'stopped at simulated time {end:.1f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
