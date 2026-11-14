"""Pure logic of the filming tool (scripts/film.py, scripts/film_edit.py): no ROS, no Gazebo.

A take films one real run of a system. The film camera is a Gazebo model of its own, static,
with no body and no collision, created in the running world: it is not part of the cell, it is
not bridged to ROS and nothing in perception or manipulation reads it. Its sensor carries
Gazebo's CameraVideoRecorder system, which encodes the frames on simulated time, so a video
plays at the speed of the simulated system however slowly the simulator ran.
"""

import math
import re
from xml.sax.saxutils import escape

MODEL_PREFIX = 'film_camera'
TOPIC_PREFIX = 'film'
VIDEO_FORMAT = 'mp4'
VIDEO_FIELDS = ('width', 'height', 'fps', 'bitrate', 'hfov_deg', 'clip')
TAKE_FIELDS = ('launch', 'warmup_s', 'shot', 'scenario')

# what a take would stop: a simulator, a launch, or a script that drives them
BUSY_MARKS = ('gz sim', 'ros2 launch openfactory_', 'run_rubik.sh', 'run_chain.sh',
              'run_benchmark.sh', '_demo.sh')
SHELL_COMMAND = re.compile(r'^(\S*/)?(ba|da|z)?sh -c ')


def busy(ps_lines, own_pids=()):
    """Lines of `ps -eo pid,args` (header removed) that show a simulator or a script driving
    one. A demo or a benchmark between two launches has no simulator but is still at work, so
    the scripts count too. A `sh -c "..."` is skipped: it is either the wrapper of a process
    that is listed anyway, or a shell whose command text only mentions these names (the
    remote shell of an SSH session that greps for them)."""
    found = []
    for line in ps_lines:
        pid, _, args = line.strip().partition(' ')
        args = args.strip()
        if not pid.isdigit() or int(pid) in own_pids or SHELL_COMMAND.match(args):
            continue
        if any(mark in args for mark in BUSY_MARKS):
            found.append(f'{pid} {args}')
    return found


def look_at(position, target):
    """(roll, pitch, yaw) of a camera link at `position` whose x axis points at `target`, image
    up as close to world z as it can be (R = Rz Ry Rx, the SDF convention)."""
    dx, dy, dz = (t - p for t, p in zip(target, position))
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance < 1e-9:
        raise ValueError('a shot needs a target away from its position')
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    return 0.0, -math.atan2(dz, horizontal), yaw


def model_name(shot_name):
    return f'{MODEL_PREFIX}_{shot_name}'


def image_topic(shot_name):
    return f'{TOPIC_PREFIX}/{shot_name}/image'


def record_service(shot_name):
    return f'/{TOPIC_PREFIX}/{shot_name}/record_video'


def camera_sdf(shot_name, video, hfov_deg=None):
    """SDF of the film camera of a shot: a static model with one camera sensor that looks along
    the x axis of its link and records itself on request (service `record_service`)."""
    hfov = math.radians(hfov_deg if hfov_deg is not None else video['hfov_deg'])
    near, far = video['clip']
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{escape(model_name(shot_name))}">
    <static>true</static>
    <link name="link">
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{video['fps']}</update_rate>
        <topic>{escape(image_topic(shot_name))}</topic>
        <camera>
          <horizontal_fov>{hfov:.6f}</horizontal_fov>
          <image>
            <width>{video['width']}</width>
            <height>{video['height']}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>{near}</near><far>{far}</far></clip>
        </camera>
        <plugin filename="gz-sim-camera-video-recorder-system"
                name="gz::sim::systems::CameraVideoRecorder">
          <service>{escape(record_service(shot_name))}</service>
          <use_sim_time>true</use_sim_time>
          <fps>{video['fps']}</fps>
          <bitrate>{video['bitrate']}</bitrate>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
'''


def check_config(config):
    """The config of scripts/config/film.yaml, checked; raises ValueError naming what is wrong."""
    video = config.get('video') or {}
    missing = [field for field in VIDEO_FIELDS if field not in video]
    if missing:
        raise ValueError(f'video: missing {missing}')
    for field in ('width', 'height', 'fps', 'bitrate'):
        if not isinstance(video[field], int) or video[field] <= 0:
            raise ValueError(f'video.{field} must be a positive integer')
    if video['width'] % 2 or video['height'] % 2:
        raise ValueError('video.width and video.height must be even (H.264, yuv420p)')
    takes = config.get('takes') or {}
    if not takes:
        raise ValueError('no takes')
    for name, take in takes.items():
        missing = [field for field in TAKE_FIELDS if field not in take]
        if missing:
            raise ValueError(f'take {name}: missing {missing}')
        shot = take['shot']
        for field in ('position', 'look_at'):
            if len(shot.get(field) or ()) != 3:
                raise ValueError(f'take {name}: shot.{field} must be [x, y, z]')
        look_at(shot['position'], shot['look_at'])
        if not take['scenario']:
            raise ValueError(f'take {name}: empty scenario')
        scenario(take)
        launch_arguments(take)
    return config


def parse_overrides(pairs):
    """['seed=12', 'arms=3'] -> {'seed': '12', 'arms': '3'}."""
    overrides = {}
    for pair in pairs:
        key, separator, value = pair.partition('=')
        if not separator or not key:
            raise ValueError(f'expected KEY=VALUE, got {pair!r}')
        overrides[key] = value
    return overrides


def take_params(take, overrides=None):
    params = dict(take.get('params') or {})
    unknown = sorted(set(overrides or {}) - set(params))
    if unknown:
        raise ValueError(f'not a parameter of this take: {unknown} (it has {sorted(params)})')
    params.update(overrides or {})
    return params


def launch_arguments(take, overrides=None):
    """`ros2 launch` arguments of a take, its parameters filled in."""
    params = take_params(take, overrides)
    return [str(argument).format(**params) for argument in take['launch']]


def scenario(take, overrides=None):
    """Shell commands of a take, its parameters filled in."""
    params = take_params(take, overrides)
    return [str(command).format(**params) for command in take['scenario']]


def take_record(name, take_name, params, video, shot, commit, sim_start, sim_end, results,
                video_file):
    """What take.json holds. `results`: [(command, exit code)] of the scenario."""
    return {
        'name': name,
        'take': take_name,
        'git_commit': commit,
        'params': params,
        'video': video,
        'shot': shot,
        'sim_start_s': sim_start,
        'sim_end_s': sim_end,
        'sim_duration_s': None if None in (sim_start, sim_end) else round(sim_end - sim_start, 3),
        'scenario': [{'command': command, 'exit_code': code} for command, code in results],
        'success': bool(results) and all(code == 0 for _, code in results),
        'video_file': video_file,
    }


def edit_name(name, speed, trim=None, extension='mp4'):
    """File name of an edit: the speed is in the name, e.g. `rubik_x8.mp4`."""
    label = f'{name}_x{speed:g}'
    if trim:
        label += f'_{trim[0]:g}-{trim[1]:g}s'
    return f'{label}.{extension}'


def ffmpeg_args(source, destination, speed=1.0, trim=None, fps=30, gif_width=None):
    """ffmpeg arguments that speed a take up (frames are dropped, none blended), optionally cut
    to `trim` = (start, end) seconds of the source. `gif_width`: write a GIF that wide instead
    of an H.264 mp4."""
    if speed <= 0:
        raise ValueError('speed must be positive')
    args = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y']
    if trim:
        start, end = trim
        if not 0 <= start < end:
            raise ValueError('trim must be START < END, in seconds of the source')
        args += ['-ss', f'{start:g}', '-to', f'{end:g}']
    args += ['-i', str(source), '-an']
    filters = [f'setpts=PTS/{speed:g}', f'fps={fps}']
    if gif_width:
        filters.append(f'scale={gif_width}:-2:flags=lanczos')
        chain = ','.join(filters)
        args += ['-filter_complex',
                 f'[0:v]{chain},split[a][b];[a]palettegen=stats_mode=diff[p];'
                 '[b][p]paletteuse=dither=bayer:bayer_scale=4']
    else:
        args += ['-vf', ','.join(filters), '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
                 '-pix_fmt', 'yuv420p', '-movflags', '+faststart']
    return args + [str(destination)]
