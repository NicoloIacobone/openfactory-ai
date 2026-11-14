"""Filming tool (scripts/film_shots.py): pure logic."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

import film_shots as fs
import scene_objects as so

REPO = Path(__file__).resolve().parents[1]
VIDEO = {'width': 1280, 'height': 720, 'fps': 15, 'bitrate': 4000000, 'hfov_deg': 50.0,
         'clip': [0.05, 20.0]}


def _take(**overrides):
    take = {
        'launch': ['openfactory_bringup', 'ring.launch.py', 'n:={arms}', 'rviz:=false'],
        'warmup_s': 90,
        'shot': {'position': [1.5, -1.5, 1.6], 'look_at': [0.0, 0.0, 0.8]},
        'params': {'arms': 3, 'handovers': 12},
        'scenario': ['python3 scripts/relay_benchmark.py --arms {arms} --handovers {handovers}'],
    }
    take.update(overrides)
    return take


def _view_axis(position, target):
    return so.quat_rotate(so.quaternion_from_rpy(*fs.look_at(position, target)), (1.0, 0.0, 0.0))


@pytest.mark.parametrize('position, target', [
    ((1.5, -1.5, 1.6), (0.0, 0.0, 0.8)),
    ((-0.4, 0.9, 0.3), (0.5, 0.0, 0.83)),
    ((0.5, 0.0, 2.0), (0.5, 0.0, 0.8)),      # straight down
])
def test_the_camera_x_axis_points_at_the_target(position, target):
    direction = [t - p for t, p in zip(target, position)]
    norm = math.sqrt(sum(v * v for v in direction))
    assert _view_axis(position, target) == pytest.approx([v / norm for v in direction], abs=1e-9)


def test_the_image_stays_level():
    roll, pitch, yaw = fs.look_at((1.5, -1.5, 1.6), (0.0, 0.0, 0.8))
    assert roll == 0.0
    assert pitch > 0.0                      # looking down
    assert yaw == pytest.approx(3 * math.pi / 4)


def test_a_shot_needs_a_target_away_from_its_position():
    with pytest.raises(ValueError):
        fs.look_at((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))


def test_camera_sdf_is_a_bodiless_static_model_that_records_on_sim_time():
    model = ET.fromstring(fs.camera_sdf('wide', VIDEO)).find('model')
    assert model.get('name') == 'film_camera_wide'
    assert model.findtext('static') == 'true'
    link = model.find('link')
    assert link.find('visual') is None and link.find('collision') is None
    sensor = link.find('sensor')
    assert sensor.get('type') == 'camera'
    assert sensor.findtext('topic') == 'film/wide/image'
    assert float(sensor.findtext('update_rate')) == 15
    assert sensor.findtext('camera/image/width') == '1280'
    assert sensor.findtext('camera/image/height') == '720'
    assert float(sensor.findtext('camera/horizontal_fov')) == pytest.approx(math.radians(50.0))
    plugin = sensor.find('plugin')
    assert plugin.get('filename') == 'gz-sim-camera-video-recorder-system'
    assert plugin.findtext('service') == fs.record_service('wide') == '/film/wide/record_video'
    assert plugin.findtext('use_sim_time') == 'true'
    assert plugin.findtext('fps') == '15'
    assert plugin.findtext('bitrate') == '4000000'


def test_a_shot_may_have_its_own_field_of_view():
    sensor = ET.fromstring(fs.camera_sdf('close', VIDEO, hfov_deg=30.0)).find('model/link/sensor')
    assert float(sensor.findtext('camera/horizontal_fov')) == pytest.approx(math.radians(30.0))


def test_the_config_of_the_repo_is_valid():
    config = yaml.safe_load((REPO / 'scripts' / 'config' / 'film.yaml').read_text())
    fs.check_config(config)
    assert {'rubik', 'ring'} <= set(config['takes'])


@pytest.mark.parametrize('config, message', [
    ({'video': {**VIDEO, 'width': 1281}, 'takes': {'a': _take()}}, 'even'),
    ({'video': {k: v for k, v in VIDEO.items() if k != 'fps'}, 'takes': {'a': _take()}}, 'fps'),
    ({'video': VIDEO, 'takes': {}}, 'no takes'),
    ({'video': VIDEO, 'takes': {'a': _take(scenario=[])}}, 'empty scenario'),
    ({'video': VIDEO, 'takes': {'a': _take(shot={'position': [0, 0, 1]})}}, 'look_at'),
    ({'video': VIDEO, 'takes': {'a': {'launch': [], 'shot': {}}}}, 'missing'),
])
def test_bad_configs_are_rejected(config, message):
    with pytest.raises(ValueError, match=message):
        fs.check_config(config)


def test_a_scenario_that_names_an_unknown_parameter_is_rejected():
    with pytest.raises(KeyError):
        fs.check_config({'video': VIDEO, 'takes': {'a': _take(scenario=['run --seed {seed}'])}})


def test_parameters_fill_the_launch_and_the_scenario():
    take = _take()
    assert fs.launch_arguments(take) == [
        'openfactory_bringup', 'ring.launch.py', 'n:=3', 'rviz:=false']
    overrides = fs.parse_overrides(['arms=2', 'handovers=6'])
    assert fs.launch_arguments(take, overrides)[2] == 'n:=2'
    assert fs.scenario(take, overrides) == [
        'python3 scripts/relay_benchmark.py --arms 2 --handovers 6']


def test_overrides_must_be_parameters_of_the_take():
    with pytest.raises(ValueError, match='seed'):
        fs.scenario(_take(), {'seed': '4'})
    with pytest.raises(ValueError):
        fs.parse_overrides(['seed'])


def test_a_take_succeeds_only_if_every_command_of_its_scenario_did():
    def record(results):
        return fs.take_record('n', 'ring', {}, VIDEO, {}, 'abc', 10.0, 70.5, results, 'n.mp4')

    assert record([('a', 0), ('b', 0)])['success'] is True
    assert record([('a', 0), ('b', 2)])['success'] is False
    assert record([])['success'] is False
    assert record([('a', 0)])['sim_duration_s'] == 60.5


def test_the_speed_is_in_the_name_of_an_edit():
    assert fs.edit_name('rubik', 8) == 'rubik_x8.mp4'
    assert fs.edit_name('ring', 1.5, trim=(10, 70), extension='gif') == 'ring_x1.5_10-70s.gif'


def test_ffmpeg_speeds_up_and_writes_h264():
    args = fs.ffmpeg_args('in.mp4', 'out.mp4', speed=8)
    assert args[0] == 'ffmpeg' and args[-1] == 'out.mp4'
    assert args[args.index('-vf') + 1] == 'setpts=PTS/8,fps=30'
    assert args[args.index('-pix_fmt') + 1] == 'yuv420p'
    assert '-ss' not in args


def test_ffmpeg_trims_the_source_before_reading_it():
    args = fs.ffmpeg_args('in.mp4', 'out.mp4', speed=2, trim=(12.5, 40))
    assert args.index('-ss') < args.index('-i')
    assert args[args.index('-ss') + 1] == '12.5' and args[args.index('-to') + 1] == '40'


def test_ffmpeg_writes_a_gif_with_its_own_palette():
    args = fs.ffmpeg_args('in.mp4', 'out.gif', speed=8, fps=12, gif_width=640)
    graph = args[args.index('-filter_complex') + 1]
    assert 'scale=640:-2' in graph and 'palettegen' in graph and 'paletteuse' in graph
    assert '-c:v' not in args


@pytest.mark.parametrize('kwargs', [{'speed': 0}, {'trim': (5, 5)}, {'trim': (-1, 4)}])
def test_ffmpeg_rejects_bad_edits(kwargs):
    with pytest.raises(ValueError):
        fs.ffmpeg_args('in.mp4', 'out.mp4', **kwargs)


def test_a_take_sees_simulators_and_the_scripts_that_drive_them():
    lines = [
        ' 169244 /bin/bash /home/nico/p7_demo.sh',
        ' 174342 sleep 50',
        ' 174370 gz sim -r -v 3 --render-engine ogre /home/nico/cell.sdf',
        ' 174341 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch openfactory_bringup x.launch.py',
        '   1887 /usr/bin/gnome-shell',
    ]
    assert [line.split()[0] for line in fs.busy(lines)] == ['169244', '174370', '174341']


def test_a_take_does_not_mistake_a_shell_that_names_them_or_itself():
    lines = [
        ' 183709 bash -c ps -eo pid,args | grep -E "gz sim|run_benchmark.sh|_demo.sh"',
        ' 174367 /bin/sh -c ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -r -v 3',
        ' 190001 /bin/bash /mnt/openfactory-ai/scripts/vm/run_benchmark.sh ring --script film.py',
        ' 190002 python3 scripts/film.py take ring',
    ]
    assert fs.busy(lines, own_pids={190001, 190002}) == []
    assert len(fs.busy(lines)) == 1          # the runner of another take does count
