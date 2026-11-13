"""Generated controllers and bridge of the ring against the files of the single-arm cell."""

from pathlib import Path

import pytest
import yaml

from openfactory_description import ring
from openfactory_simulation import ring_config

CONFIG = Path(__file__).resolve().parents[1] / 'config'
CELL_CONTROLLERS = yaml.safe_load((CONFIG / 'ros2_controllers.yaml').read_text())
CELL_BRIDGE = yaml.safe_load((CONFIG / 'bridge.yaml').read_text())


@pytest.mark.parametrize('n', [2, 3, 5])
def test_every_arm_gets_the_controllers_of_the_cell(n):
    out = ring_config.controllers(n, CELL_CONTROLLERS)
    manager = out['controller_manager']['ros__parameters']
    cell_manager = CELL_CONTROLLERS['controller_manager']['ros__parameters']
    assert manager['update_rate'] == cell_manager['update_rate']
    assert manager['use_sim_time'] is True
    assert set(ring_config.controller_names(n)) == {
        key for key, value in manager.items() if isinstance(value, dict)}
    assert set(out) == {'controller_manager', *ring_config.controller_names(n)[1:]}

    claimed = []
    cell_arm = CELL_CONTROLLERS['joint_trajectory_controller']['ros__parameters']
    for index in range(n):
        arm = out[ring.trajectory_controller(index)]['ros__parameters']
        assert arm['joints'] == ring.arm_joints(index)
        assert arm['command_interfaces'] == cell_arm['command_interfaces']
        for cell_joint, joint in zip(ring.ARM_JOINTS, arm['joints']):
            assert arm['constraints'][joint] == cell_arm['constraints'][cell_joint]
            assert cell_joint not in arm['constraints']
        assert arm['constraints']['goal_time'] == cell_arm['constraints']['goal_time']
        fingers = out[ring.gripper_controller(index)]['ros__parameters']
        assert fingers['joint'] == ring.finger_joints(index)[0]
        assert fingers['allow_stalling'] is True
        claimed += arm['joints'] + [fingers['joint']]
    assert len(claimed) == len(set(claimed))


def test_the_cell_files_are_not_modified():
    before = yaml.safe_dump(CELL_CONTROLLERS)
    ring_config.controllers(3, CELL_CONTROLLERS)
    ring_config.bridge(3, CELL_BRIDGE)
    assert yaml.safe_dump(CELL_CONTROLLERS) == before


@pytest.mark.parametrize('n', [2, 4])
def test_one_set_of_camera_topics_per_bin(n):
    out = ring_config.bridge(n, CELL_BRIDGE)
    cameras = [e for e in CELL_BRIDGE if e['ros_topic_name'].startswith('/camera/')]
    assert len(cameras) == 3
    assert len(out) == len(CELL_BRIDGE) - len(cameras) + n * len(cameras)
    assert [e for e in out if e['ros_topic_name'] == '/clock'] == [
        e for e in CELL_BRIDGE if e['ros_topic_name'] == '/clock']
    ros_topics = [e['ros_topic_name'] for e in out]
    gz_topics = [e['gz_topic_name'] for e in out]
    assert len(set(ros_topics)) == len(ros_topics) and len(set(gz_topics)) == len(gz_topics)
    for index in range(n):
        for entry in cameras:
            match, = [e for e in out
                      if e['ros_topic_name'] == f"/bin_{index}{entry['ros_topic_name']}"]
            assert match['gz_topic_name'] == f"/bin_{index}{entry['gz_topic_name']}"
            assert match.get('lazy') == entry.get('lazy')
            assert match['ros_type_name'] == entry['ros_type_name']


def test_gazebo_camera_topics_match_the_layout():
    cell = yaml.safe_load(
        (CONFIG.parents[1] / 'openfactory_description' / 'config' / 'cell.yaml').read_text())
    ring_yaml = yaml.safe_load(
        (CONFIG.parents[1] / 'openfactory_description' / 'config' / 'ring.yaml').read_text())
    layout = ring.layout(3, cell, ring_yaml)
    gz_topics = {e['gz_topic_name'] for e in ring_config.bridge(3, CELL_BRIDGE)}
    for item in layout['bins']:
        # the rgbd_camera sensor publishes <topic>/image, /depth_image and /camera_info
        for suffix in ('image', 'depth_image', 'camera_info'):
            assert f"/{item['camera_topic']}/{suffix}" in gz_topics


def test_written_files_have_no_yaml_aliases(tmp_path):
    # rcl refuses a parameter file with anchors ("Will not support aliasing")
    controllers_path, bridge_path = ring_config.write(3, tmp_path)
    for path in (controllers_path, bridge_path):
        text = path.read_text()
        assert '&id' not in text and '*id' not in text
    written = yaml.safe_load(controllers_path.read_text())
    assert written == ring_config.controllers(3, CELL_CONTROLLERS)
