"""Generated MoveIt configuration of the ring against the ring URDF and the cell's SRDF."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from openfactory_description import ring
from openfactory_moveit_config import ring_config

DESCRIPTION = Path(get_package_share_directory('openfactory_description'))
OWN = Path(get_package_share_directory('openfactory_moveit_config'))
VENDOR = Path(get_package_share_directory('ur_moveit_config')) / 'config'
CELL_SRDF = xacro.process_file(str(OWN / 'srdf' / 'openfactory_cell.srdf.xacro')).toxml()
MOVING = ['upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link',
          'gripper_palm', 'left_finger', 'right_finger']


@pytest.fixture(scope='module', params=[2, 3])
def built(request, tmp_path_factory):
    """(n, URDF root, SRDF root, set of disabled pairs) for one ring size."""
    n = request.param
    layout_file = ring.write_layout(n, tmp_path_factory.mktemp(f'ring_n{n}'))
    urdf = ET.fromstring(xacro.process_file(
        str(DESCRIPTION / 'urdf' / 'ring.urdf.xacro'),
        mappings={'layout_file': str(layout_file), 'include_fixtures': 'true'}).toxml())
    srdf = ET.fromstring(ring_config.srdf(n, CELL_SRDF))
    disabled = {frozenset((p.get('link1'), p.get('link2')))
                for p in srdf.findall('disable_collisions')}
    return n, urdf, srdf, disabled


def test_srdf_matches_the_ring_urdf(built):
    _, urdf, srdf, _ = built
    links = {link.get('name') for link in urdf.findall('link')}
    joints = {joint.get('name') for joint in urdf.findall('joint')}
    assert srdf.get('name') == urdf.get('name') == ring_config.ROBOT_NAME
    pairs = srdf.findall('disable_collisions')
    assert pairs
    for pair in pairs:
        assert pair.get('link1') in links and pair.get('link2') in links
        assert pair.get('link1') != pair.get('link2')
    assert len({frozenset((p.get('link1'), p.get('link2'))) for p in pairs}) == len(pairs)
    for chain in srdf.findall('group/chain'):
        assert chain.get('base_link') in links and chain.get('tip_link') in links
    for joint in srdf.findall('group_state/joint') + srdf.findall('group/joint'):
        assert joint.get('name') in joints
    for effector in srdf.findall('end_effector'):
        assert effector.get('parent_link') in links


def test_one_arm_group_and_one_gripper_per_arm(built):
    n, _, srdf, _ = built
    groups = {g.get('name'): g for g in srdf.findall('group')}
    assert len(groups) == 2 * n
    cell = ET.fromstring(CELL_SRDF)
    cell_states = {s.get('name') for s in cell.findall('group_state')}
    for index in range(n):
        prefix = ring.arm_prefix(index)
        chain = groups[ring.planning_group(index)].find('chain')
        assert chain.get('base_link') == prefix + 'base_link'
        assert chain.get('tip_link') == prefix + 'tool0'
        fingers = [j.get('name') for j in groups[ring.gripper_group(index)].findall('joint')]
        assert fingers == ring.finger_joints(index)
        states = {s.get('name') for s in srdf.findall('group_state')
                  if s.get('group') in (ring.planning_group(index), ring.gripper_group(index))}
        assert states == cell_states
        up, = [s for s in srdf.findall('group_state')
               if s.get('name') == 'up' and s.get('group') == ring.planning_group(index)]
        assert {j.get('name') for j in up.findall('joint')} == set(ring.arm_joints(index))
    effectors = srdf.findall('end_effector')
    assert len(effectors) == n
    assert {e.get('parent_group') for e in effectors} == {
        ring.planning_group(index) for index in range(n)}


def test_every_arm_keeps_the_collision_matrix_of_the_cell(built):
    n, _, _, disabled = built
    cell = ET.fromstring(CELL_SRDF)
    internal = [(p.get('link1'), p.get('link2')) for p in cell.findall('disable_collisions')
                if not {p.get('link1'), p.get('link2')} & set(ring_config.CELL_FIXTURES)]
    assert len(internal) >= 17                  # 11 of the UR arm + 6 of the gripper
    for index in range(n):
        prefix = ring.arm_prefix(index)
        for a, b in internal:
            assert frozenset((prefix + a, prefix + b)) in disabled


def test_moving_links_collide_with_fixtures_and_with_the_other_arms(built):
    n, urdf, _, disabled = built
    fixtures = ['table', 'floor']
    for index in range(n):
        fixtures += [ring.bin_name(index), ring.camera_prefix(index) + 'camera_link']
    arm_links = {index: [link.get('name') for link in urdf.findall('link')
                         if link.get('name').startswith(ring.arm_prefix(index))]
                 for index in range(n)}
    for index in range(n):
        prefix = ring.arm_prefix(index)
        for link in MOVING:
            for fixture in fixtures:
                assert frozenset((fixture, prefix + link)) not in disabled
            # arm against arm: only the fixed bases are skipped
            for other in range(n):
                if other != index:
                    for other_link in arm_links[other]:
                        assert frozenset((prefix + link, other_link)) not in disabled


def test_fixtures_and_bases_do_not_check_each_other(built):
    n, _, _, disabled = built
    for index in range(n):
        base = ring.arm_prefix(index) + 'base_link_inertia'
        assert frozenset(('table', base)) in disabled
        assert frozenset(('floor', base)) in disabled
        for other in range(n):
            assert frozenset((ring.bin_name(other), base)) in disabled
            assert frozenset((ring.camera_prefix(other) + 'camera_link', base)) in disabled
            assert frozenset(('table', ring.bin_name(other))) in disabled
            if other != index:
                assert frozenset((ring.arm_prefix(other) + 'base_link_inertia', base)) in disabled
                assert frozenset((ring.bin_name(index), ring.bin_name(other))) in disabled


@pytest.mark.parametrize('n', [2, 4])
def test_kinematics_limits_and_controllers(n):
    cell_kinematics = yaml.safe_load((VENDOR / 'kinematics.yaml').read_text())
    kinematics = ring_config.kinematics(n, cell_kinematics)
    assert set(kinematics) == {ring.planning_group(index) for index in range(n)}
    assert all(value == cell_kinematics['ur_manipulator'] for value in kinematics.values())

    cell_limits = yaml.safe_load((VENDOR / 'joint_limits.yaml').read_text())
    limits = ring_config.joint_limits(n, cell_limits)['joint_limits']
    assert set(limits) == {joint for index in range(n) for joint in ring.arm_joints(index)}
    assert limits['arm_1_elbow_joint'] == cell_limits['joint_limits']['elbow_joint']

    cell_controllers = yaml.safe_load((OWN / 'config' / 'moveit_controllers.yaml').read_text())
    controllers = ring_config.moveit_controllers(n, cell_controllers)
    assert controllers['trajectory_execution'] == cell_controllers['trajectory_execution']
    manager = controllers['moveit_simple_controller_manager']
    assert manager['controller_names'] == [ring.trajectory_controller(i) for i in range(n)]
    for index in range(n):
        entry = manager[ring.trajectory_controller(index)]
        assert entry['joints'] == ring.arm_joints(index)
        assert entry['type'] == 'FollowJointTrajectory' and entry['default'] is True


def test_written_files(tmp_path):
    paths = ring_config.write(3, tmp_path)
    assert set(paths) == {'srdf', 'kinematics', 'joint_limits', 'controllers'}
    assert ET.fromstring(paths['srdf'].read_text()).get('name') == ring_config.ROBOT_NAME
    for key in ('kinematics', 'joint_limits', 'controllers'):
        text = paths[key].read_text()
        assert '&id' not in text and '*id' not in text
        assert yaml.safe_load(text)
