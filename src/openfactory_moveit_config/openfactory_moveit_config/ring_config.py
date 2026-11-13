"""MoveIt configuration of the ring of N arms (phase 6, D-020), generated for one N from the
configuration of the single-arm cell: every arm gets the cell's planning group, named states,
gripper, collision matrix, kinematics solver, joint limits and controller. ROS-free.

One robot model holds every arm, so a plan for one arm is checked against all the others:
pairs of links of different arms stay enabled (only the fixed bases are skipped).
"""

import copy
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

from openfactory_description import ring

PACKAGE = 'openfactory_moveit_config'
ROBOT_NAME = 'openfactory_ring'
CELL_ARM_GROUP, CELL_GRIPPER_GROUP = 'ur_manipulator', 'gripper'
CELL_TRAJECTORY_CONTROLLER = 'joint_trajectory_controller'
# fixtures of the cell: every other link of its SRDF belongs to the arm or the gripper
CELL_BINS, CELL_CAMERA = ('input_bin', 'output_bin'), 'camera_link'
CELL_FIXTURES = (ring.TABLE_LINK, ring.FLOOR_LINK, *CELL_BINS, CELL_CAMERA)
# links of an arm that never move relative to the fixtures
BASE_LINK = 'base_link_inertia'


def _fixture_links(n):
    """(bin links, camera links) of a ring of `n` arms."""
    bins = [ring.bin_name(index) for index in range(n)]
    cameras = [ring.camera_prefix(index) + CELL_CAMERA for index in range(n)]
    return bins, cameras


def srdf(n, cell_srdf):
    """SRDF text of a ring of `n` arms from the SRDF text of the cell."""
    ring.check_n(n)
    cell = ET.fromstring(cell_srdf)
    out = ET.Element('robot', name=ROBOT_NAME)
    bins, cameras = _fixture_links(n)
    pairs = {}                                   # frozenset of two links -> reason

    def disable(a, b, reason):
        pairs.setdefault(frozenset((a, b)), (a, b, reason))

    for index in range(n):
        prefix = ring.arm_prefix(index)
        groups = {CELL_ARM_GROUP: ring.planning_group(index),
                  CELL_GRIPPER_GROUP: ring.gripper_group(index)}
        for group in cell.findall('group'):
            copied = ET.SubElement(out, 'group', name=groups[group.get('name')])
            for chain in group.findall('chain'):
                ET.SubElement(copied, 'chain', base_link=prefix + chain.get('base_link'),
                              tip_link=prefix + chain.get('tip_link'))
            for joint in group.findall('joint'):
                ET.SubElement(copied, 'joint', name=prefix + joint.get('name'))
        for state in cell.findall('group_state'):
            copied = ET.SubElement(out, 'group_state', name=state.get('name'),
                                   group=groups[state.get('group')])
            for joint in state.findall('joint'):
                ET.SubElement(copied, 'joint', name=prefix + joint.get('name'),
                              value=joint.get('value'))
        for effector in cell.findall('end_effector'):
            ET.SubElement(out, 'end_effector', name=groups[effector.get('group')],
                          parent_link=prefix + effector.get('parent_link'),
                          group=groups[effector.get('group')],
                          parent_group=groups[effector.get('parent_group')])

        for pair in cell.findall('disable_collisions'):
            a, b, reason = pair.get('link1'), pair.get('link2'), pair.get('reason')
            fixtures = [link for link in (a, b) if link in CELL_FIXTURES]
            if not fixtures:                     # within the arm and its gripper
                disable(prefix + a, prefix + b, reason)
            elif len(fixtures) == 1:             # the arm against a fixture of the cell
                fixture, = fixtures
                link = prefix + (b if a == fixture else a)
                # what holds for a bin or the camera of the cell holds for every bin and
                # camera of the ring: the links concerned do not move relative to them
                targets = (bins if fixture in CELL_BINS else
                           cameras if fixture == CELL_CAMERA else [fixture])
                for target in targets:
                    disable(target, link, reason)

    # fixtures never move relative to each other
    for a, b in itertools.combinations([ring.FLOOR_LINK, ring.TABLE_LINK, *bins, *cameras], 2):
        disable(a, b, 'Never')
    # neither do the bases of two arms
    for a, b in itertools.combinations(range(n), 2):
        disable(ring.arm_prefix(a) + BASE_LINK, ring.arm_prefix(b) + BASE_LINK, 'Never')

    for a, b, reason in pairs.values():
        ET.SubElement(out, 'disable_collisions', link1=a, link2=b, reason=reason)
    ET.indent(out)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(out, encoding='unicode') + '\n'


def kinematics(n, cell_kinematics):
    """Kinematics solver of every arm group: the one of the cell's group."""
    return {ring.planning_group(index): copy.deepcopy(cell_kinematics[CELL_ARM_GROUP])
            for index in range(ring.check_n(n))}


def joint_limits(n, cell_limits):
    """MoveIt joint limits of every arm: those of the cell's joints."""
    return {'joint_limits': {
        ring.arm_prefix(index) + joint: copy.deepcopy(limits)
        for index in range(ring.check_n(n))
        for joint, limits in cell_limits['joint_limits'].items()}}


def moveit_controllers(n, cell_controllers):
    """Trajectory execution of the ring from the cell's moveit_controllers.yaml: one
    FollowJointTrajectory controller per arm, on that arm's joints."""
    manager = cell_controllers['moveit_simple_controller_manager']
    cell_controller = manager[CELL_TRAJECTORY_CONTROLLER]
    if manager['controller_names'] != [CELL_TRAJECTORY_CONTROLLER]:
        raise ValueError('unexpected controllers in the MoveIt configuration of the cell')
    out = copy.deepcopy(cell_controllers)
    out_manager = {'controller_names': []}
    for index in range(ring.check_n(n)):
        name = ring.trajectory_controller(index)
        out_manager['controller_names'].append(name)
        out_manager[name] = {
            **copy.deepcopy(cell_controller),
            'joints': [ring.arm_prefix(index) + joint for joint in cell_controller['joints']]}
    out['moveit_simple_controller_manager'] = out_manager
    return out


def write(n, directory=None):
    """Write the four files for a ring of `n` arms from the installed configuration of the
    cell and of ur_moveit_config; returns {'srdf', 'kinematics', 'joint_limits',
    'controllers'} -> path."""
    import xacro
    import yaml
    from ament_index_python.packages import get_package_share_directory

    own = Path(get_package_share_directory(PACKAGE))
    vendor = Path(get_package_share_directory('ur_moveit_config')) / 'config'
    directory = Path(directory or ring.generated_dir(n))
    cell_srdf = xacro.process_file(str(own / 'srdf' / 'openfactory_cell.srdf.xacro')).toxml()

    def load(path):
        return yaml.safe_load(path.read_text())

    contents = {
        'srdf': ('ring.srdf', srdf(n, cell_srdf)),
        'kinematics': ('kinematics.yaml', ring.dump_yaml(
            kinematics(n, load(vendor / 'kinematics.yaml')))),
        'joint_limits': ('joint_limits.yaml', ring.dump_yaml(
            joint_limits(n, load(vendor / 'joint_limits.yaml')))),
        'controllers': ('moveit_controllers.yaml', ring.dump_yaml(
            moveit_controllers(n, load(own / 'config' / 'moveit_controllers.yaml')))),
    }
    paths = {}
    for key, (file_name, text) in contents.items():
        paths[key] = directory / file_name
        paths[key].write_text(text)
    return paths
