"""Controllers and bridge of the ring of N arms (phase 6, D-020), generated for one N from the
files of the single-arm cell: one tuning for every arm, one mapping for every camera. ROS-free.
"""

import copy
from pathlib import Path

from openfactory_description import ring

PACKAGE = 'openfactory_simulation'
TRAJECTORY_CONTROLLER, GRIPPER_CONTROLLER = 'joint_trajectory_controller', 'gripper_controller'
CAMERA_PREFIX = '/camera/'


def controllers(n, cell_controllers):
    """ros2_controllers parameters of a ring of `n` arms from those of the cell
    (config/ros2_controllers.yaml): one joint state broadcaster for every joint, and for each
    arm the cell's trajectory and gripper controllers on that arm's joints."""
    manager = cell_controllers['controller_manager']['ros__parameters']
    trajectory = cell_controllers[TRAJECTORY_CONTROLLER]['ros__parameters']
    gripper = cell_controllers[GRIPPER_CONTROLLER]['ros__parameters']
    if tuple(trajectory['joints']) != ring.ARM_JOINTS or gripper['joint'] != ring.FINGER_JOINTS[0]:
        raise ValueError('unexpected joints in the controllers of the cell')

    out_manager = {key: copy.deepcopy(value) for key, value in manager.items()
                   if key not in (TRAJECTORY_CONTROLLER, GRIPPER_CONTROLLER)}
    out = {'controller_manager': {'ros__parameters': out_manager}}
    for index in range(ring.check_n(n)):
        prefix = ring.arm_prefix(index)
        out_manager[ring.trajectory_controller(index)] = copy.deepcopy(
            manager[TRAJECTORY_CONTROLLER])
        out_manager[ring.gripper_controller(index)] = copy.deepcopy(manager[GRIPPER_CONTROLLER])

        arm = copy.deepcopy(trajectory)
        arm['joints'] = ring.arm_joints(index)
        # per-joint tolerances are keyed by the joint name
        arm['constraints'] = {
            (prefix + key if key in ring.ARM_JOINTS else key): value
            for key, value in trajectory['constraints'].items()}
        out[ring.trajectory_controller(index)] = {'ros__parameters': arm}

        fingers = copy.deepcopy(gripper)
        fingers['joint'] = prefix + gripper['joint']
        out[ring.gripper_controller(index)] = {'ros__parameters': fingers}
    return out


def controller_names(n):
    """Every controller of the ring, the joint state broadcaster first."""
    names = ['joint_state_broadcaster']
    for index in range(ring.check_n(n)):
        names += [ring.trajectory_controller(index), ring.gripper_controller(index)]
    return names


def bridge(n, cell_bridge):
    """ros_gz_bridge mappings of a ring of `n` arms from those of the cell (config/bridge.yaml):
    the camera topics once per bin, in the namespace of the bin, everything else as it is."""
    out = []
    for entry in cell_bridge:
        if not entry['ros_topic_name'].startswith(CAMERA_PREFIX):
            out.append(dict(entry))
            continue
        if not entry['gz_topic_name'].startswith(CAMERA_PREFIX):
            raise ValueError(f"unexpected Gazebo camera topic {entry['gz_topic_name']}")
        for index in range(ring.check_n(n)):
            out.append({**entry,
                        'ros_topic_name': ring.bin_topic(index, entry['ros_topic_name']),
                        'gz_topic_name': ring.bin_topic(index, entry['gz_topic_name'])})
    return out


def write(n, directory=None):
    """Write both files for a ring of `n` arms from the installed files of the cell; returns
    (controllers path, bridge path)."""
    import yaml
    from ament_index_python.packages import get_package_share_directory

    config = Path(get_package_share_directory(PACKAGE)) / 'config'
    directory = Path(directory or ring.generated_dir(n))
    controllers_path = directory / 'ros2_controllers.yaml'
    controllers_path.write_text(ring.dump_yaml(
        controllers(n, yaml.safe_load((config / 'ros2_controllers.yaml').read_text()))))
    bridge_path = directory / 'bridge.yaml'
    bridge_path.write_text(ring.dump_yaml(
        bridge(n, yaml.safe_load((config / 'bridge.yaml').read_text()))))
    return controllers_path, bridge_path
