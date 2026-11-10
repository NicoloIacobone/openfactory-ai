"""Run one pick-and-place cycle at a known pose and print the result as JSON.

The object must already be in the input bin at that pose (e.g. scripts/gz_scene.py spawn).
    ros2 run openfactory_manipulation pick_place_once --object cube [--x 0 --y 0 --yaw 0]
"""

import argparse
import json
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter

from . import constants as c
from .moveit_interface import ManipulationError
from .pick_place import PickPlace


def load_configs(config_path=None):
    own = Path(get_package_share_directory('openfactory_manipulation')) / 'config'
    description = Path(get_package_share_directory(c.DESCRIPTION_PACKAGE)) / 'config'
    config = yaml.safe_load(Path(config_path or own / 'pick_place.yaml').read_text())
    catalog = yaml.safe_load((description / 'objects.yaml').read_text())
    gripper = yaml.safe_load((description / 'gripper.yaml').read_text())
    return config, catalog, gripper


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--object', required=True, help='catalog type (cube, cylinder, prism)')
    parser.add_argument('--id', help='planning-scene id (default: the object type)')
    parser.add_argument('--x', type=float, default=0.0, help='m, in the input_bin frame')
    parser.add_argument('--y', type=float, default=0.0, help='m, in the input_bin frame')
    parser.add_argument('--yaw', type=float, default=0.0, help='rad, about the input_bin z axis')
    parser.add_argument('--config', type=Path, help='pick_place.yaml override')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])

    config, catalog, gripper = load_configs(args.config)
    rclpy.init(args=argv)
    node = Node('pick_place_once',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        runner = PickPlace(node, config, catalog, gripper)
        runner.wait_ready()
        result = runner.run(args.object, args.id or args.object, args.x, args.y, args.yaw)
    except ManipulationError as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
