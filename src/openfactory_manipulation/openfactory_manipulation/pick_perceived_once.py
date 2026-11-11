"""Run one camera-driven pick-and-place cycle and print the result as JSON.

The object is whatever scene_perception reports in the input bin (the cell must run with the
camera and perception); its pose comes from `/perception/objects`, not from the command line.
    ros2 run openfactory_manipulation pick_perceived_once
"""

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from .camera_pick import perceive
from .moveit_interface import ManipulationError
from .perception_client import PerceptionClient
from .pick_place import PickPlace
from .pick_place_once import load_configs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, help='pick_place.yaml override')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])

    config, catalog, gripper = load_configs(args.config)
    rclpy.init(args=argv)
    node = Node('pick_perceived_once',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        runner = PickPlace(node, config, catalog, gripper)
        runner.wait_ready()
        client = PerceptionClient(node)
        seen = perceive(node, runner, client)
        if not seen['ok']:
            print(json.dumps({'success': False, 'failure_reason': seen['failure_reason'],
                              'perception': seen}, indent=2))
            return 3
        result = runner.run(seen['object_type'], seen['object_type'], seen['x'], seen['y'],
                            seen['yaw'], source='perception')
        result['perception'] = seen
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
