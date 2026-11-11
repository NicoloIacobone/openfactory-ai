#!/usr/bin/env python3
"""Depth sensor model node: ideal Gazebo depth -> depth as the modelled camera would report it.

Subscribes the bridged ideal depth image and camera_info, publishes the measured depth image
with the same header (stamp and optical frame). Model and limits: openfactory_simulation
.depth_noise; parameters: openfactory_description/config/camera.yaml `depth`.

Parameters: `noise` (false: ideal depth, range limits only; can be changed while running with
`ros2 param set /depth_sensor_model noise false`), `seed`.
"""

import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from openfactory_simulation.depth_noise import apply_depth_model, frame_rng

IDEAL_TOPIC = '/camera/depth/image_ideal'
MEASURED_TOPIC = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera_info'
DESCRIPTION_PACKAGE = 'openfactory_description'
ENCODING = '32FC1'


class DepthSensorModel(Node):

    def __init__(self):
        super().__init__('depth_sensor_model')
        noise = self.declare_parameter('noise', True).value
        self.seed = self.declare_parameter('seed', 1).value
        camera_file = Path(get_package_share_directory(DESCRIPTION_PACKAGE)) / 'config' / 'camera.yaml'
        self.params = yaml.safe_load(camera_file.read_text())['depth']
        self.fx = None
        self.publisher = self.create_publisher(Image, MEASURED_TOPIC, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._on_info, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, IDEAL_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f"depth model: noise {'on' if noise else 'off'}, seed {self.seed}, {self.params}")

    def _on_info(self, msg):
        self.fx = msg.k[0]

    def _on_depth(self, msg):
        if self.fx is None:
            return                                  # no intrinsics yet
        if msg.encoding != ENCODING:
            self.get_logger().error(f'unexpected depth encoding {msg.encoding}', once=True)
            return
        dtype = np.dtype(np.float32).newbyteorder('>' if msg.is_bigendian else '<')
        rows = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 4)
        rng = (frame_rng(self.seed, msg.header.stamp.sec, msg.header.stamp.nanosec)
               if self.get_parameter('noise').value else None)
        measured = apply_depth_model(rows[:, :msg.width], self.fx, self.params, rng)
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding=ENCODING,
                    is_bigendian=sys.byteorder == 'big', step=msg.width * 4)
        out.data = measured.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = DepthSensorModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
