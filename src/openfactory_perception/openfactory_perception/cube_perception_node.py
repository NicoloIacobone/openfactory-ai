"""cube_perception: depth + colour + camera_info + TF -> the Rubik's cube in the nest (phase 7,
D-025, docs/architecture.md).

Thin node around core.cube_view.find. Inputs are camera data and TF only (rule 5); the nest
frame and the cube size are fixed infrastructure read from openfactory_description. For every
depth frame that has its colour frame:

    /perception/cube/detection   vision_msgs/Detection3DArray in the world frame, image stamp:
                                 empty if no cube stands in the nest, else one detection with
                                 the pose of the cube (centre; orientation of its top face,
                                 yaw within 45 degrees of the nest axes), the measured size
                                 and a score in (0, 1] that falls when the cube is not a
                                 square block (a layer out of square, an uneven top)
    /perception/cube/top_face    sensor_msgs/Image rgb8, 3 x 3: mean colour of the nine
                                 stickers of the top face as seen from above (row 0 on the
                                 +y side of the face, column 0 on its -x side), same stamp

The node sees whatever is above the nest: consumers use frames taken with the arm at ready.
"""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from . import constants as c
from .core import camera, cube_view
from .scene_perception_node import DEPTH_ENCODING, image_array, to_pose, transform_matrix


def score(view, side, sigma):
    """Confidence in (0, 1] that what stands in the nest is a square block of the cube's
    size: 1 for a flat top face of the right size with nothing sticking out under it."""
    residual = ((view.protrusion / sigma['protrusion']) ** 2
                + (view.roughness / sigma['roughness']) ** 2
                + sum(((s - side) / sigma['size']) ** 2 for s in view.size))
    return math.exp(-0.5 * residual)


def face_matrix(view):
    """4 x 4 pose of the cube in the nest frame: its centre, z along the normal of the top
    face, x turned by the yaw of the top face."""
    yaw = camera.yaw_matrix(view.center, view.yaw)
    # the smallest rotation that takes z to the measured normal is about a horizontal axis;
    # the tilt is small, and only its size is used downstream
    tilt = np.eye(4)
    tilt[:3, :3] = cv2.Rodrigues(np.array([view.tilt, 0.0, 0.0]))[0]
    return yaw @ tilt


class CubePerception(Node):

    def __init__(self):
        super().__init__('cube_perception')
        own = Path(get_package_share_directory('openfactory_perception')) / 'config'
        description = Path(get_package_share_directory(c.DESCRIPTION_PACKAGE)) / 'config'
        config_file = self.declare_parameter('config_file', str(own / 'perception.yaml')).value
        config = yaml.safe_load(Path(config_file).read_text())
        self.cfg = config['cube']
        self.offset = config['principal_point_offset']
        self.pitch = yaml.safe_load((description / 'rubik.yaml').read_text())['cube']['pitch']
        self.nest_frame = self.declare_parameter('nest_frame', c.NEST_FRAME).value

        self.intrinsics = None
        self.colors = {}                       # stamp in ns -> colour image message
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detection_pub = self.create_publisher(Detection3DArray, c.CUBE_DETECTION_TOPIC, 10)
        self.face_pub = self.create_publisher(Image, c.CUBE_FACE_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, c.CUBE_DEBUG_TOPIC, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, c.CAMERA_INFO_TOPIC, self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, c.COLOR_TOPIC, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, c.DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f'cube_perception: nest frame {self.nest_frame}, cubie {self.pitch} m')

    def _on_info(self, msg):
        self.intrinsics = camera.intrinsics_from_k(msg.k, self.offset)

    def _on_color(self, msg):
        self.colors[Time.from_msg(msg.header.stamp).nanoseconds] = msg
        for stamp in sorted(self.colors)[:-5]:
            del self.colors[stamp]

    def _lookup(self, target, source, stamp):
        if not self.tf_buffer.can_transform(target, source, stamp):
            self.get_logger().warn(f'no TF {target} <- {source}', throttle_duration_sec=5.0)
            return None
        return transform_matrix(self.tf_buffer.lookup_transform(target, source, stamp).transform)

    def _on_depth(self, msg):
        if self.intrinsics is None:
            return
        if msg.encoding != DEPTH_ENCODING:
            self.get_logger().error(f'unexpected depth encoding {msg.encoding}', once=True)
            return
        stamp = Time.from_msg(msg.header.stamp)
        color_msg = self.colors.get(stamp.nanoseconds)
        if color_msg is None:
            # the depth of a frame arrives after its colour (it goes through the depth model)
            self.get_logger().warn('no colour image for a depth frame',
                                   throttle_duration_sec=5.0)
            return
        if color_msg.encoding != 'rgb8':
            self.get_logger().error(f'unexpected colour encoding {color_msg.encoding}', once=True)
            return
        nest_from_optical = self._lookup(self.nest_frame, msg.header.frame_id, stamp)
        world_from_nest = self._lookup(c.WORLD_FRAME, self.nest_frame, stamp)
        if nest_from_optical is None or world_from_nest is None:
            return

        color = image_array(color_msg, np.uint8, 3)
        view = cube_view.find(image_array(msg, np.float32, 1), color, self.intrinsics,
                              nest_from_optical, self.pitch, self.cfg)
        header = Header(stamp=msg.header.stamp, frame_id=c.WORLD_FRAME)
        out = Detection3DArray(header=header)
        if view.present:
            pose = to_pose(world_from_nest @ face_matrix(view))
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = c.CUBE_CLASS
            result.hypothesis.score = float(score(view, 3 * self.pitch, self.cfg['sigma']))
            result.pose.pose = pose
            item = Detection3D(header=header, results=[result], id=c.CUBE_CLASS)
            item.bbox.center = pose
            item.bbox.size = Vector3(x=view.size[0], y=view.size[1], z=2.0 * view.center[2])
            out.detections.append(item)
            face = Image(header=Header(stamp=msg.header.stamp, frame_id=self.nest_frame),
                         height=3, width=3, encoding='rgb8',
                         is_bigendian=sys.byteorder == 'big', step=9)
            face.data = view.stickers.tobytes()
            self.face_pub.publish(face)
        self.detection_pub.publish(out)

        if self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(color_msg, color, view, nest_from_optical)

    def _publish_debug(self, color_msg, color, view, nest_from_optical):
        canvas = np.ascontiguousarray(color)
        if view.present:
            centers = cube_view.sticker_centers(view.center, view.yaw, self.pitch)
            pixels = camera.project(camera.transform_points(
                np.linalg.inv(nest_from_optical), centers.reshape(-1, 3)), self.intrinsics)
            for u, v in np.rint(pixels).astype(int):
                cv2.circle(canvas, (int(u), int(v)), 2, (255, 0, 255), 1)
        out = Image(header=color_msg.header, height=color_msg.height, width=color_msg.width,
                    encoding='rgb8', is_bigendian=sys.byteorder == 'big',
                    step=color_msg.width * 3)
        out.data = canvas.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CubePerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
