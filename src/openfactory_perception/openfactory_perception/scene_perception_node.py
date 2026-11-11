"""scene_perception: depth image + camera_info + TF -> object hypotheses (docs/architecture.md).

Thin node around core.pipeline.detect. Inputs are camera data and TF only (rule 5); the bin
geometry and the object catalog are fixed infrastructure read from openfactory_description.
Every depth frame gives one Detection3DArray in the world frame with the image stamp, empty when
the bin is empty. The node sees whatever is in the bin volume below `max_object_height`:
consumers use frames taken while the arm is clear of the input bin.
"""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from . import constants as c
from .core import camera, pipeline, primitives

DEPTH_ENCODING = '32FC1'
CLOUD_FIELDS = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(('x', 'y', 'z', 'label'))]


def image_array(msg, dtype, channels):
    """Image message as an H x W (x channels) array, without copying."""
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    row = msg.width * channels
    data = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, -1)[:, :row]
    return data if channels == 1 else data.reshape(msg.height, msg.width, channels)


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    return camera.pose_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def to_pose(matrix):
    x, y, z, w = camera.quat_from_rotation(matrix[:3, :3])
    px, py, pz = (float(v) for v in matrix[:3, 3])
    return Pose(position=Point(x=px, y=py, z=pz), orientation=Quaternion(x=x, y=y, z=z, w=w))


class ScenePerception(Node):

    def __init__(self):
        super().__init__('scene_perception')
        own = Path(get_package_share_directory('openfactory_perception')) / 'config'
        description = Path(get_package_share_directory(c.DESCRIPTION_PACKAGE)) / 'config'
        config_file = self.declare_parameter('config_file', str(own / 'perception.yaml')).value
        self.cfg = yaml.safe_load(Path(config_file).read_text())
        catalog = yaml.safe_load((description / 'objects.yaml').read_text())
        cell = yaml.safe_load((description / 'cell.yaml').read_text())
        self.inner_size = cell['bins'][c.INPUT_BIN]['inner_size']
        self.models = primitives.shape_models(catalog, self.cfg)

        self.intrinsics = None
        self.color = None
        self.color_sub = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.objects_pub = self.create_publisher(Detection3DArray, c.OBJECTS_TOPIC, 10)
        self.points_pub = self.create_publisher(
            point_cloud2.PointCloud2, c.DEBUG_POINTS_TOPIC, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, c.DEBUG_IMAGE_TOPIC, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, c.CAMERA_INFO_TOPIC, self._on_info, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, c.DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        # the colour image is only needed, and only bridged, while someone views the debug image
        self.debug_timer = self.create_timer(1.0, self._watch_debug_image)
        self.get_logger().info(
            f'scene_perception: bin {self.inner_size}, shapes {[m.name for m in self.models]}')

    def _on_info(self, msg):
        self.intrinsics = camera.intrinsics_from_k(msg.k, self.cfg['principal_point_offset'])

    def _on_color(self, msg):
        self.color = msg

    def _watch_debug_image(self):
        wanted = self.cfg['publish_debug'] and self.image_pub.get_subscription_count() > 0
        if wanted and self.color_sub is None:
            self.color_sub = self.create_subscription(
                Image, c.COLOR_TOPIC, self._on_color, qos_profile_sensor_data)
        elif not wanted and self.color_sub is not None:
            self.destroy_subscription(self.color_sub)
            self.color_sub, self.color = None, None

    def _lookup(self, target, source, stamp):
        """4 x 4 transform target <- source at the image stamp, or None while TF is missing."""
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
        bin_from_optical = self._lookup(c.INPUT_BIN_FRAME, msg.header.frame_id, stamp)
        world_from_bin = self._lookup(c.WORLD_FRAME, c.INPUT_BIN_FRAME, stamp)
        if bin_from_optical is None or world_from_bin is None:
            return

        depth = image_array(msg, np.float32, 1)
        detections, debug = pipeline.detect(
            depth, self.intrinsics, bin_from_optical, self.inner_size, self.models, self.cfg)

        header = Header(stamp=msg.header.stamp, frame_id=c.WORLD_FRAME)
        out = Detection3DArray(header=header)
        for index, detection in enumerate(detections):
            pose = to_pose(world_from_bin @ camera.yaw_matrix(detection.xyz, detection.yaw))
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = detection.class_id
            result.hypothesis.score = float(detection.score)
            result.pose.pose = pose
            item = Detection3D(header=header, results=[result], id=f'{detection.class_id}_{index}')
            item.bbox.center = pose
            item.bbox.size = Vector3(**dict(zip('xyz', (float(v) for v in detection.size))))
            out.detections.append(item)
        self.objects_pub.publish(out)

        if self.cfg['publish_debug']:
            self._publish_debug(msg, detections, debug, bin_from_optical)

    def _publish_debug(self, msg, detections, debug, bin_from_optical):
        if self.points_pub.get_subscription_count() > 0:
            cloud = np.column_stack((debug.points, debug.labels)).astype(np.float32)
            header = Header(stamp=msg.header.stamp, frame_id=c.INPUT_BIN_FRAME)
            self.points_pub.publish(point_cloud2.create_cloud(header, CLOUD_FIELDS, cloud))
        color = self.color
        if color is None or self.image_pub.get_subscription_count() == 0:
            return
        if color.encoding != 'rgb8':
            self.get_logger().error(f'unexpected colour encoding {color.encoding}', once=True)
            return
        canvas = np.ascontiguousarray(image_array(color, np.uint8, 3))
        optical_from_bin = np.linalg.inv(bin_from_optical)
        for detection in detections:
            # outline of the bounding box top face, seen in the image
            sx, sy, sz = detection.size
            corners = np.array([[dx * sx / 2, dy * sy / 2, sz / 2, 1.0]
                                for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
            in_bin = corners @ camera.yaw_matrix(detection.xyz, detection.yaw).T
            pixels = camera.project(camera.transform_points(optical_from_bin, in_bin[:, :3]),
                                    self.intrinsics)
            outline = np.rint(pixels).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [outline], True, (255, 255, 255), 1)
            anchor = (int(outline[:, 0, 0].min()), int(outline[:, 0, 1].min()) - 4)
            cv2.putText(canvas, f'{detection.class_id} {detection.score:.2f}', anchor,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        out = Image(header=color.header, height=color.height, width=color.width, encoding='rgb8',
                    is_bigendian=sys.byteorder == 'big', step=color.width * 3)
        out.data = canvas.tobytes()
        self.image_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePerception()
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
