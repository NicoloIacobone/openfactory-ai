"""Client side of the cube perception interface (phase 7): views of the Rubik's cube in the nest
from `/perception/cube/detection` and `/perception/cube/top_face`.

Manipulation consumes these views only; it never reads camera data or simulator state.
"""

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray

from . import constants as c
from .core.geometry import quat_rotate, relative, yaw_of
from .core.rubik.view import View
from .perception_client import WALL_LIMIT


def view_from_msgs(detections, face, nest_pose, seat):
    """A `View` from a detection array (world frame), the top-face image of the same stamp
    (None if no cube was seen), the world pose of the nest frame and the seat of the cube in
    it."""
    if detections.header.frame_id != c.WORLD_FRAME:
        raise ValueError(f'detections in frame {detections.header.frame_id!r}')
    stamp = detections.header.stamp.sec + detections.header.stamp.nanosec * 1e-9
    results = [d.results[0] for d in detections.detections
               if d.results and d.results[0].hypothesis.class_id == c.CUBE_CLASS]
    if not results or face is None:
        return View(stamp, False)
    p, q = results[0].pose.pose.position, results[0].pose.pose.orientation
    xyz, quat = relative(nest_pose, ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
    up = quat_rotate(quat, (0.0, 0.0, 1.0))
    data = bytes(face.data)
    stickers = tuple(tuple(float(v) for v in data[3 * k:3 * k + 3]) for k in range(9))
    return View(stamp, True, tuple(a - b for a, b in zip(xyz, seat)), yaw_of(quat),
                math.acos(max(-1.0, min(1.0, up[2]))), results[0].hypothesis.score, stickers)


class CubeViewClient:

    def __init__(self, node):
        self.node = node
        self.latest = None
        self.faces = {}                        # stamp (s, ns) -> top-face image
        self.detection_sub = node.create_subscription(
            Detection3DArray, c.CUBE_DETECTION_TOPIC, self._on_detections, 10)
        self.face_sub = node.create_subscription(Image, c.CUBE_FACE_TOPIC, self._on_face, 10)

    def _on_detections(self, msg):
        self.latest = msg

    def _on_face(self, msg):
        self.faces[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        for stamp in sorted(self.faces)[:-10]:
            del self.faces[stamp]

    def wait_fresh(self, after, timeout_s, nest_pose, seat):
        """The first view stamped later than `after` (rclpy Time), spinning the node
        meanwhile; None on timeout. The wait is counted on the node's clock, with a wall-clock
        limit in case the clock stops (PerceptionClient.wait_fresh)."""
        clock = self.node.get_clock()
        deadline = clock.now() + Duration(seconds=timeout_s)
        end = time.monotonic() + WALL_LIMIT * timeout_s
        while clock.now() < deadline and time.monotonic() < end:
            msg = self.latest
            if msg is not None and Time.from_msg(msg.header.stamp) > after:
                face = self.faces.get((msg.header.stamp.sec, msg.header.stamp.nanosec))
                if face is not None or not msg.detections:
                    return view_from_msgs(msg, face, nest_pose, seat)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return None
