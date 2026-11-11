"""Client side of the perception interface: object hypotheses from `/perception/objects`.

Manipulation consumes hypotheses only; it never reads camera data or simulator state.
"""

import time

import rclpy
from rclpy.time import Time
from vision_msgs.msg import Detection3DArray

from . import constants as c


def hypotheses_from_msg(msg):
    """Detection3DArray -> [{'class_id', 'score', 'xyz', 'quat', 'size'}] (world frame)."""
    if msg.header.frame_id != c.WORLD_FRAME:
        raise ValueError(f'hypotheses in frame {msg.header.frame_id!r}, expected {c.WORLD_FRAME!r}')
    hypotheses = []
    for detection in msg.detections:
        if not detection.results:
            continue
        result = detection.results[0]
        p, q = result.pose.pose.position, result.pose.pose.orientation
        size = detection.bbox.size
        hypotheses.append({
            'class_id': result.hypothesis.class_id, 'score': result.hypothesis.score,
            'xyz': (p.x, p.y, p.z), 'quat': (q.x, q.y, q.z, q.w),
            'size': (size.x, size.y, size.z),
        })
    return hypotheses


class PerceptionClient:

    def __init__(self, node):
        self.node = node
        self.latest = None
        self.objects_sub = node.create_subscription(
            Detection3DArray, c.OBJECTS_TOPIC, self._on_objects, 10)

    def _on_objects(self, msg):
        self.latest = msg

    def wait_fresh(self, after, timeout_s):
        """Hypotheses of the first view stamped later than `after` (rclpy Time), spinning the
        node meanwhile: (hypotheses, stamp in seconds), or None on timeout."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            msg = self.latest
            if msg is not None and Time.from_msg(msg.header.stamp) > after:
                stamp = msg.header.stamp
                return hypotheses_from_msg(msg), stamp.sec + stamp.nanosec * 1e-9
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return None
