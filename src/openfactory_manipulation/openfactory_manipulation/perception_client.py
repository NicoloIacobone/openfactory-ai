"""Client side of the perception interface: object hypotheses from `/perception/objects`.

Manipulation consumes hypotheses only; it never reads camera data or simulator state.
"""

import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from vision_msgs.msg import Detection3DArray

from . import constants as c

WALL_LIMIT = 10        # wall-clock limit of a wait, as a multiple of its timeout on the node clock


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

    def __init__(self, node, topic=c.OBJECTS_TOPIC):
        self.node = node
        self.topic = topic
        self.latest = None
        self.objects_sub = node.create_subscription(
            Detection3DArray, topic, self._on_objects, 10)

    def _on_objects(self, msg):
        self.latest = msg

    def wait_fresh(self, after, timeout_s):
        """Hypotheses of the first view stamped later than `after` (rclpy Time), spinning the
        node meanwhile: (hypotheses, stamp in seconds), or None on timeout.

        `timeout_s` is counted on the node's clock, the clock of the image stamps: a
        simulation that runs slower than real time gets the same number of frames to deliver
        a view (with three cameras the VM runs at half speed and a view once took more than
        10 s of wall time, E-006). A wall-clock limit of WALL_LIMIT times that still ends the
        wait if the clock stops.
        """
        clock = self.node.get_clock()
        deadline = clock.now() + Duration(seconds=timeout_s)
        end = time.monotonic() + WALL_LIMIT * timeout_s
        while clock.now() < deadline and time.monotonic() < end:
            msg = self.latest
            if msg is not None and Time.from_msg(msg.header.stamp) > after:
                stamp = msg.header.stamp
                return hypotheses_from_msg(msg), stamp.sec + stamp.nanosec * 1e-9
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return None
