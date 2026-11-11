"""Camera-driven object selection for one pick (phase 3).

The arm goes to the ready configuration, clear of the input bin, a view newer than that is
taken from perception, and the most confident catalog hypothesis becomes the object pose that
PickPlace.run used to get as a known pose. Nothing here reads camera data or simulator state.
"""

import time

from . import constants as c
from .core import selection


def perceive(node, runner, client):
    """Look at the input bin and choose what to pick.

    Returns {'ok', 'failure_reason', 'hypotheses', 'selected', 'object_type', 'x', 'y', 'yaw',
    'stamp', 'perception_s'}; x, y, yaw are in the input_bin frame. `perception_s` is the wall
    time from asking for a view to having chosen the object.
    """
    cfg = runner.cfg['perception']
    result = {'ok': False, 'failure_reason': None, 'hypotheses': [], 'selected': None,
              'object_type': None, 'x': None, 'y': None, 'yaw': None, 'stamp': None,
              'perception_s': None}
    if not runner.go_ready():
        result['failure_reason'] = 'planning_failed'
        return result
    runner.moveit.spin_for(cfg['settle_s'])

    start = time.monotonic()
    view = client.wait_fresh(node.get_clock().now(), cfg['timeout_s'])
    if view is None:
        result.update(failure_reason='timeout', perception_s=time.monotonic() - start)
        return result
    hypotheses, stamp = view
    selected = selection.select_object(hypotheses, runner.catalog, cfg['min_score'])
    result.update(hypotheses=hypotheses, stamp=stamp)
    if selected is None:
        result.update(failure_reason=selection.NO_DETECTION, perception_s=time.monotonic() - start)
        return result
    x, y, yaw = selection.pose_in_bin(runner.moveit.frame_pose(c.INPUT_BIN_FRAME), selected)
    result.update(ok=True, selected=selected, object_type=selected['class_id'], x=x, y=y, yaw=yaw,
                  perception_s=time.monotonic() - start)
    return result
