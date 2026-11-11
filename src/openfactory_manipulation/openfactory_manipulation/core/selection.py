"""ROS-free choice of the object to pick among perception hypotheses (phase 3).

A hypothesis is {'class_id', 'score', 'xyz', 'quat'} with the pose of the catalog object frame
in the world frame. Phase 3 picks one separate object: the most confident hypothesis of a
catalog type. Grasp candidates and feasibility ranking come with phase 4.
"""

from .geometry import relative, yaw_of

NO_DETECTION = 'no_detection'


def select_object(hypotheses, catalog, min_score):
    """The hypothesis to pick, or None: highest score among catalog types at or above
    `min_score` (unknown clusters and weak hypotheses are never picked)."""
    usable = [h for h in hypotheses
              if h['class_id'] in catalog['objects'] and h['score'] >= min_score]
    return max(usable, key=lambda h: h['score']) if usable else None


def pose_in_bin(bin_pose, hypothesis):
    """(x, y, yaw) of a hypothesis in a bin frame given by its world pose (xyz, quat)."""
    (x, y, _), quat = relative(bin_pose, (hypothesis['xyz'], hypothesis['quat']))
    return x, y, yaw_of(quat)
