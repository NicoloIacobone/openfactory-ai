"""ROS-free decisions of the relay loop of the ring of N arms (phase 6, D-021).

Arm i picks from bin i and places into bin (i + 1) mod N, one arm at a time. Every bin has a
camera, so what the single-arm cell had to remember about its output bin (D-017) is seen here:
the free place points come from the view of the destination bin, and a handover counts only
when the object is seen there. Everything comes from perception and from the cell's own
actions, never from the simulator.
"""

import math

from . import task

# Why the loop ends; `handovers_done` is the only one that means the job is done
END_REASONS = ('handovers_done', 'no_work', 'failure_budget', 'park_failed', 'no_view')
# Failure reason the loop adds to those of the bin-picking attempt
NOT_ARRIVED = 'object_not_arrived'


def source_bin(arm):
    return arm


def destination_bin(arm, n):
    return (arm + 1) % n


def footprint_radius(obj):
    """Radius of the circle around the top-view bounding box of a seen object."""
    return math.hypot(obj.size[0], obj.size[1]) / 2


def room(xy, objects):
    """Distance from a place point to the nearest footprint circle of the objects seen in its
    bin (negative inside one); infinite in an empty bin."""
    return min((math.hypot(obj.x - xy[0], obj.y - xy[1]) - footprint_radius(obj)
                for obj in objects), default=math.inf)


def free_slots(slots, objects, margin):
    """[(index, xy)] of the place points with room for one more object, the roomiest first.

    `slots`: the configured place points of a bin; `objects`: what a view shows in that bin;
    `margin`: the room a place point needs. Rooms are compared to the centimetre, so that
    place points with about the same room keep the configured order.
    """
    free = [(index, tuple(xy), room(xy, objects)) for index, xy in enumerate(slots)]
    free = [item for item in free if item[2] >= margin]
    free.sort(key=lambda item: (-round(min(item[2], 1.0), 2), item[0]))
    return [(index, xy) for index, xy, _ in free]


def can_work(pickable, free):
    """An arm has something to do when its source bin shows an object it may try and its
    destination bin has a free place point."""
    return bool(pickable) and bool(free)


def next_arm(last, ready):
    """The arm whose turn it is: the first one after `last` (None at the start), going round
    the ring, with something to do; None when no arm has."""
    n = len(ready)
    first = 0 if last is None else last + 1
    for offset in range(n):
        arm = (first + offset) % n
        if ready[arm]:
            return arm
    return None


def prune_failures(failures, objects, radius):
    """Failed grasps [(x, y, finger_yaw)] of a bin that still have their object: once the
    object is gone (picked after all, or moved), another one may come to rest there."""
    return [failure for failure in failures
            if task.still_there(failure[0], failure[1], objects, radius)]


def nearest(xy, objects, radius):
    """The object of a view nearest to a point of its bin, within `radius`; None if none."""
    best, best_distance = None, radius
    for obj in objects:
        distance = math.hypot(obj.x - xy[0], obj.y - xy[1])
        if distance <= best_distance:
            best, best_distance = obj, distance
    return best


def prune_arrivals(arrivals, objects, radius):
    """Arrivals [(x, y, turn)] of a bin whose object is still seen where it arrived."""
    return [arrival for arrival in arrivals
            if task.still_there(arrival[0], arrival[1], objects, radius)]


def waiting_since(obj, arrivals, radius):
    """The turn in which an object seen in a bin arrived there, -1 if it was there from the
    start: the lower, the longer it has waited. `arrivals`: [(x, y, turn)] of that bin.

    The loop takes the object that waited longest first. Without this the grasp ranking
    prefers the object that was just set down (it stands alone on the roomiest place point),
    and one object goes round while the others never move.
    """
    turns = [turn for x, y, turn in arrivals
             if math.hypot(obj.x - x, obj.y - y) <= radius]
    return max(turns, default=-1)


def arrived(place_xy, objects, radius):
    """True if a view of the destination bin shows an object within `radius` of the place
    point the handed-over object was released at."""
    return task.still_there(place_xy[0], place_xy[1], objects, radius)


def handover_outcome(attempt_success, attempt_reason, arrival):
    """(success, failure_reason) of one handover as the cell itself can tell.

    `attempt_success`, `attempt_reason`: the verdict of the pick (fingers, execution, object
    gone from the source bin); `arrival`: whether the next view of the destination bin shows
    the object at its place point (None without a view).
    """
    if not attempt_success:
        return False, attempt_reason
    if arrival is None:
        return False, 'no_view'
    if not arrival:
        return False, NOT_ARRIVED
    return True, None


def run_end(step_reason, tried, handovers, idle, n):
    """Why a run of the loop ends after a turn, or None to go on.

    `step_reason`: the end reason of the turn itself, if any; `tried`: handovers tried so far,
    `handovers` the number asked for (None: for ever); `idle`: turns in a row in which the arm
    found nothing to try once at ready, although the survey had shown it work.
    """
    if step_reason is not None:
        return step_reason
    if handovers is not None and tried >= handovers:
        return 'handovers_done'
    if idle >= n:
        return 'no_work'                         # every arm in turn found nothing to try
    return None


def summarize(records, n):
    """Counts of a run of the loop: turns, handovers tried and done, per arm, and full laps
    (every arm has done at least that many handovers)."""
    tried = [r for r in records if r['attempt'] is not None]
    done = [0] * n
    for record in tried:
        done[record['arm']] += record['success']
    return {
        'turns': len(records),
        'skipped_turns': len(records) - len(tried),
        'handovers_tried': len(tried),
        'handovers_done': sum(done),
        'handovers_by_arm': done,
        'laps': min(done) if done else 0,
        'failures': [r['failure_reason'] for r in tried if not r['success']],
    }
