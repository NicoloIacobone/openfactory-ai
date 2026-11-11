"""ROS-free grasp candidates for catalog primitives resting on a bin floor (phase 4, D-016).

Everything is expressed in the bin frame (origin at the centre of the inner floor, z up). The
order is fixed (docs/agent-guides/manipulation.md): candidates are generated from geometry,
hard filters reject them with a reason, and only the survivors are ranked. This module holds
the generation, the geometric part of the filters (gripper width, fingers and palm against the
bin walls and against the other objects seen in the bin) and the baseline score. MoveIt has the
last word on feasibility: IK, collisions and the approach and lift lines are checked by
grasp_selector on every candidate that passes here.

Top-down grasps only: the tool axis is vertical and the fingers close along `finger_yaw`.
"""

import math
from dataclasses import dataclass

from .geometry import compose, relative, wrap_angle, yaw_of
from .pick_place import finger_geometry, grasp_offset, top_down_quat

UNKNOWN = 'unknown'
# Reasons a candidate is rejected for, in the order the filters run
GEOMETRIC_REJECTIONS = ('gripper_width', 'wall_collision', 'neighbour_collision')
MOVEIT_REJECTIONS = ('ik_failed', 'approach_blocked', 'lift_blocked')
_CIRCLE_SEGMENTS = 16


@dataclass(frozen=True)
class SceneObject:
    """One perception hypothesis in the bin frame; `size` is its bounding box (x, y, z)."""

    index: int
    class_id: str
    score: float
    x: float
    y: float
    yaw: float
    size: tuple


@dataclass(frozen=True)
class Grasp:
    """A top-down grasp of one object: gripper_tcp at (x, y, z), fingers closing along
    `finger_yaw` (direction of the tool y axis, in (-pi/2, pi/2]) on a width of `width`."""

    object_index: int
    class_id: str
    label: str
    x: float
    y: float
    z: float
    finger_yaw: float
    width: float
    approach_position: float      # finger joint value during the approach: half the opening


def scene_objects(hypotheses, bin_pose):
    """Perception hypotheses (world frame, see perception_client) as SceneObjects in the frame
    of the bin whose world pose is `bin_pose`."""
    objects = []
    for index, h in enumerate(hypotheses):
        (x, y, _), quat = relative(bin_pose, (h['xyz'], h['quat']))
        objects.append(SceneObject(index, h['class_id'], h['score'], x, y, yaw_of(quat),
                                   tuple(h['size'])))
    return objects


def half_turn(angle):
    """A direction (the gripper looks the same after half a turn) reduced to (-pi/2, pi/2]."""
    angle = wrap_angle(angle)
    if angle > math.pi / 2:
        angle -= math.pi
    elif angle <= -math.pi / 2:
        angle += math.pi
    return angle


def generate(obj, spec, gripper, params):
    """Top-down candidates for one object of catalog type `spec`.

    Box: across x and across y. Cylinder (standing): `cylinder_directions` evenly spread
    directions. Triangular prism (ridge up): across its length only, on the two triangular end
    faces; the slanted faces cannot be held by parallel pads.
    """
    shape = spec['shape']
    if shape == 'box':
        directions = [('across_x', obj.yaw, spec['size'][0]),
                      ('across_y', obj.yaw + math.pi / 2, spec['size'][1])]
    elif shape == 'cylinder':
        n = params['cylinder_directions']
        directions = [(f'direction_{k}', k * math.pi / n, 2 * spec['radius']) for k in range(n)]
    elif shape == 'triangular_prism':
        directions = [('across_length', obj.yaw, spec['length'])]
    else:
        raise ValueError(f'unknown shape {shape!r}')

    height = obj.size[2]
    z = height / 2 + grasp_offset(spec, gripper, params)
    stroke = gripper['finger']['stroke']
    return [Grasp(obj.index, obj.class_id, label, obj.x, obj.y, z, half_turn(direction), width,
                  min(width / 2 + params['preclose_margin'], stroke))
            for label, direction, width in directions]


# -- planar geometry ---------------------------------------------------------------------------

def rectangle(x, y, yaw, size_x, size_y):
    """Corners of a rectangle centred on (x, y), its x side turned by `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((size_x / 2, size_y / 2), (-size_x / 2, size_y / 2),
                           (-size_x / 2, -size_y / 2), (size_x / 2, -size_y / 2))]


def footprint(obj, spec=None):
    """Top-view outline of an object: a circle (as a polygon around it) for a standing
    cylinder, the bounding rectangle otherwise, also for a cluster of unknown shape."""
    if spec is not None and spec['shape'] == 'cylinder':
        # vertices on the circumscribed circle, so the polygon never lies inside the cylinder
        radius = spec['radius'] / math.cos(math.pi / _CIRCLE_SEGMENTS)
        return [(obj.x + radius * math.cos(2 * math.pi * k / _CIRCLE_SEGMENTS),
                 obj.y + radius * math.sin(2 * math.pi * k / _CIRCLE_SEGMENTS))
                for k in range(_CIRCLE_SEGMENTS)]
    return rectangle(obj.x, obj.y, obj.yaw, obj.size[0], obj.size[1])


def _edges(polygon):
    return [(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon))]


def _overlap(a, b):
    """True if two convex polygons overlap or touch (separating-axis test)."""
    for polygon in (a, b):
        for (x0, y0), (x1, y1) in _edges(polygon):
            nx, ny = y1 - y0, x0 - x1
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def _point_segment(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def polygon_distance(a, b):
    """Distance between two convex polygons; 0 when they overlap or touch."""
    if _overlap(a, b):
        return 0.0
    return min(min(_point_segment(p, q0, q1) for p in a for q0, q1 in _edges(b)),
               min(_point_segment(p, q0, q1) for p in b for q0, q1 in _edges(a)))


# -- geometric filters -------------------------------------------------------------------------

def gripper_parts(grasp, gripper):
    """[(top-view polygon, lowest height above the bin floor)] of the two fingers at the
    approach opening and of the palm, with the gripper at the grasp pose."""
    tip_below, palm_above = finger_geometry(gripper)
    width, thickness, _ = gripper['finger']['size']
    palm_x, palm_y, _ = gripper['palm']['size']
    c, s = math.cos(grasp.finger_yaw), math.sin(grasp.finger_yaw)
    parts = []
    for side in (1.0, -1.0):
        # finger box: inner pad face at the approach opening, the box extends outwards
        offset = side * (grasp.approach_position + thickness / 2)
        parts.append((rectangle(grasp.x + c * offset, grasp.y + s * offset, grasp.finger_yaw,
                                thickness, width), grasp.z - tip_below))
    parts.append((rectangle(grasp.x, grasp.y, grasp.finger_yaw, palm_y, palm_x),
                  grasp.z + palm_above))
    return parts


def wall_clearance(parts, inner_size):
    """Smallest distance of the gripper parts below the rim to the inner faces of the walls;
    negative when a part is in a wall. Infinite when nothing is below the rim."""
    half_x, half_y, rim = inner_size[0] / 2, inner_size[1] / 2, inner_size[2]
    return min((min(half_x - abs(x), half_y - abs(y))
                for polygon, low in parts if low < rim for x, y in polygon),
               default=math.inf)


def neighbour_clearance(parts, obstacles):
    """Smallest top-view distance of the gripper parts to the obstacles that reach up to them.

    `obstacles`: [(footprint polygon, height above the bin floor)]. Infinite with none in reach.
    """
    return min((polygon_distance(polygon, outline)
                for polygon, low in parts for outline, height in obstacles if low < height),
               default=math.inf)


def obstacles_for(grasp, objects, catalog):
    """Footprint and height of every object in the bin except the one being grasped."""
    return [(footprint(other, catalog['objects'].get(other.class_id)), other.size[2])
            for other in objects if other.index != grasp.object_index]


def prefilter(grasp, gripper, inner_size, obstacles, params):
    """(rejection reason or None, {'wall_clearance', 'neighbour_clearance'}).

    Hard geometric filters, in order: the object fits the open gripper, the gripper keeps
    `finger_clearance` from the bin walls and from the other objects.
    """
    parts = gripper_parts(grasp, gripper)
    features = {'wall_clearance': wall_clearance(parts, inner_size),
                'neighbour_clearance': neighbour_clearance(parts, obstacles)}
    if grasp.width / 2 + params['preclose_margin'] > gripper['finger']['stroke']:
        return 'gripper_width', features
    if features['wall_clearance'] < params['finger_clearance']:
        return 'wall_collision', features
    if features['neighbour_clearance'] < params['finger_clearance']:
        return 'neighbour_collision', features
    return None, features


# -- ranking -----------------------------------------------------------------------------------

def score(confidence, clearance, motion, weights):
    """Baseline hand-designed score of a feasible candidate, higher is better.

    `confidence`: perception score of the object; `clearance`: smallest gap of the gripper to
    walls and neighbours (m); `motion`: joint-space distance from the ready configuration to
    the pre-grasp (rad). Clearance and motion are normalised by their caps. Collisions never
    enter here: they are hard filters.
    """
    room = min(max(clearance, 0.0), weights['clearance_cap']) / weights['clearance_cap']
    effort = min(max(motion, 0.0), weights['motion_cap']) / weights['motion_cap']
    return (weights['confidence'] * confidence + weights['clearance'] * room
            - weights['motion'] * effort)


def joint_distance(a, b):
    """Largest joint displacement between two arm configurations (rad). Not wrapped: the arm
    joints turn more than one revolution, so 2 pi apart is a real motion."""
    return max(abs(x - y) for x, y in zip(a, b))


# -- poses -------------------------------------------------------------------------------------

def tcp_pose(grasp, bin_pose):
    """World pose (xyz, quat) of gripper_tcp for a grasp given in the frame of `bin_pose`."""
    return compose(bin_pose, ((grasp.x, grasp.y, grasp.z), top_down_quat(grasp.finger_yaw)))
