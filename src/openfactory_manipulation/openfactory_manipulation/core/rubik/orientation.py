"""How the cube sits in the nest and what the cell's primitives do to it.

An orientation is the rotation, one of the 24, from the cube frame (`cube.py`: x to R, y to U,
z to F) to the nest frame, whose z points up. The nest holds the bottom layer; the gripper
holds the two layers above it, the cube's core among them, so the cube frame follows the
gripper.

    turn   the gripper turns k x 90 degrees about the nest z axis (right-handed: k = 1 is
           anticlockwise seen from above). The cube frame turns with it, and in that frame the
           face that is down makes k quarter turns anticlockwise seen from outside.
    flip   the gripper lifts the cube, turns it `yaw` quarter turns about z, then applies a
           fixed rotation of the nest frame (the cell's flip, a quarter turn about a
           horizontal axis) and sets it down again.
"""

from . import cubies as cb
from . import rotations as rot
from .cube import FACE_OF, format_move

UP = (0, 0, 1)
DOWN = (0, 0, -1)
ORIENTATIONS = rot.ROTATIONS


def face_towards(orientation, direction):
    """Face of the cube that looks along the nest-frame unit vector `direction`."""
    return FACE_OF[rot.mat_vec(rot.transpose(orientation), direction)]


def down_face(orientation):
    return face_towards(orientation, DOWN)


def up_face(orientation):
    return face_towards(orientation, UP)


def with_face_down(face):
    """The four orientations that have `face` down."""
    return tuple(o for o in ORIENTATIONS if down_face(o) == face)


def gripper_turns(turns):
    """Gripper quarter turns (-1, 1 or 2) that make `turns` clockwise turns of the down face."""
    return {1: -1, 2: 2, 3: 1}[turns % 4]


def turn_move(orientation, quarter_turns):
    """Move, such as "F'", that `quarter_turns` of the gripper make; None for a full turn."""
    if quarter_turns % 4 == 0:
        return None
    return format_move(down_face(orientation), -quarter_turns)


def after_turn(orientation, quarter_turns):
    return rot.mat_mul(rot.quarter_turn(UP, quarter_turns), orientation)


def after_flip(orientation, flip, yaw=0):
    return rot.mat_mul(flip, rot.mat_mul(rot.quarter_turn(UP, yaw), orientation))


def physical_turn(cubies, orientation, quarter_turns):
    """(arrangement, orientation) after a turn, from the rigid motion of the cubies alone.

    Every cubie on or above the middle of the cube turns with the gripper about the nest z
    axis, the bottom layer stays where it is, and the result is read in the new cube frame.
    It is the geometry `turn_move` and `after_turn` summarise, used to check them.
    """
    turn = rot.quarter_turn(UP, quarter_turns)
    new_orientation = rot.mat_mul(turn, orientation)
    back = rot.transpose(new_orientation)
    result = {}
    for home, (slot, rotation) in cubies.items():
        position = rot.mat_vec(orientation, slot)
        attitude = rot.mat_mul(orientation, rotation)
        if position[2] >= 0:
            position = rot.mat_vec(turn, position)
            attitude = rot.mat_mul(turn, attitude)
        rotation = rot.IDENTITY if cb.is_center(home) else rot.mat_mul(back, attitude)
        result[home] = (rot.mat_vec(back, position), rotation)
    return result, new_orientation
