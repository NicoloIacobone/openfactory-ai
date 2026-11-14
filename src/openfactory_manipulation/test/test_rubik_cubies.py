import math
import random

import pytest

from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import cubies as cb
from openfactory_manipulation.core.rubik import orientation as ori
from openfactory_manipulation.core.rubik import rotations as rot

ALL_MOVES = [face + suffix for face in c.FACES for suffix in ('', "'", '2')]


def scrambled(seed):
    return c.apply_moves(c.SOLVED, c.random_scramble(random.Random(seed)))


def test_the_24_rotations():
    assert len(rot.ROTATIONS) == 24
    assert all(rot.det(m) == 1 for m in rot.ROTATIONS)
    assert rot.mat_vec(rot.quarter_turn(rot.Z, 1), rot.X) == rot.Y
    assert rot.mat_vec(rot.quarter_turn((0, 0, -1), 1), rot.X) == (0, -1, 0)
    assert rot.quarter_turn(rot.X, 4) == rot.IDENTITY
    assert rot.quarter_turn(rot.Y, -1) == rot.quarter_turn(rot.Y, 3)


@pytest.mark.parametrize('m', rot.ROTATIONS)
def test_rpy_and_nearest_rotation_round_trip(m):
    back = rot.from_rpy(*rot.rpy_of(m))
    assert all(back[i][j] == pytest.approx(m[i][j], abs=1e-12)
               for i in range(3) for j in range(3))
    tilted = rot.mat_mul(m, rot.from_rpy(0.02, 0.0, 0.0))
    nearest, angle = rot.nearest_rotation(tilted)
    assert nearest == m
    assert angle == pytest.approx(0.02, abs=1e-6)


def test_quaternion_to_matrix():
    half = math.sqrt(0.5)
    m = rot.from_quaternion((0.0, 0.0, half, half))
    assert rot.nearest_rotation(m)[0] == rot.quarter_turn(rot.Z, 1)


def test_cubie_names():
    assert len(cb.HOMES) == 26
    assert cb.name_of((1, 0, -1)) == 'cubie_pzn'
    assert all(cb.home_of(cb.name_of(home)) == home for home in cb.HOMES)
    assert sum(cb.is_center(home) for home in cb.HOMES) == 6


def test_solved_arrangement():
    assert cb.arrangement(c.SOLVED) == cb.solved_arrangement()
    assert cb.state_of(cb.solved_arrangement()) == c.SOLVED


@pytest.mark.parametrize('seed', range(20))
def test_arrangement_round_trip(seed):
    state = scrambled(seed)
    cubies = cb.arrangement(state)
    assert sorted(slot for slot, _ in cubies.values()) == list(cb.HOMES)
    assert all(r in rot.ROTATIONS for _, r in cubies.values())
    assert cb.state_of(cubies) == state


@pytest.mark.parametrize('move', ALL_MOVES)
def test_cubie_moves_agree_with_facelet_moves(move):
    state = scrambled(11)
    face, turns = c.parse_move(move)
    assert cb.state_of(cb.apply_move(cb.arrangement(state), face, turns)) == c.apply_move(
        state, move)


def test_arrangement_rejects_impossible_pieces():
    cells = list(c.SOLVED)
    cells[19], cells[28] = cells[28], cells[19]          # U and D on one edge
    with pytest.raises(ValueError, match='impossible'):
        cb.arrangement(''.join(cells))
    cells = list(c.SOLVED)
    cells[9], cells[20] = cells[20], cells[9]            # mirrored corner URF
    with pytest.raises(ValueError, match='mirrored'):
        cb.arrangement(''.join(cells))


def test_faces_of_an_orientation():
    assert ori.up_face(rot.IDENTITY) == 'F'              # cube z (F) along nest z
    assert ori.down_face(rot.IDENTITY) == 'B'
    assert len(ori.ORIENTATIONS) == 24
    for face in c.FACES:
        four = ori.with_face_down(face)
        assert len(four) == 4
        assert {ori.up_face(o) for o in four} == {c.FACE_OF[tuple(-x for x in c.NORMAL_OF[face])]}


@pytest.mark.parametrize('orientation', ori.ORIENTATIONS)
@pytest.mark.parametrize('k', [-1, 1, 2, 3])
def test_turn_summary_matches_the_rigid_motion(orientation, k):
    state = scrambled(23)
    cubies, new_orientation = ori.physical_turn(cb.arrangement(state), orientation, k)
    move = ori.turn_move(orientation, k)
    assert move[0] == ori.down_face(orientation)
    assert cb.state_of(cubies) == c.apply_move(state, move)
    assert new_orientation == ori.after_turn(orientation, k)
    # the face that is down stays down, and the gripper turns asked for a move make it
    assert ori.down_face(new_orientation) == ori.down_face(orientation)
    face, turns = c.parse_move(move)
    assert ori.turn_move(orientation, ori.gripper_turns(turns)) == move


def test_full_turn_is_no_move():
    assert ori.turn_move(rot.IDENTITY, 4) is None
    assert ori.turn_move(rot.IDENTITY, 0) is None


def test_flip_brings_a_side_face_down():
    # a quarter turn about the nest y axis that takes -x (towards the robot) to -z (down)
    flip = rot.quarter_turn(rot.Y, -1)
    assert rot.mat_vec(flip, (-1, 0, 0)) == ori.DOWN
    for orientation in ori.ORIENTATIONS:
        sides = set()
        for yaw in range(4):
            flipped = ori.after_flip(orientation, flip, yaw)
            sides.add(ori.down_face(flipped))
            towards_robot = ori.face_towards(
                rot.mat_mul(rot.quarter_turn(ori.UP, yaw), orientation), (-1, 0, 0))
            assert ori.down_face(flipped) == towards_robot
        # the four yaws bring down the four side faces, never the top or the bottom one
        assert len(sides) == 4
        assert ori.up_face(orientation) not in sides
        assert ori.down_face(orientation) not in sides
