import math
import random

import pytest

import rubik_truth as rt
from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import cubies as cb
from openfactory_manipulation.core.rubik import rotations as rot

PITCH = 0.019
CORE_POSE = ((0.4, -0.1, 0.8), g.quat_from_rpy(0.3, -1.1, 2.0))


def axis_quat(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def poses_of(state, layer=None, angle=0.0):
    """Link poses of a cube in `state`; the cubies on the `layer` face are turned by `angle`
    about its outward normal, as a layer that is not square."""
    poses = {rt.CORE: CORE_POSE}
    for home, (slot, rotation) in cb.arrangement(state).items():
        pose = (tuple(PITCH * v for v in slot), g.quat_from_rpy(*rot.rpy_of(rotation)))
        if layer and sum(a * b for a, b in zip(c.NORMAL_OF[layer], slot)) == 1:
            pose = g.compose(((0.0, 0.0, 0.0), axis_quat(c.NORMAL_OF[layer], angle)), pose)
        poses[cb.name_of(home)] = g.compose(CORE_POSE, pose)
    return poses


@pytest.mark.parametrize('seed', range(6))
def test_reads_the_state_of_a_square_cube(seed):
    state = c.apply_moves(c.SOLVED, c.random_scramble(random.Random(seed)))
    truth = rt.read(poses_of(state), PITCH)
    assert truth.state == state and truth.error is None
    assert truth.position_error < 1e-12 and truth.angle_error < 1e-7
    assert truth.square(1e-6, 1e-6)


def test_a_layer_slightly_out_of_square_still_reads_and_is_measured():
    state = c.apply_moves(c.SOLVED, "R U F' L2")
    truth = rt.read(poses_of(state, 'R', 0.02), PITCH)
    assert truth.state == state
    assert truth.angle_error == pytest.approx(0.02, abs=1e-6)
    # a corner of the layer is sqrt(2) pitches from the axis
    assert truth.position_error == pytest.approx(0.02 * math.sqrt(2) * PITCH, rel=0.01)
    assert not truth.square(1e-4, 1e-3) and truth.square(1e-3, 0.03)


def test_a_layer_in_the_middle_of_a_turn_has_no_state():
    truth = rt.read(poses_of(c.SOLVED, 'U', math.radians(40)), PITCH)
    assert truth.state is None and truth.error
    assert not truth.square(1e-3, 0.03)


def test_a_quarter_turn_of_a_layer_reads_as_the_move():
    # U clockwise seen from outside is -90 degrees about its outward normal
    truth = rt.read(poses_of(c.SOLVED, 'U', -math.pi / 2), PITCH)
    assert truth.state == c.apply_move(c.SOLVED, 'U')
    assert truth.square(1e-9, 1e-7)


@pytest.mark.parametrize('face', c.FACES)
def test_layer_angle_is_the_spin_of_the_centre(face):
    for angle in (0.0, 0.3, -1.2, 3.0):
        assert rt.layer_angle(poses_of(c.SOLVED, face, angle), face) == pytest.approx(
            angle, abs=1e-9)
    other = 'U' if face in 'RLFB' else 'R'
    assert rt.layer_angle(poses_of(c.SOLVED, other, 0.5), face) == pytest.approx(0.0, abs=1e-9)


def test_missing_links_are_reported():
    poses = poses_of(c.SOLVED)
    del poses['cubie_ppp']
    truth = rt.read(poses, PITCH)
    assert truth.state is None and 'cubie_ppp' in truth.error
