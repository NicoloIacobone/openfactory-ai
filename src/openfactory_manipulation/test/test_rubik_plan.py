import random

import pytest

from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import orientation as ori
from openfactory_manipulation.core.rubik import plan, scan, view
from openfactory_manipulation.core.rubik import rotations as rot

KINDS = {'over': rot.quarter_turn(rot.Y, 1), 'side': rot.quarter_turn(rot.X, 1)}
COSTS = {'flip': 14.0, 'yaw': 5.0}
OPTIONS = plan.flip_options(KINDS, COSTS)
# sticker colours of the six faces, and how the same face looks in a view (darker, noisy)
REFERENCE = {'U': (255, 255, 255), 'R': (204, 0, 0), 'F': (0, 153, 38), 'D': (255, 217, 0),
             'L': (255, 102, 0), 'B': (0, 51, 204)}


def scrambled(seed):
    return c.apply_moves(c.SOLVED, c.random_scramble(random.Random(seed)))


def look(state, orientation, rng=None, gain=0.8):
    """The nine colours a view shows, as the camera would see them."""
    colours = []
    for index in view.top_facelets(orientation):
        rgb = REFERENCE[state[index]]
        noise = (rng.uniform(-8, 8) if rng else 0.0)
        colours.append(tuple(max(0.0, min(255.0, gain * v + noise)) for v in rgb))
    return tuple(colours)


def test_flip_kinds_of_the_cell():
    # 'over': the arm reaches over the nest; the face that looked away from the robot (+x)
    # comes down. 'side': the face that looked along -y comes down.
    assert rot.mat_vec(KINDS['over'], (1, 0, 0)) == ori.DOWN
    assert rot.mat_vec(KINDS['side'], (0, -1, 0)) == ori.DOWN
    assert len(OPTIONS) == 8


@pytest.mark.parametrize('orientation', ori.ORIENTATIONS)
def test_top_facelets_are_the_up_face_in_reading_order(orientation):
    indices = view.top_facelets(orientation)
    up = ori.up_face(orientation)
    start = 9 * c.FACES.index(up)
    assert sorted(indices) == list(range(start, start + 9))
    assert indices[4] == start + 4
    # the sticker seen at the +x, +y corner of the nest is the up face's corner there
    seen = c.SOLVED[indices[2]]
    assert seen == up


def test_scan_plan_shows_every_face_from_any_start():
    ops = plan.scan_plan(OPTIONS)
    assert 4 <= len(ops) <= 6
    for orientation in ori.ORIENTATIONS:
        seen = {ori.up_face(orientation)}
        for op in ops:
            _, orientation = plan.apply(c.SOLVED, orientation, op, KINDS)
            seen.add(ori.up_face(orientation))
        assert seen == set(c.FACES)


@pytest.mark.parametrize('seed', range(8))
def test_scan_reads_the_state_and_the_orientation(seed):
    rng = random.Random(seed)
    state = scrambled(seed)
    start = rng.choice(ori.ORIENTATIONS)
    orientation, turned = start, rot.IDENTITY
    views, seen = [(turned, look(state, orientation, rng))], {ori.up_face(orientation)}
    for op in plan.scan_plan(OPTIONS):
        _, after = plan.apply(state, orientation, op, KINDS)
        turned = rot.mat_mul(rot.mat_mul(after, rot.transpose(orientation)), turned)
        orientation = after
        if ori.up_face(orientation) not in seen:
            seen.add(ori.up_face(orientation))
            views.append((turned, look(state, orientation, rng)))
    result = scan.assemble(views, REFERENCE)
    assert result.get('error') is None
    assert result['state'] == state
    assert result['orientation'] == start
    assert set(result['centers']) == set(c.FACES)
    # the centres seen are what later views are read against
    assert scan.read_face(look(state, orientation, rng), result['centers']) == \
        scan.expected_face(state, orientation)


def test_scan_rejects_what_cannot_be_a_cube():
    state = scrambled(3)
    views, turned, orientation, seen = [], rot.IDENTITY, rot.IDENTITY, set()
    for op in [None] + plan.scan_plan(OPTIONS):
        if op is not None:
            _, after = plan.apply(state, orientation, op, KINDS)
            turned = rot.mat_mul(rot.mat_mul(after, rot.transpose(orientation)), turned)
            orientation = after
        if ori.up_face(orientation) not in seen:
            seen.add(ori.up_face(orientation))
            views.append((turned, look(state, orientation)))
    assert scan.assemble(views[:5], REFERENCE) == {'error': 'faces_missing'}
    # a view attributed to a flip that did not happen
    wrong = list(views)
    wrong[2] = (views[1][0], views[2][1])
    assert scan.assemble(wrong, REFERENCE)['error'] == 'inconsistent_views'
    # two stickers of one view swapped: an edge flipped, a corner twisted or pieces swapped
    stickers = list(views[0][1])
    stickers[1], stickers[3] = stickers[3], stickers[1]
    broken = scan.assemble([(views[0][0], tuple(stickers))] + views[1:], REFERENCE)
    if stickers[1] != stickers[3]:
        assert 'error' in broken and 'state' in broken


@pytest.mark.parametrize('seed', range(10))
def test_solve_plan_makes_the_moves(seed):
    rng = random.Random(100 + seed)
    state = scrambled(seed)
    moves = c.random_scramble(rng, 20)
    start = rng.choice(ori.ORIENTATIONS)
    ops, cost = plan.solve_plan(moves, start, OPTIONS, turn_cost=10.0)
    expected, orientation, made = state, start, []
    for op in ops:
        if op[0] == 'turn':
            made.append(ori.turn_move(orientation, op[1]))
        expected, orientation = plan.apply(expected, orientation, op, KINDS)
    assert made == moves
    assert expected == c.apply_moves(state, moves)
    flips = sum(op[0] == 'flip' for op in ops)
    # never more than two flips for a move, and on average well under two
    assert flips <= 2 * len(moves)
    assert flips <= 1.6 * len(moves)
    assert cost == pytest.approx(
        10.0 * len(moves) + sum(14.0 + (5.0 if op[2] else 0.0) for op in ops if op[0] == 'flip'))


def test_solve_plan_of_nothing_is_empty():
    assert plan.solve_plan([], rot.IDENTITY, OPTIONS) == ([], 0.0)


def test_view_problems():
    limits = {'seat_offset': 0.002, 'seat_height': 0.003, 'seat_tilt': 0.05, 'seat_yaw': 0.06,
              'min_score': 0.5}
    good = view.View(1.0, True, (0.0005, -0.0004, 0.0003), 0.02, 0.01, 0.9)
    assert view.problem(good, limits) is None
    assert view.problem(None, limits) == 'no_view'
    assert view.problem(view.View(1.0, False), limits) == 'no_cube'
    assert view.problem(view.View(1.0, True, (0.004, 0.0, 0.0), 0.0, 0.0, 0.9), limits) == 'not_seated'
    assert view.problem(view.View(1.0, True, (0.0, 0.0, 0.006), 0.0, 0.0, 0.9), limits) == 'not_seated'
    assert view.problem(view.View(1.0, True, (0.0, 0.0, 0.0), 0.2, 0.0, 0.9), limits) == 'not_seated'
    assert view.problem(view.View(1.0, True, (0.0, 0.0, 0.0), 0.0, 0.0, 0.2), limits) == 'not_square'
