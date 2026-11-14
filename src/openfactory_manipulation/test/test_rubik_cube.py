import random

import pytest

from openfactory_manipulation.core.rubik import cube as c

ALL_MOVES = [face + suffix for face in c.FACES for suffix in ('', "'", '2')]
UF, UR = (7, 19), (5, 10)


def face_slice(state, face):
    start = 9 * c.FACES.index(face)
    return state[start:start + 9]


def swap(state, *pairs):
    cells = list(state)
    for a, b in pairs:
        cells[a], cells[b] = cells[b], cells[a]
    return ''.join(cells)


def test_r_from_solved_matches_kociemba_reference():
    expected = 'UUFUUFUUF' 'RRRRRRRRR' 'FFDFFDFFD' 'DDBDDBDDB' 'LLLLLLLLL' 'UBBUBBUBB'
    assert c.apply_move(c.SOLVED, 'R') == expected


def test_u_moves_the_front_top_row_to_the_left():
    state = c.apply_move(c.SOLVED, 'U')
    assert face_slice(state, 'F')[:3] == 'RRR'
    assert face_slice(state, 'L')[:3] == 'FFF'
    assert face_slice(state, 'B')[:3] == 'LLL'
    assert face_slice(state, 'R')[:3] == 'BBB'
    assert face_slice(state, 'U') == 'U' * 9


@pytest.mark.parametrize('face', c.FACES)
def test_turns_compose(face):
    state = c.apply_moves(c.SOLVED, "R U F' L2 D B'")
    assert c.apply_moves(state, ' '.join([face] * 4)) == state
    assert c.apply_moves(state, f"{face} {face}'") == state
    assert c.apply_move(state, face + '2') == c.apply_moves(state, [face, face])


def test_sexy_move_six_times_is_identity():
    assert c.apply_moves(c.SOLVED, ' '.join(["R U R' U'"] * 6)) == c.SOLVED


@pytest.mark.parametrize('move', ALL_MOVES)
def test_move_changes_only_its_layer(move):
    state = c.apply_moves(c.SOLVED, "R U2 F' L D' B2 R'")
    after = c.apply_move(state, move)
    layer = c.affected_indices(move)
    assert {i for i in range(54) if state[i] != after[i]} <= layer
    assert len(layer) == 20
    c.validate_state(after)


@pytest.mark.parametrize('move', ALL_MOVES)
def test_format_move_inverts_parse_move(move):
    assert c.format_move(*c.parse_move(move)) == move
    face, turns = c.parse_move(move)
    assert c.format_move(face, turns - 4) == move


@pytest.mark.parametrize('bad', ['', 'X', 'R3', 'r', "U''", "F2'"])
def test_parse_move_rejects_bad_moves(bad):
    with pytest.raises(ValueError):
        c.parse_move(bad)


@pytest.mark.parametrize('bad', [
    c.SOLVED[:-1],
    c.SOLVED[:-1] + 'X',
    'R' + c.SOLVED[1:],
    c.SOLVED[:4] + 'R' + c.SOLVED[5:13] + 'U' + c.SOLVED[14:],
])
def test_validate_state_rejects_bad_states(bad):
    with pytest.raises(ValueError):
        c.validate_state(bad)


def test_random_scrambles_are_seeded_and_solvable():
    assert c.random_scramble(random.Random(3)) == c.random_scramble(random.Random(3))
    rng = random.Random(7)
    for _ in range(200):
        scramble = c.random_scramble(rng)
        assert len(scramble) == 25
        assert all(a[0] != b[0] for a, b in zip(scramble, scramble[1:]))
        c.check_solvable(c.apply_moves(c.SOLVED, scramble))


def test_twisted_corner_rejected():
    cells = list(c.SOLVED)
    cells[8], cells[9], cells[20] = 'F', 'U', 'R'
    with pytest.raises(ValueError, match='corner'):
        c.check_solvable(''.join(cells))


def test_flipped_edge_rejected():
    with pytest.raises(ValueError, match='edge'):
        c.check_solvable(swap(c.SOLVED, UF))


def test_two_swapped_edges_rejected():
    with pytest.raises(ValueError, match='parity'):
        c.check_solvable(swap(c.SOLVED, (UF[0], UR[0]), (UF[1], UR[1])))


def test_impossible_piece_rejected():
    # U and D on the same edge: a piece that does not exist
    with pytest.raises(ValueError, match='impossible'):
        c.check_solvable(swap(c.SOLVED, (19, 28)))


def test_solver_solves_random_scrambles():
    pytest.importorskip('kociemba')
    from openfactory_manipulation.core.rubik.solver import solve

    assert solve(c.SOLVED) == []
    rng = random.Random(5)
    for _ in range(20):
        state = c.apply_moves(c.SOLVED, c.random_scramble(rng))
        moves = solve(state)
        assert c.apply_moves(state, moves) == c.SOLVED
        assert len(moves) <= 25
    with pytest.raises(ValueError):
        solve(swap(c.SOLVED, UF))
