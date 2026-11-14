"""Pure model of the 3x3x3 cube as 54 facelets (ported from the `doppelbot` project).

State: a string of 54 characters, faces in the order U R F D L B, 9 facelets per face row by
row (Kociemba's convention). Each face is read looking at it from outside: U with B at the top,
D with F at the top, R F L B with U at the top. A facelet holds the letter of the face its
colour belongs to, so the solved cube is `SOLVED`.

Cube frame: x towards R, y towards U, z towards F, origin at the cube centre, coordinates in
cubie units. The six centres never move in this frame: it is the frame of the cube's core.

Moves: a letter is a quarter turn of that face clockwise seen from outside, `'` anticlockwise,
`2` a half turn. The permutations are derived from the geometry (position and normal of every
sticker), not written by hand.
"""

import random

FACES = 'URFDLB'
SOLVED = ''.join(face * 9 for face in FACES)
CENTER_INDICES = tuple(9 * i + 4 for i in range(6))

# Per face: outward normal, direction of the columns (left to right) and of the rows (top to
# bottom), seen from outside.
_FACE_FRAMES = {
    'U': ((0, 1, 0), (1, 0, 0), (0, 0, 1)),
    'R': ((1, 0, 0), (0, 0, -1), (0, -1, 0)),
    'F': ((0, 0, 1), (1, 0, 0), (0, -1, 0)),
    'D': ((0, -1, 0), (1, 0, 0), (0, 0, -1)),
    'L': ((-1, 0, 0), (0, 0, 1), (0, -1, 0)),
    'B': ((0, 0, -1), (-1, 0, 0), (0, -1, 0)),
}
NORMAL_OF = {face: frame[0] for face, frame in _FACE_FRAMES.items()}
FACE_OF = {normal: face for face, normal in NORMAL_OF.items()}

_SUFFIX_TURNS = {'': 1, "'": 3, '2': 2}
SUFFIX_OF = {turns: suffix for suffix, turns in _SUFFIX_TURNS.items()}


def _add(*vectors):
    return tuple(sum(components) for components in zip(*vectors))


def _scale(k, v):
    return tuple(k * c for c in v)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def sticker_frames():
    """(position, normal) of each of the 54 stickers, in the order of the state string."""
    stickers = []
    for face in FACES:
        normal, col_dir, row_dir = _FACE_FRAMES[face]
        for row in range(3):
            for col in range(3):
                pos = _add(normal, _scale(col - 1, col_dir), _scale(row - 1, row_dir))
                stickers.append((pos, normal))
    return stickers


def _rotate_clockwise(v, axis):
    """Quarter turn clockwise seen from the tip of `axis` (-90 degrees about the axis)."""
    return _add(_scale(-1, _cross(axis, v)), _scale(_dot(axis, v), axis))


def _quarter_turn_permutation(face):
    """perm[i] = index where sticker i ends after a clockwise quarter turn of `face`."""
    stickers = sticker_frames()
    index_of = {sticker: i for i, sticker in enumerate(stickers)}
    axis = NORMAL_OF[face]
    perm = list(range(54))
    for i, (pos, normal) in enumerate(stickers):
        if _dot(pos, axis) == 1:
            perm[i] = index_of[(_rotate_clockwise(pos, axis), _rotate_clockwise(normal, axis))]
    return tuple(perm)


_QUARTER_TURNS = {face: _quarter_turn_permutation(face) for face in FACES}


def parse_move(move):
    """'R' -> ('R', 1), "U'" -> ('U', 3), 'F2' -> ('F', 2): face and clockwise quarter turns."""
    if not move or move[0] not in FACES or move[1:] not in _SUFFIX_TURNS:
        raise ValueError(f'invalid move: {move!r}')
    return move[0], _SUFFIX_TURNS[move[1:]]


def format_move(face, turns):
    """Inverse of parse_move; `turns` is taken modulo 4 and must not be a multiple of 4."""
    return face + SUFFIX_OF[turns % 4]


def parse_moves(sequence):
    """Space-separated sequence such as "R U R' U'"; every move is validated."""
    moves = sequence.split()
    for move in moves:
        parse_move(move)
    return moves


def affected_indices(move):
    """Indices of the stickers a move displaces: 8 of the face and a row of 3 on each neighbour."""
    face, _ = parse_move(move)
    perm = _QUARTER_TURNS[face]
    return frozenset(i for i in range(54) if perm[i] != i)


def validate_state(state):
    """Basic checks: 54 characters, only the letters URFDLB, 9 of each, centres at home."""
    if len(state) != 54:
        raise ValueError(f'the state must have 54 characters, it has {len(state)}')
    unknown = set(state) - set(FACES)
    if unknown:
        raise ValueError(f'invalid characters: {"".join(sorted(unknown))}')
    for face in FACES:
        if state.count(face) != 9:
            raise ValueError(f'colour {face} appears {state.count(face)} times instead of 9')
    for face, index in zip(FACES, CENTER_INDICES):
        if state[index] != face:
            raise ValueError(f'the centre of face {face} is {state[index]}')


def _cubie_stickers():
    """Sticker indices of every corner and edge, the reference sticker first.

    Reference: the U/D sticker if the piece has one, otherwise F/B (Kociemba's convention).
    The other two stickers of a corner follow anticlockwise seen from outside.
    """
    frames = sticker_frames()
    by_position = {}
    for i, (pos, _) in enumerate(frames):
        by_position.setdefault(pos, []).append(i)
    corners, edges = [], []
    for pos in sorted(by_position):
        indices = by_position[pos]
        if len(indices) == 3:
            ref = next(i for i in indices if frames[i][1][1] != 0)
            a, b = (i for i in indices if i != ref)
            if _dot(frames[ref][1], _cross(frames[a][1], frames[b][1])) < 0:
                a, b = b, a
            corners.append((ref, a, b))
        elif len(indices) == 2:
            ref = next((i for i in indices if frames[i][1][1] != 0), None)
            if ref is None:
                ref = next(i for i in indices if frames[i][1][2] != 0)
            edges.append((ref, next(i for i in indices if i != ref)))
    return tuple(corners), tuple(edges)


_CORNERS, _EDGES = _cubie_stickers()


def _permutation_parity(perm):
    seen, cycles = set(), 0
    for start in range(len(perm)):
        if start not in seen:
            cycles += 1
            i = start
            while i not in seen:
                seen.add(i)
                i = perm[i]
    return (len(perm) - cycles) % 2


def _piece_key(colors):
    """Identity of a piece from its colours, whatever its orientation.

    For corners the cyclic order counts: a corner with two colours swapped is its mirror image,
    which does not exist on a real cube.
    """
    if len(colors) == 2:
        return frozenset(colors)
    return min(colors[k:] + colors[:k] for k in range(len(colors)))


def _identify(state, pieces, kind):
    """For every position, the index of the piece (in the solved cube) that sits there."""
    home = {_piece_key(tuple(SOLVED[i] for i in piece)): n for n, piece in enumerate(pieces)}
    perm = []
    for piece in pieces:
        colors = _piece_key(tuple(state[i] for i in piece))
        if colors not in home:
            raise ValueError(
                f'{kind} with impossible colours: {"".join(state[i] for i in piece)}')
        perm.append(home[colors])
    if len(set(perm)) != len(perm):
        raise ValueError(f'duplicate {kind}: two pieces have the same colours')
    return perm


def check_solvable(state):
    """Raise ValueError, with the reason, if the state cannot be reached by turning faces.

    On top of validate_state: existing pieces without duplicates, total corner twist a multiple
    of 3, an even number of flipped edges, the same parity for corners and edges.
    """
    validate_state(state)
    corner_perm = _identify(state, _CORNERS, 'corner')
    edge_perm = _identify(state, _EDGES, 'edge')
    twist = sum(next(k for k, i in enumerate(piece) if state[i] in 'UD') for piece in _CORNERS)
    if twist % 3:
        raise ValueError('a corner is twisted in place (total twist not a multiple of 3)')
    flips = 0
    for ref, other in _EDGES:
        primary = 'UD' if {state[ref], state[other]} & set('UD') else 'FB'
        flips += state[ref] not in primary
    if flips % 2:
        raise ValueError('an edge is flipped in place')
    if _permutation_parity(corner_perm) != _permutation_parity(edge_perm):
        raise ValueError('two pieces are swapped (corners and edges have different parity)')


def apply_move(state, move):
    face, turns = parse_move(move)
    perm = _QUARTER_TURNS[face]
    cells = list(state)
    for _ in range(turns):
        moved = [''] * 54
        for i, target in enumerate(perm):
            moved[target] = cells[i]
        cells = moved
    return ''.join(cells)


def apply_moves(state, sequence):
    """Apply a space-separated sequence, or a list of moves."""
    moves = parse_moves(sequence) if isinstance(sequence, str) else sequence
    for move in moves:
        state = apply_move(state, move)
    return state


def is_solved(state):
    return state == SOLVED


def random_scramble(rng: random.Random, length=25):
    """Seeded random scramble; two consecutive moves never turn the same face."""
    moves = []
    while len(moves) < length:
        move = rng.choice(FACES) + rng.choice(tuple(_SUFFIX_TURNS))
        if not moves or moves[-1][0] != move[0]:
            moves.append(move)
    return moves
