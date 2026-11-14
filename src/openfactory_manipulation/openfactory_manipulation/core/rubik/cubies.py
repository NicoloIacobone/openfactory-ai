"""The cube as 26 rigid cubies: the view the simulator model and its evaluation need.

A cubie is named by its home, the slot (x, y, z) it occupies in the solved cube, in cubie units
of the cube frame (`cube.py`). An arrangement maps every home to (slot, rotation): where the
cubie is now and the rotation, one of the 24, that takes it there from home; a sticker whose
home normal is n now faces `rotation @ n`. Centres stay at home in the cube frame; how far a
centre has spun about its own axis does not show on its sticker and is left as the identity.
"""

from . import rotations as rot
from .cube import FACE_OF, NORMAL_OF, sticker_frames

_STICKERS = sticker_frames()
INDEX_OF = {sticker: i for i, sticker in enumerate(_STICKERS)}
HOMES = tuple(sorted({pos for pos, _ in _STICKERS}))
_HOME_SET = frozenset(HOMES)


def is_center(home):
    return sum(abs(c) for c in home) == 1


def normals_of(slot):
    """Outward normals of the faces a cubie in `slot` shows, in x, y, z order."""
    return tuple(tuple(c if j == i else 0 for j in range(3))
                 for i, c in enumerate(slot) if c != 0)


def name_of(home):
    """Link name of a cubie in the simulator model, e.g. `cubie_pzn` for (1, 0, -1)."""
    return 'cubie_' + ''.join('nzp'[c + 1] for c in home)


def home_of(name):
    return tuple('nzp'.index(ch) - 1 for ch in name.removeprefix('cubie_'))


def solved_arrangement():
    return {home: (home, rot.IDENTITY) for home in HOMES}


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def arrangement(state):
    """Arrangement of the 54-facelet `state`; ValueError if its pieces cannot exist."""
    result = {}
    for slot in HOMES:
        now = normals_of(slot)
        colours = ''.join(state[INDEX_OF[(slot, n)]] for n in now)
        was = tuple(NORMAL_OF[c] for c in colours)
        home = tuple(sum(c) for c in zip(*was))
        if home not in _HOME_SET or len(normals_of(home)) != len(now):
            raise ValueError(f'piece with impossible colours: {colours}')
        if home in result:
            raise ValueError(f'duplicate piece: {colours}')
        if len(now) == 1:
            if home != slot:
                raise ValueError(f'the centre at {slot} shows {colours}')
            rotation = rot.IDENTITY
        else:
            if len(now) == 2:
                now, was = now + (_cross(*now),), was + (_cross(*was),)
            # rotation @ was[i] = now[i]; `was` is orthonormal, so its inverse is its transpose
            rotation = rot.mat_mul(rot.from_columns(now), rot.transpose(rot.from_columns(was)))
            if rot.det(rotation) != 1:
                raise ValueError(f'mirrored piece: {colours}')
        result[home] = (slot, rotation)
    return result


def state_of(cubies):
    """54-facelet state of an arrangement."""
    cells = [''] * 54
    for home, (slot, rotation) in cubies.items():
        for normal in normals_of(home):
            cells[INDEX_OF[(slot, rot.mat_vec(rotation, normal))]] = FACE_OF[normal]
    if '' in cells:
        raise ValueError('the arrangement leaves facelets uncovered')
    return ''.join(cells)


def turn_layer(cubies, axis, turns, layers=(1,)):
    """Arrangement after the cubies whose slot coordinate along the signed unit `axis` is in
    `layers` turn by turns x 90 degrees about it (right-handed). Centres keep the identity."""
    turn = rot.quarter_turn(axis, turns)
    result = {}
    for home, (slot, rotation) in cubies.items():
        if sum(a * c for a, c in zip(axis, slot)) in layers:
            slot = rot.mat_vec(turn, slot)
            rotation = rot.IDENTITY if is_center(home) else rot.mat_mul(turn, rotation)
        result[home] = (slot, rotation)
    return result


def apply_move(cubies, face, turns):
    """`turns` clockwise quarter turns of `face`, on an arrangement."""
    return turn_layer(cubies, NORMAL_OF[face], -turns)
