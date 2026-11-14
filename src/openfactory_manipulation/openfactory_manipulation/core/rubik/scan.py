"""From six views of the cube to its state (phase 7, D-026).

The cell does not know how the cube sits at the start. It looks at the face that is up, flips
the cube, looks again, until it has seen the six faces; for every view it knows only how it
has turned the cube since the first one. The centre stickers say which face each view shows
(their colours are those of the cube model, fixed infrastructure), and from that the
orientation at the start; every other sticker takes the letter of the centre its colour is
closest to, nine per face at most. The result must be a state a cube can be in.
"""

import itertools
import math

from . import rotations as rot
from .cube import FACES, NORMAL_OF, check_solvable
from .view import top_facelets, up_direction


def distance(a, b):
    return math.dist(a, b)


def name_centers(centers, reference):
    """Face letter of each of up to six centre colours: the assignment of distinct letters
    closest to the `reference` colours ({letter: rgb}) as a whole."""
    best = min(itertools.permutations(FACES, len(centers)),
               key=lambda names: sum(distance(c, reference[n]) for c, n in zip(centers, names)))
    return list(best)


def start_orientation(turned, letters):
    """Orientation of the cube at the first view, from the face letter each view showed up
    and how the cube had been turned since the first view (`turned`, rotations of the nest
    frame). None if no orientation agrees with every view."""
    matches = [o for o in rot.ROTATIONS
               if all(rot.mat_vec(o, NORMAL_OF[letter]) == up_direction(q)
                      for q, letter in zip(turned, letters))]
    return matches[0] if len(matches) == 1 else None


def classify(samples, centers):
    """Letter of each of 54 colours, given the colour of the six centres ({letter: rgb}):
    the nearest centre, at most nine per letter, closest pairs first. Returns (state,
    indices whose nearest centre was full or nearly tied with the second)."""
    order = list(FACES)
    distances = [[distance(s, centers[f]) for f in order] for s in samples]
    assignment, counts = [None] * 54, [0] * 6
    for d, i, f in sorted((distances[i][f], i, f) for i in range(54) for f in range(6)):
        if assignment[i] is None and counts[f] < 9:
            assignment[i] = f
            counts[f] += 1
    doubtful = []
    for i in range(54):
        nearest, second = sorted(range(6), key=lambda f: distances[i][f])[:2]
        if assignment[i] != nearest or distances[i][nearest] > 0.6 * distances[i][second]:
            doubtful.append(i)
    return ''.join(order[f] for f in assignment), doubtful


def assemble(views, reference):
    """State of the cube from six views.

    `views`: [(rotation of the cube since the first view, nine sticker colours)], one per
    face; `reference`: {letter: rgb} of the cube model. Returns a dict with `state`,
    `orientation` (at the first view), `centers` (the colours the centres were seen with) and
    `doubtful`, or with `error`: 'faces_missing', 'inconsistent_views', or the reason the
    state cannot exist.
    """
    if len(views) != 6:
        return {'error': 'faces_missing'}
    turned = [q for q, _ in views]
    letters = name_centers([stickers[4] for _, stickers in views], reference)
    if len(set(letters)) != 6:
        return {'error': 'faces_missing'}
    orientation = start_orientation(turned, letters)
    if orientation is None:
        return {'error': 'inconsistent_views'}
    samples = [None] * 54
    for (q, stickers), letter in zip(views, letters):
        for index, colour in zip(top_facelets(rot.mat_mul(q, orientation)), stickers):
            samples[index] = colour
    if any(s is None for s in samples):
        return {'error': 'inconsistent_views'}
    centers = {letter: stickers[4] for (_, stickers), letter in zip(views, letters)}
    state, doubtful = classify(samples, centers)
    try:
        check_solvable(state)
    except ValueError as error:
        return {'error': str(error), 'state': state, 'doubtful': doubtful}
    return {'state': state, 'orientation': orientation, 'centers': centers,
            'doubtful': doubtful}


def read_face(stickers, centers):
    """Letters of the nine stickers of one view, each the nearest of the six `centers`."""
    return ''.join(min(FACES, key=lambda f: distance(colour, centers[f])) for colour in stickers)


def expected_face(state, orientation):
    """Letters a view must show for a cube in `state` seated in `orientation`."""
    return ''.join(state[i] for i in top_facelets(orientation))
