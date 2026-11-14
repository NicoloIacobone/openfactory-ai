"""What one view of the camera says about the cube in the nest, and which facelets it shows.

A view is what `cube_perception` publishes, expressed in the nest frame: where the cube is,
how its top face is turned and tilted, how confident perception is that it is a square block,
and the colours of the nine stickers of the top face as seen from above (row 0 on the +y side
of the nest, column 0 on its -x side).
"""

import math
from dataclasses import dataclass

from . import rotations as rot
from .cubies import INDEX_OF

UP = (0, 0, 1)


@dataclass(frozen=True)
class View:
    stamp: float                 # s, image stamp
    present: bool                # a cube stands in the nest
    offset: tuple = (0.0, 0.0, 0.0)   # m, cube centre minus its seat
    yaw: float = 0.0             # rad, top face about the vertical, within 45 degrees
    tilt: float = 0.0            # rad, top face from horizontal
    score: float = 0.0           # confidence that the cube is a square block
    stickers: tuple = ()         # nine (r, g, b), row by row


def problem(view, limits):
    """None if the view shows a cube seated and square in the nest, else why not:
    'no_view', 'no_cube', 'not_seated' or 'not_square'."""
    if view is None:
        return 'no_view'
    if not view.present:
        return 'no_cube'
    if (math.hypot(view.offset[0], view.offset[1]) > limits['seat_offset']
            or abs(view.offset[2]) > limits['seat_height']
            or view.tilt > limits['seat_tilt'] or abs(view.yaw) > limits['seat_yaw']):
        return 'not_seated'
    if view.score < limits['min_score']:
        return 'not_square'
    return None


def top_facelets(orientation):
    """Indices in the 54-facelet state of the nine stickers a view shows, in the order of
    `View.stickers`, for a cube seated in `orientation` (cube frame to nest frame)."""
    back = rot.transpose(orientation)
    normal = rot.mat_vec(back, UP)
    return tuple(INDEX_OF[(rot.mat_vec(back, (col - 1, 1 - row, 1)), normal)]
                 for row in range(3) for col in range(3))


def up_direction(rotation):
    """The direction, in the nest frame the cube started in, of the face that is up after
    the cube has been turned by `rotation` since then."""
    return rot.mat_vec(rot.transpose(rotation), UP)
