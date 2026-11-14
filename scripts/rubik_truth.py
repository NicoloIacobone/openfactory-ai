"""Ground truth of the Rubik's cube from the simulator's link poses (harness only, rule 5).

Pure functions: the poses are read from Gazebo by the harness (`gz_scene.GzScene.model_poses`
with the link names of the cube model) and passed in as {name: (xyz, quaternion xyzw)}, all in
one frame. Nothing here is imported by perception or manipulation.
"""

import math
from dataclasses import dataclass

from openfactory_manipulation.core import geometry as g
from openfactory_manipulation.core.rubik import cubies as cb
from openfactory_manipulation.core.rubik import rotations as rot
from openfactory_manipulation.core.rubik.cube import NORMAL_OF

CORE = 'core'
LINKS = (CORE,) + tuple(cb.name_of(home) for home in cb.HOMES)


@dataclass(frozen=True)
class Truth:
    state: str | None            # 54 facelets in the cube frame; None while no state can be read
    error: str | None            # why not
    position_error: float        # m, largest distance of a cubie from the centre of its slot
    angle_error: float           # rad, largest angle of a cubie from the nearest of the 24 rotations
    cubies: dict                 # home -> (slot, rotation), as read (None if a pose is missing)

    def square(self, position_tolerance, angle_tolerance):
        """True if every cubie sits in a slot of the grid within the tolerances."""
        return (self.state is not None and self.position_error <= position_tolerance
                and self.angle_error <= angle_tolerance)


def read(poses, pitch):
    """State of the cube whose link poses are `poses`."""
    missing = [name for name in LINKS if name not in poses]
    if missing:
        return Truth(None, f'no pose for {missing[0]} and {len(missing) - 1} more',
                     math.inf, math.inf, {})
    cubies, position_error, angle_error = {}, 0.0, 0.0
    for home in cb.HOMES:
        xyz, quat = g.relative(poses[CORE], poses[cb.name_of(home)])
        slot = tuple(round(v / pitch) for v in xyz)
        position_error = max(position_error, math.dist(xyz, [pitch * c for c in slot]))
        rotation, angle = rot.nearest_rotation(rot.from_quaternion(quat))
        if cb.is_center(home):
            # a centre spins about its own axis with its layer: only its tilt is an error
            normal = home
            now = g.quat_rotate(quat, normal)
            angle = math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(now, normal)))))
            rotation = rot.IDENTITY
        angle_error = max(angle_error, angle)
        cubies[home] = (slot, rotation)
    try:
        state, error = cb.state_of(cubies), None
    except (ValueError, KeyError) as exc:
        state, error = None, f'cubies off the grid: {exc!r}'
    return Truth(state, error, position_error, angle_error, cubies)



def layer_angle(poses, face):
    """Angle in (-pi, pi] of the centre cubie of `face` about its outward normal, from home."""
    normal = NORMAL_OF[face]
    _, (x, y, z, w) = g.relative(poses[CORE], poses[cb.name_of(normal)])
    along = x * normal[0] + y * normal[1] + z * normal[2]
    return g.wrap_angle(2.0 * math.atan2(along, w))
