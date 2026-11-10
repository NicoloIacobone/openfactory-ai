"""ROS-free scene helpers: object catalog geometry, generated SDF models, spawn poses.

The catalog is openfactory_description/config/objects.yaml and the cell layout is
openfactory_description/config/cell.yaml; both are passed in as loaded dicts. Catalog geometry
and quaternions come from openfactory_manipulation.core (ROS-free), so the simulator models and
the manipulation code share one implementation.
"""

import math
from xml.sax.saxutils import escape

from openfactory_manipulation.core.geometry import quat_from_rpy as quaternion_from_rpy  # noqa: F401
from openfactory_manipulation.core.objects import SHAPES, bounding_box  # noqa: F401
from openfactory_manipulation.core.objects import prism_height as _prism_height


def inertia(spec):
    """(centre of mass [x, y, z] in the object frame, [ixx, iyy, izz] about it)."""
    m, shape = spec['mass'], spec['shape']
    if shape == 'box':
        x, y, z = spec['size']
        return [0.0, 0.0, 0.0], [m * (y * y + z * z) / 12, m * (x * x + z * z) / 12,
                                 m * (x * x + y * y) / 12]
    if shape == 'cylinder':
        r, h = spec['radius'], spec['length']
        side = m * (3 * r * r + h * h) / 12
        return [0.0, 0.0, 0.0], [side, side, m * r * r / 2]
    if shape == 'triangular_prism':
        s, length, h = spec['side'], spec['length'], _prism_height(spec)
        # equilateral cross-section: polar moment m s^2/12 about the centroid, half per in-plane
        # axis; the centroid is h/3 above the base, the frame origin h/2 above it
        across = m * (s * s / 24 + length * length / 12)
        return [0.0, 0.0, -h / 6], [m * s * s / 12, across, across]
    raise ValueError(f'unknown shape {shape!r}')


def _fmt(values):
    return ' '.join(f'{v:.6g}' for v in values)


def geometry_sdf(spec):
    """(pose of the geometry in the link frame as an SDF pose string, <geometry> inner XML)."""
    shape = spec['shape']
    if shape == 'box':
        return '0 0 0 0 0 0', f"<box><size>{_fmt(spec['size'])}</size></box>"
    if shape == 'cylinder':
        return '0 0 0 0 0 0', (f"<cylinder><radius>{spec['radius']:.6g}</radius>"
                               f"<length>{spec['length']:.6g}</length></cylinder>")
    if shape == 'triangular_prism':
        # SDF extrudes a polyline along its own z from 0 to `height`. Rotating it by
        # rpy (pi/2, 0, pi/2) maps polyline x -> object y, y -> object z, z -> object x;
        # the x offset centres the extrusion on the object origin.
        s, h, length = spec['side'], _prism_height(spec), spec['length']
        points = [(-s / 2, -h / 2), (s / 2, -h / 2), (0.0, h / 2)]
        xml = ''.join(f'<point>{_fmt(p)}</point>' for p in points)
        pose = _fmt([-length / 2, 0.0, 0.0, math.pi / 2, 0.0, math.pi / 2])
        return pose, f'<polyline>{xml}<height>{length:.6g}</height></polyline>'
    raise ValueError(f'unknown shape {shape!r}')


def model_sdf(model_name, spec, friction):
    """Complete SDF document for one dynamic object model."""
    com, (ixx, iyy, izz) = inertia(spec)
    pose, geometry = geometry_sdf(spec)
    color = _fmt(spec['color'])
    name = escape(model_name)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <pose>{_fmt(com)} 0 0 0</pose>
        <mass>{spec['mass']:.6g}</mass>
        <inertia>
          <ixx>{ixx:.6g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.6g}</iyy><iyz>0</iyz><izz>{izz:.6g}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>{pose}</pose>
        <geometry>{geometry}</geometry>
        <surface>
          <friction><ode><mu>{friction:.6g}</mu><mu2>{friction:.6g}</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <pose>{pose}</pose>
        <geometry>{geometry}</geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def bin_origin(cell, bin_name):
    """World position of a bin frame (centre of its inner floor); bins are not rotated."""
    table = cell['table']
    spec = cell['bins'][bin_name]
    return [
        table['xyz'][0] + spec['xy'][0],
        table['xyz'][1] + spec['xy'][1],
        table['xyz'][2] + table['size'][2] + cell['bin_wall'],
    ]


def spawn_pose(cell, bin_name, spec, x, y, yaw, drop=0.002):
    """World (xyz, yaw) for an object resting at (x, y, yaw) in a bin frame, `drop` above the floor."""
    inner = cell['bins'][bin_name]['inner_size']
    if abs(x) > inner[0] / 2 or abs(y) > inner[1] / 2:
        raise ValueError(f'({x}, {y}) is outside the {bin_name} bin')
    ox, oy, oz = bin_origin(cell, bin_name)
    return [ox + x, oy + y, oz + bounding_box(spec)[2] / 2 + drop], yaw

