"""ROS-free generator of the Rubik's cube simulator model (phase 7, D-023).

The model is built from openfactory_description/config/rubik.yaml (passed in as a loaded dict)
and from a cube state, so that a cube can be spawned already scrambled: its cubies are placed
where that state has them. Model frame = cube frame (x towards R, y towards U, z towards F).

    core               the root link, at the cube centre
    axle_<face>        six links on revolute joints about the outward face normals
    cubie_<xyz>        26 free links named after their home slot (`cubies.name_of`)

The links are joined at run time by the model's system plugin (RubikCubeSystem in
openfactory_simulation), which welds every cubie to the axle of its layer or to the core.
"""

from xml.sax.saxutils import escape

from openfactory_manipulation.core.rubik import cubies as cb
from openfactory_manipulation.core.rubik import rotations as rot
from openfactory_manipulation.core.rubik.cube import FACE_OF, NORMAL_OF, SOLVED

MODEL_NAME = 'rubik_cube'
PLUGIN_FILE = 'RubikCubeSystem'
PLUGIN_NAME = 'openfactory::RubikCubeSystem'
# Order in which the plugin numbers the axles: +x, -x, +y, -y, +z, -z
AXLE_FACES = ('R', 'L', 'U', 'D', 'F', 'B')
# Parameters of `mechanism` in rubik.yaml that the plugin reads from its SDF element
PLUGIN_PARAMS = ('breakaway_torque', 'detent_torque', 'detent_stiffness', 'detent_window',
                 'snap_tolerance', 'initial_axis')


def _fmt(values):
    return ' '.join(f'{v:.9g}' for v in values)


def side(config):
    """Side of the whole cube in metres."""
    return 3 * config['cube']['pitch']


def mass(config):
    cube = config['cube']
    return 26 * cube['cubie_mass'] + cube['core_mass'] + 6 * cube['axle_mass']


def cubie_pose(config, slot, rotation):
    """Pose (x, y, z, roll, pitch, yaw) of a cubie in the model frame."""
    pitch = config['cube']['pitch']
    return [pitch * c for c in slot] + list(rot.rpy_of(rotation))


def _inertial(mass_kg, moment):
    return (f'<inertial><mass>{mass_kg:.9g}</mass><inertia><ixx>{moment:.9g}</ixx><ixy>0</ixy>'
            f'<ixz>0</ixz><iyy>{moment:.9g}</iyy><iyz>0</iyz><izz>{moment:.9g}</izz></inertia>'
            '</inertial>')


def _material(color):
    rgba = _fmt(color)
    return (f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
            '<specular>0 0 0 1</specular></material>')


def _sticker(config, normal):
    cube = config['cube']
    size, thickness = cube['sticker']['size'], cube['sticker']['thickness']
    offset = cube['pitch'] / 2 + thickness / 2
    box = [thickness if c else size for c in normal]
    name = 'sticker_' + FACE_OF[normal]
    return (f'<visual name="{name}"><pose>{_fmt([offset * c for c in normal])} 0 0 0</pose>'
            f'<geometry><box><size>{_fmt(box)}</size></box></geometry>'
            f'{_material(cube["colors"][FACE_OF[normal]])}</visual>')


def _cubie(config, home, slot, rotation):
    cube = config['cube']
    pitch, friction = cube['pitch'], cube['friction']
    box = f'<geometry><box><size>{_fmt([pitch] * 3)}</size></box></geometry>'
    stickers = ''.join(_sticker(config, normal) for normal in cb.normals_of(home))
    return f"""
    <link name="{cb.name_of(home)}">
      <pose>{_fmt(cubie_pose(config, slot, rotation))}</pose>
      {_inertial(cube['cubie_mass'], cube['cubie_mass'] * pitch * pitch / 6)}
      <collision name="collision">{box}
        <surface><friction><ode><mu>{friction:.6g}</mu><mu2>{friction:.6g}</mu2></ode></friction></surface>
      </collision>
      <visual name="body">{box}{_material(cube['body_color'])}</visual>
      {stickers}
    </link>"""


def _axle(config, face):
    cube, mech = config['cube'], config['mechanism']
    radius = cube['pitch'] / 2
    friction = mech['layer_friction']
    return f"""
    <link name="axle_{face}">
      {_inertial(cube['axle_mass'], 0.4 * cube['axle_mass'] * radius * radius)}
    </link>
    <joint name="axle_{face}_joint" type="revolute">
      <parent>core</parent><child>axle_{face}</child>
      <axis>
        <xyz>{_fmt(NORMAL_OF[face])}</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><friction>{friction:.9g}</friction><damping>{mech['layer_damping']:.9g}</damping></dynamics>
      </axis>
    </joint>"""


def cube_sdf(config, state=SOLVED, name=MODEL_NAME, fixed_core=False):
    """Complete SDF document of a cube in `state`.

    `fixed_core` joins the core to the world: the mechanism test bench, where faces are turned
    by torques applied to their centre cubies and nothing else holds the cube.
    """
    cube, mech = config['cube'], config['mechanism']
    radius = cube['pitch'] / 2
    links = ''.join(_axle(config, face) for face in AXLE_FACES)
    for home, (slot, rotation) in sorted(cb.arrangement(state).items()):
        links += _cubie(config, home, slot, rotation)
    fix = ('\n    <joint name="bench_fix" type="fixed"><parent>world</parent><child>core</child>'
           '</joint>' if fixed_core else '')
    params = ''.join(f'\n      <{key}>{mech[key]:.9g}</{key}>' for key in PLUGIN_PARAMS)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{escape(name)}">
    <self_collide>false</self_collide>
    <link name="core">
      {_inertial(cube['core_mass'], 0.4 * cube['core_mass'] * radius * radius)}
    </link>{fix}{links}
    <plugin filename="{PLUGIN_FILE}" name="{PLUGIN_NAME}">
      <pitch>{cube['pitch']:.9g}</pitch>{params}
    </plugin>
  </model>
</sdf>
"""
