"""Ring of N arms and N bins (phase 6, D-020): layout and names. ROS-free.

Arm i picks from bin i and places into bin (i + 1) mod N. Every arm sees its two bins exactly as
the single arm of the cell sees `input_bin` and `output_bin` (config/cell.yaml): at `standoff`
in front of its base and half a `chord` to each side, the source on its right and the
destination on its left. So what was measured on the cell (reach, ready configuration, place
points) holds for every arm of every ring.

Geometry, seen from above with the ring centre at the world origin:
- the bins are the vertices of a regular N-gon of side `chord`:
      r_bin = chord / (2 sin(pi / N))
- the arms stand outside it, on the axis of each side, facing the centre:
      r_arm = chord / (2 tan(pi / N)) + standoff
- neighbouring arm bases are chord cos(pi / N) + 2 standoff sin(pi / N) apart, which never
  falls below `chord`: the geometry sets no upper limit on N.
Bins advance clockwise (the destination is on an arm's left while it faces the centre). N = 2
is the cell with a second arm facing the first across the two bins.
"""

import math
import tempfile
from pathlib import Path

PACKAGE = 'openfactory_description'
ARM_JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)
FINGER_JOINTS = ('left_finger_joint', 'right_finger_joint')
TABLE_LINK, FLOOR_LINK = 'table', 'floor'


# -- names ------------------------------------------------------------------------------------

def arm_name(index):
    return f'arm_{index}'


def arm_prefix(index):
    """Prefix of every link and joint of arm `index`."""
    return f'arm_{index}_'


def arm_joints(index):
    return [arm_prefix(index) + name for name in ARM_JOINTS]


def finger_joints(index):
    return [arm_prefix(index) + name for name in FINGER_JOINTS]


def planning_group(index):
    return f'arm_{index}_manipulator'


def gripper_group(index):
    return f'arm_{index}_gripper'


def trajectory_controller(index):
    return f'arm_{index}_joint_trajectory_controller'


def gripper_controller(index):
    return f'arm_{index}_gripper_controller'


def bin_name(index):
    """Link and frame of bin `index`: the centre of its inner floor surface."""
    return f'bin_{index}'


def camera_prefix(index):
    """Prefix of the frames of the camera above bin `index` (`<prefix>camera_link`, ...)."""
    return f'bin_{index}_'


def bin_namespace(index):
    """ROS namespace of the camera and perception topics of bin `index`."""
    return f'/bin_{index}'


def bin_topic(index, topic):
    """A camera or perception topic of the single-arm cell (`/camera/...`,
    `/perception/...`) as the topic of bin `index`."""
    return bin_namespace(index) + topic


def source_bin(arm):
    return arm


def destination_bin(arm, n):
    return (arm + 1) % n


# -- layout -----------------------------------------------------------------------------------

def check_n(n):
    if not isinstance(n, int) or isinstance(n, bool) or n < 2:
        raise ValueError(f'a ring needs an integer number of arms >= 2, not {n!r}')
    return n


def cell_geometry(cell):
    """(chord, standoff) of the single-arm cell: the distance between its two bins and the
    distance from the robot base to the line that joins them."""
    source, destination = cell['bins']['input'], cell['bins']['output']
    mount = cell['robot_mount']
    if any(abs(v) > 1e-9 for v in mount['rpy']):
        raise ValueError('the ring layout needs a robot mount without rotation')
    if abs(source['xy'][0] - destination['xy'][0]) > 1e-9:
        raise ValueError('the ring layout needs both bins at the same distance in front')
    if abs(source['xy'][1] + destination['xy'][1] - 2 * mount['xyz'][1]) > 1e-9:
        raise ValueError('the ring layout needs the bins symmetric about the robot x axis')
    if source['xy'][1] > destination['xy'][1]:
        raise ValueError('the ring layout needs the input bin on the robot right (-y)')
    if source['inner_size'] != destination['inner_size']:
        raise ValueError('the ring layout needs bins of one size')
    chord = destination['xy'][1] - source['xy'][1]
    standoff = source['xy'][0] - mount['xyz'][0]
    return chord, standoff


def radii(n, chord, standoff):
    """(r_bin, r_arm) of a ring of `n` arms."""
    check_n(n)
    return chord / (2 * math.sin(math.pi / n)), chord / (2 * math.tan(math.pi / n)) + standoff


def arm_spacing(n, chord, standoff):
    """Distance between the bases of two neighbouring arms."""
    return 2 * radii(n, chord, standoff)[1] * math.sin(math.pi / n)


def layout(n, cell, ring):
    """Poses of the arms, the bins, their cameras and the table of a ring of `n` arms.

    `cell`: config/cell.yaml, the source of the local geometry; `ring`: config/ring.yaml.
    Returns plain lists and floats (it is written to YAML for xacro): world x, y on the table
    top and yaw about z for every arm base and bin frame; the camera mount in its bin frame.
    """
    chord, standoff = cell_geometry(cell)
    r_bin, r_arm = radii(n, chord, standoff)
    wall = cell['bin_wall']
    mount = cell['camera_mount']
    source = cell['bins']['input']
    if any(abs(a - b) > 1e-9 for a, b in zip(mount['xyz'][:2], source['xy'])):
        raise ValueError('the ring layout needs the camera straight above the input bin')

    arms, bins = [], []
    for index in range(n):
        along = -2 * math.pi * index / n                 # bins advance clockwise
        bins.append({
            'name': bin_name(index),
            'xy': [r_bin * math.cos(along), r_bin * math.sin(along)],
            # frame x along the ring, as the cell's bins have x along the robot x at N = 2
            'yaw': _wrap(along + math.pi / 2),
            'camera_prefix': camera_prefix(index),
            'camera_topic': f'bin_{index}/camera',
        })
        between = -2 * math.pi * (index + 0.5) / n       # between bin i and bin i + 1
        arms.append({
            'name': arm_name(index),
            'prefix': arm_prefix(index),
            'xy': [r_arm * math.cos(between), r_arm * math.sin(between)],
            'yaw': _wrap(between + math.pi),             # facing the centre
            'source_bin': bin_name(source_bin(index)),
            'destination_bin': bin_name(destination_bin(index, n)),
        })
    table_radius = r_arm + ring['table_margin']
    return {
        'n': n,
        'chord': chord,
        'standoff': standoff,
        'table': {'radius': table_radius,
                  'height': cell['table']['xyz'][2] + cell['table']['size'][2]},
        'floor_size': 2 * (table_radius + ring['floor_margin']),
        'mount_z': mount_z(cell),
        'bin_wall': wall,
        'bin_inner_size': list(source['inner_size']),
        # in the frame of its bin: straight above the centre of the floor, image axes along
        # the bin axes, as in the cell
        'camera_mount': {'xyz': [0.0, 0.0, mount['xyz'][2] - wall], 'rpy': list(mount['rpy'])},
        'camera_rate_hz': ring.get('camera_rate_hz'),
        'arms': arms,
        'bins': bins,
    }


def mount_z(cell):
    """Height of the arm bases above the table top."""
    return cell['robot_mount']['xyz'][2]


def _wrap(angle):
    """Angle in (-pi, pi]."""
    angle = math.fmod(angle + math.pi, 2 * math.pi)
    if angle <= 0:
        angle += 2 * math.pi
    return angle - math.pi


def frame_pose(item, z):
    """World pose ((x, y, z), quaternion xyzw) of an arm base or a bin frame of a layout."""
    half = item['yaw'] / 2
    return (item['xy'][0], item['xy'][1], z), (0.0, 0.0, math.sin(half), math.cos(half))


def bin_poses(ring_layout):
    """{bin name: world pose of the bin frame} of a layout: the centre of each inner floor."""
    z = ring_layout['table']['height'] + ring_layout['bin_wall']
    return {item['name']: frame_pose(item, z) for item in ring_layout['bins']}


# -- files generated for one N ----------------------------------------------------------------

def generated_dir(n):
    """Directory of the files generated for a ring of `n` arms (layout, controllers, bridge,
    SRDF): rewritten by every launch, so nothing in it is a source."""
    path = Path(tempfile.gettempdir()) / 'openfactory_ring' / f'n{check_n(n)}'
    path.mkdir(parents=True, exist_ok=True)
    return path


def dump_yaml(data):
    """YAML text without anchors and aliases: the parameter-file parser of rcl rejects them,
    and generated dicts share sub-objects."""
    import yaml

    class NoAliasDumper(yaml.SafeDumper):

        def ignore_aliases(self, data):
            return True

    return yaml.dump(data, Dumper=NoAliasDumper, sort_keys=False)


def load_layout(n):
    """Layout of a ring of `n` arms from the installed cell.yaml and ring.yaml."""
    import yaml
    from ament_index_python.packages import get_package_share_directory

    config = Path(get_package_share_directory(PACKAGE)) / 'config'
    return layout(check_n(n), yaml.safe_load((config / 'cell.yaml').read_text()),
                  yaml.safe_load((config / 'ring.yaml').read_text()))


def write_layout(n, directory=None):
    """Write the layout of a ring of `n` arms as YAML for xacro; returns the file path."""
    path = Path(directory or generated_dir(n)) / 'layout.yaml'
    path.write_text(dump_yaml(load_layout(n)))
    return path
