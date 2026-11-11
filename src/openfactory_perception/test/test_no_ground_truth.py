"""Rule 5: perception never consumes simulator ground truth.

Its inputs are camera data (depth, colour, camera_info) and TF. Ground truth lives in Gazebo
(`/world/<world>/dynamic_pose/info`, read by the harness only) and, if ever bridged, under
`/ground_truth/` in ROS. No file of this package may refer to either, nor talk to Gazebo.
"""

from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
FORBIDDEN = ('ground_truth', 'dynamic_pose', 'pose/info', 'gz.transport', 'gz.msgs', 'ros_gz',
             'gz_scene', 'image_ideal')


def test_no_file_refers_to_ground_truth():
    offenders = []
    for path in PACKAGE.rglob('*'):
        if not path.is_file() or path.suffix not in ('.py', '.yaml', '.xml', '.cfg'):
            continue
        if path.resolve() == Path(__file__).resolve():
            continue
        text = path.read_text()
        offenders += [f'{path.relative_to(PACKAGE)}: {word}' for word in FORBIDDEN if word in text]
    assert not offenders, offenders
