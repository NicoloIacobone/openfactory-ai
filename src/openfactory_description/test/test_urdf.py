"""Checks the generated cell URDFs against config/cell.yaml."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

PKG = Path(get_package_share_directory('openfactory_description'))
CELL = yaml.safe_load((PKG / 'config' / 'cell.yaml').read_text())

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def _urdf(name, **mappings):
    doc = xacro.process_file(str(PKG / 'urdf' / name), mappings=mappings)
    return ET.fromstring(doc.toxml())


def _joint(root, name):
    joint = root.find(f"joint[@name='{name}']")
    assert joint is not None, f'missing joint {name}'
    return joint


def _xyz(element):
    return [float(v) for v in element.find('origin').get('xyz').split()]


@pytest.mark.parametrize('include_table', ['true', 'false'])
def test_cell_robot(include_table):
    root = _urdf('openfactory_cell.urdf.xacro', include_table=include_table)
    links = {link.get('name') for link in root.findall('link')}

    assert ('table' in links) == (include_table == 'true')
    for name in ARM_JOINTS:
        assert _joint(root, name).get('type') == 'revolute'

    # ros2_control interfaces exist for every arm joint (gz_ros2_control needs them)
    control_joints = {j.get('name') for j in root.findall('ros2_control/joint')}
    assert set(ARM_JOINTS) <= control_joints

    base = _joint(root, 'base_link-base_link_inertia').find('parent').get('link')
    assert base == 'base_link'
    mount = root.find("joint/child[@link='base_link']/..")
    assert mount.find('parent').get('link') == 'world'

    table = CELL['table']
    expected = [
        table['xyz'][0] + CELL['robot_mount']['xyz'][0],
        table['xyz'][1] + CELL['robot_mount']['xyz'][1],
        table['xyz'][2] + table['size'][2] + CELL['robot_mount']['xyz'][2],
    ]
    assert _xyz(mount) == pytest.approx(expected)


def test_table_model_matches_cell_table():
    standalone = _urdf('table.urdf.xacro')
    in_cell = _urdf('openfactory_cell.urdf.xacro', include_table='true')

    for root in (standalone, in_cell):
        joint = _joint(root, 'table_joint')
        assert joint.find('parent').get('link') == 'world'
        table = CELL['table']
        assert _xyz(joint) == pytest.approx(
            [table['xyz'][0], table['xyz'][1], table['xyz'][2] + table['size'][2]])
        size = root.find("link[@name='table']/collision/geometry/box").get('size')
        assert [float(v) for v in size.split()] == pytest.approx(table['size'])
