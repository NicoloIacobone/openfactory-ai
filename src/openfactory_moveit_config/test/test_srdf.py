"""Checks that the SRDF matches the cell URDF it is loaded with."""

import xml.etree.ElementTree as ET
from pathlib import Path

import xacro

from ament_index_python.packages import get_package_share_directory


def _process(package, *parts, **mappings):
    path = Path(get_package_share_directory(package)).joinpath(*parts)
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


URDF = _process('openfactory_description', 'urdf', 'openfactory_cell.urdf.xacro',
                include_table='true')
SRDF = _process('openfactory_moveit_config', 'srdf', 'openfactory_cell.srdf.xacro')
LINKS = {link.get('name') for link in URDF.findall('link')}
JOINTS = {joint.get('name') for joint in URDF.findall('joint')}


def test_robot_names_match():
    assert SRDF.get('name') == URDF.get('name')


def test_srdf_references_existing_links_and_joints():
    for pair in SRDF.findall('disable_collisions'):
        assert pair.get('link1') in LINKS
        assert pair.get('link2') in LINKS
    for chain in SRDF.findall('group/chain'):
        assert chain.get('base_link') in LINKS
        assert chain.get('tip_link') in LINKS
    for joint in SRDF.findall('group_state/joint'):
        assert joint.get('name') in JOINTS


def test_table_collides_with_moving_arm_links():
    disabled = {
        frozenset((p.get('link1'), p.get('link2'))) for p in SRDF.findall('disable_collisions')}
    for link in ['upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']:
        assert frozenset(('table', link)) not in disabled
