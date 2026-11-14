"""Checks that the SRDF of the Rubik's cube cell matches the URDF it is loaded with."""

import xml.etree.ElementTree as ET
from pathlib import Path

import xacro

from ament_index_python.packages import get_package_share_directory


def _process(package, *parts, **mappings):
    path = Path(get_package_share_directory(package)).joinpath(*parts)
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


URDF = _process('openfactory_description', 'urdf', 'rubik_cell.urdf.xacro',
                include_fixtures='true')
SRDF = _process('openfactory_moveit_config', 'srdf', 'rubik_cell.srdf.xacro')
CELL_SRDF = _process('openfactory_moveit_config', 'srdf', 'openfactory_cell.srdf.xacro')
LINKS = {link.get('name') for link in URDF.findall('link')}
FIXTURES = ('table', 'floor', 'pedestal', 'nest', 'camera_link')
MOVING = ('upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link',
          'gripper_palm', 'left_finger', 'right_finger')


def _disabled(srdf):
    return {frozenset((p.get('link1'), p.get('link2'))) for p in srdf.findall('disable_collisions')}


def test_srdf_references_existing_links():
    assert SRDF.get('name') == URDF.get('name')
    for pair in SRDF.findall('disable_collisions'):
        assert pair.get('link1') in LINKS, pair.get('link1')
        assert pair.get('link2') in LINKS, pair.get('link2')


def test_fixtures_collide_with_the_moving_arm():
    disabled = _disabled(SRDF)
    for fixture in FIXTURES:
        for link in MOVING:
            assert frozenset((fixture, link)) not in disabled


def test_fixtures_do_not_collide_with_each_other():
    disabled = _disabled(SRDF)
    for i, a in enumerate(FIXTURES):
        for b in FIXTURES[i + 1:]:
            assert frozenset((a, b)) in disabled, (a, b)


def test_arm_and_gripper_matrix_is_the_cells():
    """Everything that does not name a bin or the nest is shared with the cell's SRDF."""
    def own(srdf, fixtures):
        return {pair for pair in _disabled(srdf) if not pair & set(fixtures)}

    assert own(SRDF, ('pedestal', 'nest', 'camera_link')) == own(
        CELL_SRDF, ('input_bin', 'output_bin', 'camera_link'))
    assert [g.get('name') for g in SRDF.findall('group')] == [
        g.get('name') for g in CELL_SRDF.findall('group')]
