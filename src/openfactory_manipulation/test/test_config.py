"""Consistency of config/pick_place.yaml with the cell layout, the gripper and the catalog."""

from pathlib import Path

import yaml

from openfactory_manipulation.core.objects import bounding_box

PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'openfactory_description' / 'config'
CONFIG = yaml.safe_load((PACKAGE / 'config' / 'pick_place.yaml').read_text())
CELL = yaml.safe_load((DESCRIPTION / 'cell.yaml').read_text())
GRIPPER = yaml.safe_load((DESCRIPTION / 'gripper.yaml').read_text())
CATALOG = yaml.safe_load((DESCRIPTION / 'objects.yaml').read_text())


def test_the_fingers_fit_between_every_place_point_and_the_walls():
    """Whatever the finger direction, the fingers opened to release the widest object keep
    their clearance from the walls of the output bin (a finger pressed against a wall is a
    start state in collision for MoveIt, and nothing can be planned from it)."""
    inner = CELL['bins']['output']['inner_size']
    widest = max(max(bounding_box(spec)[:2]) for spec in CATALOG['objects'].values())
    reach = widest / 2 + CONFIG['grasp']['preclose_margin'] + GRIPPER['finger']['size'][1]
    for x, y in CONFIG['place']['slots']:
        to_wall = min(inner[0] / 2 - abs(x), inner[1] / 2 - abs(y))
        assert to_wall - reach >= CONFIG['selection']['finger_clearance'], (x, y)


def test_place_points_are_enough_and_apart():
    slots = CONFIG['place']['slots']
    assert len(slots) == len({tuple(slot) for slot in slots}) >= 5
    widest = max(max(bounding_box(spec)[:2]) for spec in CATALOG['objects'].values())
    # two placed objects, each grown by the padding on its side, do not overlap
    apart = widest + 2 * CONFIG['place']['obstacle_padding']
    for i, (ax, ay) in enumerate(slots):
        for bx, by in slots[i + 1:]:
            assert max(abs(ax - bx), abs(ay - by)) >= apart, ((ax, ay), (bx, by))


def test_ready_is_a_full_arm_configuration():
    assert len(CONFIG['ready_joints']) == 6
