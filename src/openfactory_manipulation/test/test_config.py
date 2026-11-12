"""Consistency of config/pick_place.yaml with the cell layout, the gripper and the catalog."""

import math
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
    """Worst case between two place points: the fingers, open to release the widest object,
    point at the neighbour, and the neighbour, grown by its padding, points a corner back.
    They must not meet, or no finger direction is sure to fit and placing gets blocked."""
    slots = CONFIG['place']['slots']
    assert len(slots) == len({tuple(slot) for slot in slots}) >= 5
    padding = CONFIG['place']['obstacle_padding']
    finger = GRIPPER['finger']['size']
    reach = corner = 0.0
    for spec in CATALOG['objects'].values():
        x, y, _ = bounding_box(spec)
        along = max(x, y) / 2 + CONFIG['grasp']['preclose_margin'] + finger[1]
        reach = max(reach, math.hypot(along, finger[0] / 2))
        corner = max(corner, math.hypot(x / 2 + padding, y / 2 + padding))
    for i, (ax, ay) in enumerate(slots):
        for bx, by in slots[i + 1:]:
            assert math.hypot(ax - bx, ay - by) >= reach + corner, ((ax, ay), (bx, by))


def test_ready_is_a_full_arm_configuration():
    assert len(CONFIG['ready_joints']) == 6


def test_a_missed_unknown_grasp_is_still_detected():
    """A miss closes the jaws to the close target, 2 x squeeze_depth under the measured width:
    the wider tolerance for measured widths must stay below that."""
    selection, grasp = CONFIG['selection'], CONFIG['grasp']
    assert grasp['width_tolerance'] <= selection['unknown_width_tolerance'] \
        < 2 * grasp['squeeze_depth']


def test_the_default_ranking_has_its_model():
    from openfactory_manipulation.core import ranking
    weights = CONFIG['ranking']
    assert weights['strategy'] in ranking.STRATEGIES
    if weights['strategy'] == 'learned':
        ranking.Ranker(weights, ranking.load_model(PACKAGE / 'config' / weights['model']))
