import math
from pathlib import Path

import pytest
import yaml

import pick_place_eval as ev
from openfactory_manipulation.core.geometry import quat_from_rpy

CONFIG = Path(__file__).resolve().parents[1] / 'src' / 'openfactory_description' / 'config'
CELL = yaml.safe_load((CONFIG / 'cell.yaml').read_text())
CUBE = {'shape': 'box', 'size': [0.05, 0.05, 0.05]}
TOL = {'max_tilt_rad': 0.1, 'rest_height_m': 0.005}
UPRIGHT = (0.0, 0.0, 0.0, 1.0)


def test_tilt():
    assert ev.tilt(UPRIGHT) == pytest.approx(0.0)
    assert ev.tilt(quat_from_rpy(0.0, math.pi / 2, 0.3)) == pytest.approx(math.pi / 2)
    assert ev.tilt(quat_from_rpy(0.0, 0.0, 2.0)) == pytest.approx(0.0, abs=1e-9)


def test_object_resting_in_the_output_bin():
    result = ev.evaluate_place(((0.4094, 0.30, 0.785), UPRIGHT), CELL, CUBE, (0.0, 0.0), TOL)
    assert result['in_output_bin'] and not result['in_input_bin']
    assert result['place_error_m'] == pytest.approx(0.0094)


@pytest.mark.parametrize('pose, reason', [
    (((0.40, -0.30, 0.785), UPRIGHT), 'still in the input bin'),
    (((0.40, 0.30, 0.80), UPRIGHT), 'not resting on the floor'),
    (((0.40, 0.30, 0.785), quat_from_rpy(0.5, 0.0, 0.0)), 'tipped over'),
    (((0.40, 0.0, 0.775), UPRIGHT), 'on the table between the bins'),
])
def test_object_not_placed(pose, reason):
    assert not ev.evaluate_place(pose, CELL, CUBE, (0.0, 0.0), TOL)['in_output_bin'], reason


def test_missing_object():
    result = ev.evaluate_place(None, CELL, CUBE, (0.0, 0.0), TOL)
    assert not result['object_found'] and not result['in_output_bin']


def test_outcome_needs_every_source():
    ok = {'success': True, 'failure_reason': None}
    placed = {'in_output_bin': True}
    assert ev.outcome(ok, placed, 0) == (True, None)
    assert ev.outcome({'success': False, 'failure_reason': 'grasp_failed'}, placed, 0) == (
        False, 'grasp_failed')
    assert ev.outcome(ok, {'in_output_bin': False}, 0) == (False, 'place_failed')
    assert ev.outcome(ok, placed, 3) == (False, 'collision')
