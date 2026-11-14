"""The Rubik's cube cell's configuration files agree with each other: the grasp of
rubik.yaml on the gripper, the cube and the nest of openfactory_description."""

import math
from pathlib import Path

import pytest
import yaml

from openfactory_manipulation.core.rubik import orientation as ori
from openfactory_manipulation.core.rubik import plan
from openfactory_manipulation.core.rubik import rotations as rot
from openfactory_manipulation.core.rubik import station as st

SRC = Path(__file__).resolve().parents[2]
DESCRIPTION = SRC / 'openfactory_description' / 'config'
RUBIK = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config' / 'rubik.yaml').read_text())
CUBE = yaml.safe_load((DESCRIPTION / 'rubik.yaml').read_text())
LAYOUT = yaml.safe_load((DESCRIPTION / 'rubik_cell.yaml').read_text())
GRIPPER = yaml.safe_load((DESCRIPTION / 'rubik_gripper.yaml').read_text())
PITCH = CUBE['cube']['pitch']
STATION = st.Station(PITCH, GRIPPER, RUBIK['grasp']['tip_margin'])


def test_fingers_hold_the_centre_cubie_of_a_face_clear_of_the_nest():
    nest, pad = LAYOUT['nest'], GRIPPER['finger']['pad']['size']
    rim = nest['wall_height'] + nest['lead_in']['height']
    # tool down: the fingertips are above the bottom layer and above the rim of the nest
    assert STATION.tip_height() == pytest.approx(PITCH + RUBIK['grasp']['tip_margin'])
    assert STATION.tip_height() - rim >= 0.005
    # the ribs span the middle layer only, centred on the cube centre
    assert STATION.tip_height() + pad[2] <= 2 * PITCH - 0.001
    assert STATION.tip_height() + pad[2] / 2 == pytest.approx(STATION.seat[2])
    # tool horizontal, as a flip sets the cube down: the fingers pass over the rim
    release = STATION.lifted(RUBIK['flip']['release_height'])[2]
    assert release - GRIPPER['finger']['size'][0] / 2 - rim >= 0.005


def test_fingers_open_round_the_cube_and_close_on_it():
    grasp, finger = RUBIK['grasp'], GRIPPER['finger']
    held = 1.5 * PITCH + finger['pad']['size'][1]        # finger joint with the ribs on the cube
    assert held + grasp['preclose_margin'] <= finger['stroke']
    assert 0.0 < grasp['squeeze_depth'] <= 0.001
    assert grasp['width_tolerance'] < 2 * grasp['preclose_margin']
    # the approach starts with the fingertips above the cube
    assert STATION.tip_height() + grasp['approach_height'] >= 3 * PITCH + 0.01


def test_a_lifted_cube_clears_the_nest_when_it_is_tipped():
    nest = LAYOUT['nest']
    rim = nest['wall_height'] + nest['lead_in']['height']
    corner = 1.5 * PITCH * math.sqrt(2)
    assert STATION.lifted(RUBIK['flip']['lift_height'])[2] - corner - rim >= 0.01
    # set down a little above the seat, within what the lead-in takes
    assert 0.0 < RUBIK['flip']['release_height'] <= nest['lead_in']['height']


def test_flip_kinds_are_quarter_turns_that_reach_every_face():
    kinds = plan.kind_rotations(RUBIK['flip']['kinds'])
    assert set(kinds) == {'over', 'side'}
    for name, spec in RUBIK['flip']['kinds'].items():
        finger, tool = spec['finger_axis'], spec['tool_axis']
        assert sum(a * b for a, b in zip(finger, tool)) == 0 and tool[2] == 0 and finger[2] == 0
        assert rot.mat_vec(kinds[name], ori.DOWN) == tuple(tool)
    # 'over': the face that looked away from the robot (+x) comes down
    assert rot.mat_vec(kinds['over'], (1, 0, 0)) == ori.DOWN
    assert rot.mat_vec(kinds['side'], (0, -1, 0)) == ori.DOWN
    options = plan.flip_options(kinds, RUBIK['plan'])
    assert len(plan._shortest(options)) == 24
    assert len(plan.scan_plan(options)) <= 6


def test_view_limits_fit_the_nest_and_the_evaluation():
    view, nest = RUBIK['view'], LAYOUT['nest']
    play = 2 * nest['clearance'] / (3 * PITCH)                 # rad the nest leaves the cube
    assert view['seat_yaw'] > play + 0.02
    assert view['seat_offset'] > nest['clearance'] * math.sqrt(2)
    assert 0.0 < view['min_score'] < 1.0
    assert RUBIK['task']['rescans'] >= 2
