import math
from pathlib import Path

import pytest
import yaml

import bin_picking_eval as be
import randomize_scene as rs
from openfactory_manipulation.core.geometry import quat_from_rpy

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / 'src' / 'openfactory_description' / 'config'
CELL = yaml.safe_load((CONFIG / 'cell.yaml').read_text())
CATALOG = yaml.safe_load((CONFIG / 'objects.yaml').read_text())
PHASE4 = yaml.safe_load((REPO / 'scripts' / 'config' / 'phase4_bin_picking.yaml').read_text())
TOL = PHASE4['evaluation']
UPRIGHT = (0.0, 0.0, 0.0, 1.0)
# bin frames in the world: input (0.40, -0.30, 0.76), output (0.40, 0.30, 0.76)
OBJECTS = [{'name': 'cube_0', 'type': 'cube'}, {'name': 'prism_1', 'type': 'prism'},
           {'name': 'cylinder_2', 'type': 'cylinder'}]
PRISM_Z = 0.05 * math.sqrt(3) / 4


def poses(cube=(0.40, -0.30), prism=(0.48, -0.25), cylinder=(0.33, -0.36), prism_quat=UPRIGHT,
          prism_z=0.76 + PRISM_Z):
    return {'cube_0': ((*cube, 0.785), UPRIGHT),
            'prism_1': ((*prism, prism_z), prism_quat),
            'cylinder_2': ((*cylinder, 0.80), UPRIGHT)}


def states(now, earlier=None):
    return be.object_states(OBJECTS, earlier or now, now, CELL, CATALOG, TOL)


@pytest.mark.parametrize('xyz, expected', [
    ((0.40, -0.30, 0.785), be.INPUT), ((0.40, 0.30, 0.785), be.OUTPUT),
    ((0.40, 0.0, 0.775), be.ELSEWHERE),            # on the table between the bins
    ((0.40, 0.30, 0.90), be.ELSEWHERE),            # above the rim
    ((0.56, 0.30, 0.785), be.ELSEWHERE),           # beyond the wall
    (None, be.MISSING)])
def test_location(xyz, expected):
    assert be.location(xyz, CELL) == expected


def test_object_states_tell_bins_rest_and_orientation():
    found = states(poses(), poses())
    assert all(s['location'] == be.INPUT and s['at_rest'] and s['upright']
               for s in found.values())
    # prism rolled onto another rectangular face in the output bin: deposited, not upright
    rolled = poses(prism=(0.40, 0.30), prism_quat=quat_from_rpy(2 * math.pi / 3, 0.0, 0.4),
                   prism_z=0.76 + 0.011)
    prism = states(rolled, rolled)['prism_1']
    assert prism['location'] == be.OUTPUT and be.deposited(prism) and not prism['upright']
    # still moving between the two readings
    moving = states(poses(cube=(0.40, 0.30)), poses(cube=(0.40, 0.295)))['cube_0']
    assert moving['location'] == be.OUTPUT and not be.deposited(moving)
    gone = be.object_states(OBJECTS, {}, {}, CELL, CATALOG, TOL)['cube_0']
    assert gone['location'] == be.MISSING and not be.deposited(gone)


def test_match_target_takes_the_nearest_object_in_the_input_bin():
    before = states(poses())
    assert be.match_target(0.001, -0.001, before, CELL, 0.02) == 'cube_0'
    assert be.match_target(0.08, 0.05, before, CELL, 0.02) == 'prism_1'
    assert be.match_target(0.0, 0.08, before, CELL, 0.02) is None
    # an object already in the output bin is never the target
    after = states(poses(cube=(0.40, 0.30)))
    assert be.match_target(0.0, 0.0, after, CELL, 0.02) is None


def test_evaluate_attempt():
    before = states(poses())
    done = be.evaluate_attempt('cube_0', before, states(poses(cube=(0.41, 0.31))), TOL)
    assert done['target_deposited'] and done['target_type'] == 'cube'
    assert done['newly_deposited'] == ['cube_0'] and done['disturbed'] == [] == done['lost']

    # the cube stayed, the fingers pushed the prism and the cylinder ended on the table
    pushed = poses(prism=(0.49, -0.25), cylinder=(0.40, 0.0))
    missed = be.evaluate_attempt('cube_0', before, states(pushed), TOL)
    assert not missed['target_deposited'] and missed['target_location'] == be.INPUT
    assert missed['disturbed'] == ['cylinder_2', 'prism_1'] and missed['lost'] == ['cylinder_2']

    nothing = be.evaluate_attempt(None, before, before, TOL)
    assert not nothing['target_deposited'] and nothing['target'] is None


@pytest.mark.parametrize('cell_ok, reason, arrived, invalid, expected', [
    (True, None, True, 0, (True, None, 'agree')),
    (True, None, False, 0, (False, 'place_failed', 'undetected_failure')),
    (True, None, True, 2, (False, 'collision', 'agree')),
    (False, 'grasp_failed', False, 0, (False, 'grasp_failed', 'agree')),
    (False, 'object_not_removed', True, 0, (False, 'object_not_removed', 'false_alarm')),
])
def test_attempt_result_needs_every_source(cell_ok, reason, arrived, invalid, expected):
    evaluation = {'target_deposited': arrived}
    assert be.attempt_result(cell_ok, reason, evaluation, invalid) == expected


def test_scene_result():
    final = states(poses(cube=(0.41, 0.31), prism=(0.40, 0.25)))
    result = be.scene_result(OBJECTS, final, 'no_feasible_grasp', 3)
    assert (result['n_objects'], result['deposited']) == (3, 2)
    assert result['completion'] == pytest.approx(2 / 3)
    assert result['left_in_input'] == ['cylinder_2'] and result['end_correct']
    assert result['by_type']['cylinder'] == {'objects': 1, 'deposited': 0}
    # the cell said empty while an object is still in the input bin
    assert not be.scene_result(OBJECTS, final, 'bin_empty', 3)['end_correct']
    empty = be.scene_result([], {}, 'bin_empty', 0)
    assert empty['completion'] is None and empty['end_correct'] and empty['n_objects'] == 0


def _attempt(success, reason=None, cell_ok=None, detection='agree', invalid=0, recovered=None,
             view=1.0, category='cube', rejected=None):
    cell_ok = success if cell_ok is None else cell_ok
    return {
        'object_category': category, 'success': success, 'failure_reason': reason,
        'detection': detection, 'candidates_total': 8, 'candidates_rejected': rejected or {},
        'execution_result': {'cell_success': cell_ok,
                             'cell_failure': None if cell_ok else reason,
                             'failed_stage': None, 'recovered': recovered},
        'verification': {'view_stamp': view},
        'evaluation': {'states_checked': 100, 'states_invalid': invalid, 'disturbed': []},
        'perception_s': 0.6, 'planning_s': 0.3, 'execution_s': 15.0, 'total_s': 16.0,
    }


def _scene(n, done, end='bin_empty', attempts=None):
    return {'n_objects': n, 'deposited': done, 'completion': done / n if n else None,
            'upright': done, 'lost': [], 'end_reason': end, 'end_correct': True,
            'attempts': n if attempts is None else attempts,
            'by_type': {'cube': {'objects': n, 'deposited': done}} if n else {}}


def test_summarize_metrics_and_thresholds():
    attempts = ([_attempt(True) for _ in range(9)]
                + [_attempt(False, 'grasp_failed', recovered={'ok': True}),
                   _attempt(False, 'planning_failed', recovered={'ok': False}),
                   _attempt(False, 'place_failed', cell_ok=True,
                            detection='undetected_failure')])
    attempts[0]['candidates_rejected'] = {'wall_collision': 3, 'ik_failed': 1}
    scenes = [_scene(5, 5), _scene(5, 4, 'no_feasible_grasp', 7), _scene(0, 0, attempts=0)]
    summary = be.summarize(attempts, scenes, PHASE4['thresholds'])

    assert summary['pick_success']['rate'] == pytest.approx(9 / 12)
    assert summary['completion']['rate'] == pytest.approx(0.9)
    assert summary['completion_by_type']['cube']['successes'] == 9
    assert (summary['scenes'], summary['scenes_cleared'], summary['empty_scenes']) == (2, 1, 1)
    assert summary['end_reasons'] == {'bin_empty': 2, 'no_feasible_grasp': 1}
    assert summary['undetected_failures'] == 1 and summary['false_alarms'] == 0
    # two failures the cell detected, one recovered
    assert (summary['recovery']['successes'], summary['recovery']['n']) == (1, 2)
    # one MoveIt rejection and one planning failure over 12 x 8 candidates
    assert (summary['planning_failure']['successes'], summary['planning_failure']['n']) == (2, 96)
    assert summary['states_invalid'] == 0 and summary['collision']['rate'] == 0.0
    assert summary['checks'] == {
        'completion': True, 'completion_by_type': True, 'pick_success': False,
        'collisions': True, 'undetected_failures': True, 'recovery': False,
        'empty_scenes': True}
    assert not summary['passed']


def test_a_clean_run_passes_and_an_attempt_on_an_empty_bin_fails():
    attempts = [_attempt(True) for _ in range(10)]
    clean = be.summarize(attempts, [_scene(5, 5), _scene(5, 5), _scene(0, 0, attempts=0)],
                         PHASE4['thresholds'])
    assert clean['passed'] and clean['recovery']['n'] == 0
    phantom = be.summarize(attempts, [_scene(5, 5), _scene(5, 5), _scene(0, 0, attempts=1)],
                           PHASE4['thresholds'])
    assert not phantom['checks']['empty_scenes'] and not phantom['passed']
    collided = be.summarize([_attempt(False, 'collision', cell_ok=True, invalid=3)] + attempts,
                            [_scene(5, 5), _scene(5, 5)], PHASE4['thresholds'])
    assert not collided['checks']['collisions']


def test_every_scene_of_the_phase_4_sets_can_be_sampled():
    """The benchmark must not stop on a scene the sampler cannot fill."""
    for chosen in PHASE4['sets'].values():
        for index in range(chosen['scenes']):
            scene = rs.sample_scene(chosen['seed'], index, PHASE4['objects_per_scene'], CATALOG,
                                    CELL, PHASE4['scene'])
            assert len(scene) == PHASE4['objects_per_scene']
            assert len({item['name'] for item in scene}) == len(scene)
    types = [item['type'] for index in range(PHASE4['sets']['eval']['scenes'])
             for item in rs.sample_scene(PHASE4['sets']['eval']['seed'], index,
                                         PHASE4['objects_per_scene'], CATALOG, CELL,
                                         PHASE4['scene'])]
    # enough objects of each type for a per-type rate
    assert all(types.count(name) >= 20 for name in CATALOG['objects'])
