import math
from pathlib import Path

import pytest
import yaml

import randomize_scene as rs

CONFIG = Path(__file__).resolve().parents[1] / 'src' / 'openfactory_description' / 'config'
CATALOG = yaml.safe_load((CONFIG / 'objects.yaml').read_text())
CELL = yaml.safe_load((CONFIG / 'cell.yaml').read_text())
PARAMS = {'wall_margin': 0.012, 'min_gap': 0.02}
INNER = CELL['bins']['input']['inner_size']


def test_same_seed_and_index_give_the_same_scene():
    a = rs.sample_scene(1, 4, 3, CATALOG, CELL, PARAMS)
    assert a == rs.sample_scene(1, 4, 3, CATALOG, CELL, PARAMS)
    assert a != rs.sample_scene(1, 5, 3, CATALOG, CELL, PARAMS)
    assert a != rs.sample_scene(2, 4, 3, CATALOG, CELL, PARAMS)


def test_footprint_radius():
    objects = CATALOG['objects']
    assert rs.footprint_radius(objects['cube']) == pytest.approx(0.05 * math.sqrt(2) / 2)
    assert rs.footprint_radius(objects['cylinder']) == pytest.approx(0.025)
    assert rs.footprint_radius(objects['prism']) == pytest.approx(math.hypot(0.06, 0.05) / 2)


@pytest.mark.parametrize('n_objects', [0, 1, 2, 3])
def test_objects_are_inside_the_bin_and_apart(n_objects):
    for index in range(50):
        scene = rs.sample_scene(7, index, n_objects, CATALOG, CELL, PARAMS)
        assert len(scene) == n_objects
        assert len({item['name'] for item in scene}) == n_objects
        for item in scene:
            radius = rs.footprint_radius(CATALOG['objects'][item['type']])
            assert abs(item['x']) + radius <= INNER[0] / 2 - PARAMS['wall_margin'] + 1e-12
            assert abs(item['y']) + radius <= INNER[1] / 2 - PARAMS['wall_margin'] + 1e-12
            assert -math.pi <= item['yaw'] <= math.pi
        for i, a in enumerate(scene):
            for b in scene[i + 1:]:
                gap = (math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                       - rs.footprint_radius(CATALOG['objects'][a['type']])
                       - rs.footprint_radius(CATALOG['objects'][b['type']]))
                assert gap >= PARAMS['min_gap'] - 1e-12


def test_every_type_and_the_whole_bin_are_used():
    scenes = [rs.sample_scene(3, i, 1, CATALOG, CELL, PARAMS)[0] for i in range(300)]
    assert {s['type'] for s in scenes} == set(CATALOG['objects'])
    assert max(s['x'] for s in scenes) > 0.08 and min(s['x'] for s in scenes) < -0.08
    assert max(s['y'] for s in scenes) > 0.04 and min(s['y'] for s in scenes) < -0.04


def test_fixed_types_and_a_centre_region():
    params = {**PARAMS, 'region': [0.075, 0.035]}
    for index in range(100):
        scene = rs.sample_scene(1, index, 1, CATALOG, CELL, params, types=['prism'])
        assert scene[0]['type'] == 'prism' and scene[0]['name'] == 'prism_0'
        assert abs(scene[0]['x']) <= 0.075 and abs(scene[0]['y']) <= 0.035
    with pytest.raises(ValueError):
        rs.sample_scene(1, 0, 2, CATALOG, CELL, params, types=['cube'])


def test_an_impossible_scene_is_refused():
    with pytest.raises(ValueError, match='no room'):
        rs.sample_scene(1, 0, 12, CATALOG, CELL, PARAMS)
    with pytest.raises(ValueError, match='does not fit'):
        rs.sample_scene(1, 0, 1, CATALOG, CELL, {'wall_margin': 0.2, 'min_gap': 0.0})


PHASE4_PARAMS = {'wall_margin': 0.008, 'min_gap': 0.010}


def test_phase_4_scenes_are_unchanged_by_the_rest_states():
    """Scene 3 of the E-004 evaluation seed, recorded before tumbled objects existed."""
    scene = rs.sample_scene(1, 3, 5, CATALOG, CELL, PHASE4_PARAMS)
    expected = [('cube', -0.063892, -0.052446, -2.215069), ('cube', 0.035246, 0.020345, 0.357302),
                ('cylinder', 0.09489, -0.074249, 0.065397),
                ('cube', -0.094975, 0.053837, 0.582784), ('cube', 0.022877, -0.064084, -1.53789)]
    assert [(i['type'], i['x'], i['y'], i['yaw']) for i in scene] == [
        (t, pytest.approx(x, abs=1e-6), pytest.approx(y, abs=1e-6), pytest.approx(w, abs=1e-6))
        for t, x, y, w in expected]
    assert {i['rest'] for i in scene} == {'catalog'}


def test_tumbled_share():
    params = {**PHASE4_PARAMS, 'tumbled_share': 0.5}
    scenes = [rs.sample_scene(5000, i, 5, CATALOG, CELL, params) for i in range(60)]
    items = [item for scene in scenes for item in scene]
    assert all(item['rest'] == 'catalog' for item in items if item['type'] == 'cube')
    for name in ('cylinder', 'prism'):
        rests = [item['rest'] for item in items if item['type'] == name]
        assert 0.3 < rests.count('tumbled') / len(rests) < 0.7
    for scene in scenes:
        for i, a in enumerate(scene):
            ra = rs.footprint_radius(CATALOG['objects'][a['type']], a['rest'])
            assert abs(a['x']) + ra <= INNER[0] / 2 - params['wall_margin'] + 1e-12
            assert abs(a['y']) + ra <= INNER[1] / 2 - params['wall_margin'] + 1e-12
            for b in scene[i + 1:]:
                rb = rs.footprint_radius(CATALOG['objects'][b['type']], b['rest'])
                assert math.hypot(a['x'] - b['x'], a['y'] - b['y']) - ra - rb >= \
                    params['min_gap'] - 1e-12
    everything = {**params, 'tumbled_share': 1.0}
    scene = rs.sample_scene(5000, 0, 5, CATALOG, CELL, everything, types=['cylinder', 'prism',
                                                                            'cube', 'prism', 'cube'])
    assert [i['rest'] for i in scene] == ['tumbled', 'tumbled', 'catalog', 'tumbled', 'catalog']


def test_tumbled_footprint_radius():
    objects = CATALOG['objects']
    assert rs.footprint_radius(objects['cylinder'], 'tumbled') == pytest.approx(
        math.hypot(0.05, 0.08) / 2)
    assert rs.footprint_radius(objects['prism'], 'tumbled') == pytest.approx(
        math.hypot(0.05, 0.05 * math.sqrt(3) / 2) / 2)
