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
