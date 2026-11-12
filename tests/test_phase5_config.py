"""Consistency of scripts/config/phase5_tumbled.yaml (E-005)."""

from pathlib import Path

import yaml

import randomize_scene as rs
from openfactory_manipulation.core import ranking

REPO = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((REPO / 'scripts' / 'config' / 'phase5_tumbled.yaml').read_text())
PHASE4 = yaml.safe_load((REPO / 'scripts' / 'config' / 'phase4_bin_picking.yaml').read_text())
DESCRIPTION = REPO / 'src' / 'openfactory_description' / 'config'
CATALOG = yaml.safe_load((DESCRIPTION / 'objects.yaml').read_text())
CELL = yaml.safe_load((DESCRIPTION / 'cell.yaml').read_text())


def test_strategies_are_known_and_the_default_exists():
    assert CONFIG['default_strategy'] in CONFIG['strategies']
    for name, strategy in CONFIG['strategies'].items():
        assert strategy['ranking'] in ranking.STRATEGIES, name
        assert isinstance(strategy['unknown_grasps'], bool), name
        if strategy['ranking'] == 'learned':
            assert strategy['model']


def test_seeds_are_disjoint_from_each_other_and_from_phase_4():
    seeds = [s['seed'] for s in CONFIG['sets'].values()]
    assert len(seeds) == len(set(seeds))
    assert not set(seeds) & {s['seed'] for s in PHASE4['sets'].values()}
    assert CONFIG['sets']['train']['empty_scenes'] == 0


def test_every_scene_can_be_sampled_and_holds_tumbled_objects():
    tumbled = 0
    for chosen in CONFIG['sets'].values():
        for index in range(chosen['scenes']):
            scene = rs.sample_scene(chosen['seed'], index, CONFIG['objects_per_scene'], CATALOG,
                                    CELL, CONFIG['scene'])
            assert len(scene) == CONFIG['objects_per_scene']
            tumbled += sum(item['rest'] == 'tumbled' for item in scene)
    assert tumbled >= 100
