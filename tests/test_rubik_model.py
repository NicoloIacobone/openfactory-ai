import random
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

import rubik_model as rm
from openfactory_manipulation.core.rubik import cube as c
from openfactory_manipulation.core.rubik import cubies as cb
from openfactory_manipulation.core.rubik import rotations as rot

CONFIG = yaml.safe_load((Path(__file__).resolve().parents[1] / 'src' / 'openfactory_description'
                         / 'config' / 'rubik.yaml').read_text())
GRIPPER = yaml.safe_load((Path(__file__).resolve().parents[1] / 'src' / 'openfactory_description'
                          / 'config' / 'gripper.yaml').read_text())
PLUGIN_SOURCE = (Path(__file__).resolve().parents[1] / 'src' / 'openfactory_simulation' / 'src'
                 / 'rubik_cube_system.cc').read_text()


def model(state=c.SOLVED, **kwargs):
    return ET.fromstring(rm.cube_sdf(CONFIG, state, **kwargs)).find('model')


def test_cube_is_a_real_size_cube_that_fits_the_gripper():
    assert rm.side(CONFIG) == pytest.approx(0.057)
    assert 0.07 < rm.mass(CONFIG) < 0.12
    assert rm.side(CONFIG) + 0.02 < 2 * GRIPPER['finger']['stroke']


def test_links_and_joints():
    m = model()
    names = [link.get('name') for link in m.findall('link')]
    assert names[0] == 'core'
    assert sorted(n for n in names if n.startswith('axle_')) == sorted(
        'axle_' + f for f in c.FACES)
    assert sorted(n for n in names if n.startswith('cubie_')) == sorted(
        cb.name_of(home) for home in cb.HOMES)
    joints = {j.get('name'): j for j in m.findall('joint')}
    assert set(joints) == {f'axle_{f}_joint' for f in c.FACES}
    for face in c.FACES:
        joint = joints[f'axle_{face}_joint']
        assert joint.find('parent').text == 'core'
        axis = tuple(float(v) for v in joint.find('axis/xyz').text.split())
        assert axis == c.NORMAL_OF[face]
        mech = CONFIG['mechanism']
        assert float(joint.find('axis/dynamics/friction').text) == pytest.approx(
            mech['layer_friction'])
    total = sum(float(link.find('inertial/mass').text) for link in m.findall('link'))
    assert total == pytest.approx(rm.mass(CONFIG))
    assert m.find('self_collide').text == 'false'


def test_fixed_core_is_only_for_the_bench():
    assert 'bench_fix' not in rm.cube_sdf(CONFIG)
    joint = [j for j in model(fixed_core=True).findall('joint') if j.get('name') == 'bench_fix']
    assert joint[0].find('parent').text == 'world' and joint[0].find('child').text == 'core'


def test_stickers_of_a_solved_cube_face_outwards_with_their_colour():
    pitch = CONFIG['cube']['pitch']
    for link in model().findall('link'):
        name = link.get('name')
        if not name.startswith('cubie_'):
            continue
        home = cb.home_of(name)
        assert [float(v) for v in link.find('pose').text.split()] == pytest.approx(
            [pitch * x for x in home] + [0, 0, 0])
        stickers = [v for v in link.findall('visual') if v.get('name').startswith('sticker_')]
        assert len(stickers) == len(cb.normals_of(home))
        for sticker in stickers:
            face = sticker.get('name')[-1]
            offset = [float(v) for v in sticker.find('pose').text.split()][:3]
            reach = abs(max(offset, key=abs))
            assert reach > pitch / 2
            assert [round(v / reach) for v in offset] == list(c.NORMAL_OF[face])
            rgba = [float(v) for v in sticker.find('material/diffuse').text.split()]
            assert rgba == pytest.approx(CONFIG['cube']['colors'][face])


@pytest.mark.parametrize('seed', range(5))
def test_a_scrambled_model_reads_back_as_its_state(seed):
    state = c.apply_moves(c.SOLVED, c.random_scramble(random.Random(seed)))
    pitch = CONFIG['cube']['pitch']
    cubies = {}
    for link in model(state).findall('link'):
        if link.get('name').startswith('cubie_'):
            x, y, z, roll, pitch_angle, yaw = (float(v) for v in link.find('pose').text.split())
            rotation, error = rot.nearest_rotation(rot.from_rpy(roll, pitch_angle, yaw))
            assert error < 1e-6
            cubies[cb.home_of(link.get('name'))] = (
                tuple(round(v / pitch) for v in (x, y, z)), rotation)
    # centres carry no orientation information: any spin about their axis reads the same
    assert cb.state_of(cubies) == state


def test_plugin_parameters_match_the_plugin_source():
    plugin = model().find('plugin')
    assert plugin.get('filename') == rm.PLUGIN_FILE and plugin.get('name') == rm.PLUGIN_NAME
    assert f'openfactory::{rm.PLUGIN_NAME.split("::")[1]}' in PLUGIN_SOURCE
    for key in rm.PLUGIN_PARAMS + ('pitch', 'layer_inertia'):
        assert plugin.find(key) is not None
        assert f'"{key}"' in PLUGIN_SOURCE, key
    # the plugin numbers the axles in this order
    order = PLUGIN_SOURCE.split('kFaces{')[1].split('}')[0].replace('"', '').replace(' ', '')
    assert tuple(order.split(',')) == rm.AXLE_FACES


def test_mechanism_parameters_are_consistent():
    mech = CONFIG['mechanism']
    assert mech['breakaway_torque'] == pytest.approx(
        mech['layer_friction'] + mech['detent_torque'])
    # the detent can pull a free layer square against its friction, down to the dead band
    assert mech['detent_torque'] > mech['layer_friction']
    assert mech['snap_tolerance'] <= 1.75e-3 < mech['detent_window'] < 0.78
    # a layer that nothing holds rests well inside the tolerance of an axis change
    assert 2 * mech['layer_friction'] / mech['detent_stiffness'] < mech['snap_tolerance']
    # the detent is an explicit spring on the lightest thing it moves, one layer of 9 cubies:
    # stable only well below 2 / step
    cube = CONFIG['cube']
    inertia = rm.layer_inertia(CONFIG)
    assert inertia == pytest.approx(1.46e-5, rel=0.02)
    assert (mech['detent_stiffness'] / inertia) ** 0.5 * 0.001 < 1.2
