import math

import pytest

import hold_probe_eval as hp
from openfactory_manipulation.core.geometry import quat_from_rpy, quat_multiply

THIRD = 2 * math.pi / 3


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.1])
@pytest.mark.parametrize('roll', [0.0, 0.25, -0.6, 2.0])
def test_roll_about_the_object_x_axis_whatever_its_yaw(yaw, roll):
    initial = quat_from_rpy(0.0, 0.0, yaw)
    turned = quat_multiply(initial, quat_from_rpy(roll, 0.0, 0.0))    # about the object's own x
    assert hp.roll_about_x(initial, turned) == pytest.approx(roll)
    assert hp.rotation_angle(initial, turned) == pytest.approx(abs(roll))


def test_a_turn_about_the_vertical_is_not_a_roll():
    initial = quat_from_rpy(0.0, 0.0, 0.3)
    assert hp.roll_about_x(initial, quat_from_rpy(0.0, 0.0, 0.5)) == pytest.approx(0.0)
    assert hp.rotation_angle(initial, quat_from_rpy(0.0, 0.0, 0.5)) == pytest.approx(0.2)


@pytest.mark.parametrize('roll, face', [
    (0.0, 0), (0.4, 0), (-0.4, 0), (THIRD, 1), (THIRD + 0.2, 1), (-THIRD, 2), (-THIRD - 0.3, 2)])
def test_face_index_of_the_prism(roll, face):
    assert hp.face_index(roll, THIRD) == face


def test_summarize_hold_counts_only_the_samples_in_the_air():
    rest = quat_from_rpy(0.0, 0.0, 0.5)

    def turned(roll):
        return quat_multiply(rest, quat_from_rpy(roll, 0.0, 0.0))

    samples = [
        (0.0, (0.4, -0.3, 0.78), rest),
        (1.0, (0.4, -0.3, 0.90), turned(0.05)),           # lifted
        (2.0, (0.4, 0.0, 0.93), turned(-0.30)),           # transfer
        (3.0, (0.4, 0.3, 0.93), turned(0.20)),
        (4.0, (0.4, 0.3, 0.77), turned(THIRD)),           # released, fell onto the next face
    ]
    result = hp.summarize_hold(samples, THIRD, air_margin=0.02)
    assert result['samples_held'] == 3
    assert result['max_roll_held_rad'] == pytest.approx(0.30)
    assert result['roll_before_release_rad'] == pytest.approx(0.20)
    assert result['final_roll_rad'] == pytest.approx(THIRD)
    assert result['final_face'] == 1 and result['face_changed']
    assert hp.summarize_hold([], THIRD, 0.02) is None


def test_summarize_hold_of_an_object_that_never_left_the_floor():
    rest = quat_from_rpy(0.0, 0.0, 0.0)
    result = hp.summarize_hold([(0.0, (0.0, 0.0, 0.02), rest), (1.0, (0.0, 0.0, 0.02), rest)],
                               THIRD, air_margin=0.02)
    assert result['samples_held'] == 0 and result['max_roll_held_rad'] is None
    assert not result['face_changed']


def test_deep_merge_leaves_the_base_untouched():
    base = {'grasp': {'floor_clearance': 0.005, 'lift_height': 0.15}, 'place': {'xy': [0, 0]}}
    merged = hp.deep_merge(base, {'grasp': {'floor_clearance': 0.012}, 'extra': 1})
    assert merged['grasp'] == {'floor_clearance': 0.012, 'lift_height': 0.15}
    assert merged['place'] == {'xy': [0, 0]} and merged['extra'] == 1
    assert base['grasp']['floor_clearance'] == 0.005 and 'extra' not in base
