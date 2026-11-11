import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openfactory_simulation import depth_noise as dn  # noqa: E402

CAMERA = yaml.safe_load(
    (Path(__file__).resolve().parents[2] / 'openfactory_description' / 'config' / 'camera.yaml')
    .read_text())
PARAMS = CAMERA['depth']
FX = (CAMERA['image']['width'] / 2) / math.tan(math.radians(CAMERA['image']['hfov_deg']) / 2)


def test_sigma_grows_with_the_square_of_depth():
    sigma = dn.depth_sigma([0.5, 1.0], FX, PARAMS['baseline'], PARAMS['disparity_noise_px'])
    assert sigma[1] == pytest.approx(4 * sigma[0])
    # the level the camera is modelled with: about 1.4 mm at the bin floor distance
    assert dn.depth_sigma(0.7, FX, PARAMS['baseline'], 0.08) == pytest.approx(0.00142, abs=2e-5)


@pytest.mark.parametrize('z', [0.5, 0.7, 1.0])
def test_measured_noise_matches_the_model(z):
    params = {**PARAMS, 'quantization': 0.0}
    measured = dn.apply_depth_model(np.full((200, 200), z), FX, params, dn.frame_rng(1, 0, 0))
    expected = dn.depth_sigma(z, FX, params['baseline'], params['disparity_noise_px'])
    assert np.isfinite(measured).all()
    assert measured.std() == pytest.approx(expected, rel=0.05)
    assert measured.mean() == pytest.approx(z, abs=expected / 20)


def test_quantization_to_the_depth_unit():
    measured = dn.apply_depth_model(np.full((50, 50), 0.7), FX, PARAMS, dn.frame_rng(1, 0, 0))
    steps = measured.astype(np.float64) / PARAMS['quantization']
    assert np.abs(steps - np.round(steps)).max() < 1e-3
    assert len(np.unique(measured)) > 3


def test_range_limits_and_invalid_input():
    ideal = np.array([[0.10, 0.239, 0.241, 1.0, 1.999, 2.5, np.inf, -np.inf, np.nan, 0.0]])
    measured = dn.apply_depth_model(ideal, FX, PARAMS)
    assert np.isnan(measured[0, [0, 1, 5, 6, 7, 8, 9]]).all()
    assert measured[0, [2, 3, 4]] == pytest.approx([0.241, 1.0, 1.999])
    assert measured.dtype == np.float32


def test_ideal_depth_is_untouched_without_a_generator():
    ideal = np.linspace(0.3, 1.9, 100).reshape(10, 10)
    assert dn.apply_depth_model(ideal, FX, PARAMS) == pytest.approx(ideal, abs=1e-6)


def test_same_seed_and_stamp_give_the_same_frame():
    ideal = np.full((20, 20), 0.7)
    a = dn.apply_depth_model(ideal, FX, PARAMS, dn.frame_rng(3, 12, 400))
    b = dn.apply_depth_model(ideal, FX, PARAMS, dn.frame_rng(3, 12, 400))
    c = dn.apply_depth_model(ideal, FX, PARAMS, dn.frame_rng(3, 12, 600))
    d = dn.apply_depth_model(ideal, FX, PARAMS, dn.frame_rng(4, 12, 400))
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c) and not np.array_equal(a, d)
