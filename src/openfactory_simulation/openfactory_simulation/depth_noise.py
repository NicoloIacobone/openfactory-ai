"""Depth error model of the simulated RGB-D camera (D-011). ROS-free.

Gazebo renders ideal z-depth. A stereo depth camera such as the RealSense D415 measures
disparity d = f B / z with a roughly constant subpixel error, so its depth error grows with
z^2: sigma_z = z^2 sigma_d / (f B). The model adds independent Gaussian disparity noise per
pixel, rounds to the depth unit of the stream and invalidates pixels outside [min_z, max_z].

It is not a stereo-matching simulation: no spatially correlated error, no holes on shiny or
textureless surfaces, no flying pixels at depth edges, no invalid band at the image border.
"""

import numpy as np

INVALID = np.nan                # REP 118: invalid depth in a 32FC1 image


def depth_sigma(z, fx, baseline, sigma_px):
    """Standard deviation of the depth error at depth z (metres)."""
    return np.asarray(z, dtype=np.float64) ** 2 * sigma_px / (fx * baseline)


def frame_rng(seed, sec, nanosec):
    """Generator for one frame: the same seed and image stamp give the same noise."""
    return np.random.default_rng([int(seed), int(sec), int(nanosec)])


def apply_depth_model(depth, fx, params, rng=None):
    """Measured depth image (float32, metres, NaN = invalid) for an ideal one.

    `params`: min_z, max_z, baseline, disparity_noise_px, quantization (camera.yaml `depth`).
    With `rng` None the depth stays ideal and only the range limits apply. Non-finite and
    non-positive input pixels (no surface within the render range) are invalid.
    """
    ideal = np.asarray(depth, dtype=np.float64)
    valid = np.isfinite(ideal) & (ideal > 0.0)
    z = ideal[valid]
    if rng is not None:
        focal_baseline = fx * params['baseline']
        disparity = focal_baseline / z + rng.normal(0.0, params['disparity_noise_px'], z.shape)
        z = np.where(disparity > 0.0, focal_baseline / np.maximum(disparity, 1e-9), np.inf)
        if params['quantization'] > 0.0:
            z = np.round(z / params['quantization']) * params['quantization']
    z[(z < params['min_z']) | (z > params['max_z'])] = INVALID
    measured = np.full(ideal.shape, INVALID, dtype=np.float32)
    measured[valid] = z
    return measured
