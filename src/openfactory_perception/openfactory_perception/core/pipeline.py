"""Geometric perception baseline: one depth image -> object hypotheses in the bin frame.

Steps (docs/agent-guides/perception.md): depth -> points, transform to the bin frame, crop to
the bin volume and remove the floor, segment on a top-view height map, recognise each cluster.
ROS-free; the node supplies the intrinsics (camera_info) and the bin-from-camera transform (TF).
"""

from dataclasses import dataclass

import numpy as np

from . import camera, primitives, segmentation


@dataclass(frozen=True)
class Debug:
    """Intermediate data for debug topics and evaluation tools."""

    points: np.ndarray            # N x 3 object points in the bin frame
    labels: np.ndarray            # N cluster ids (0: no cluster)
    pixels: np.ndarray            # N x 2 image (v, u) of each point
    cluster_ids: tuple            # id of the cluster behind each detection, same order


def detect(depth, intrinsics, bin_from_optical, inner_size, models, params):
    """(detections sorted by decreasing score, Debug) for one depth image.

    `depth`: H x W z-depth in metres, invalid pixels NaN, inf or 0; `intrinsics`: camera
    .Intrinsics; `bin_from_optical`: 4 x 4 transform of optical-frame points into the bin
    frame; `inner_size`: bin inner (x, y, wall height); `models`: primitives.shape_models();
    `params`: config/perception.yaml.
    """
    optical, pixels = camera.depth_to_points(depth, intrinsics, *params['depth_range'])
    points = camera.transform_points(bin_from_optical, optical)
    inside = segmentation.crop_to_bin(points, inner_size, params['wall_margin'],
                                      params['floor_threshold'], params['max_object_height'])
    points, pixels = points[inside], pixels[inside]
    if len(points) == 0:
        return [], Debug(points, np.zeros(0, dtype=np.intp), pixels, ())

    cell = params['cell']
    heights, count = segmentation.height_map(points, inner_size, cell)
    labels, blobs = segmentation.segment(
        count, int(round(params['min_area'] / (cell * cell))),
        params['close_cells'], params['open_cells'])
    row, col = segmentation.grid_indices(points, inner_size, cell)
    point_labels = labels[row, col]

    found = []
    for blob in blobs:
        cluster = points[point_labels == blob]
        cell_heights = heights[(labels == blob) & (count > 0)]
        if len(cluster) < 3 or len(cell_heights) == 0:
            continue
        features = primitives.measure(cluster, cell_heights, cell, params['flat_tolerance'])
        found.append((primitives.classify(features, models, params), blob))
    found.sort(key=lambda item: -item[0].score)
    return ([d for d, _ in found],
            Debug(points, point_labels, pixels, tuple(blob for _, blob in found)))
