"""Bin crop, floor removal and object segmentation on a top-view height map. ROS-free.

Points are in the bin frame: origin at the centre of the inner floor, z up, x/y along the bin
sides. The bin geometry is fixed infrastructure (cell.yaml), so the floor is the known plane
z = 0 and the walls are the known sides of the inner volume.
"""

import cv2
import numpy as np


def crop_to_bin(points, inner_size, wall_margin, floor_threshold, max_height):
    """Mask of the points above the floor and inside the bin, `wall_margin` away from the walls.

    `floor_threshold` removes the floor (and its depth noise); `max_height` removes whatever
    is above the tallest object that can rest in the bin, e.g. the arm.
    """
    half_x = inner_size[0] / 2.0 - wall_margin
    half_y = inner_size[1] / 2.0 - wall_margin
    return ((np.abs(points[:, 0]) < half_x) & (np.abs(points[:, 1]) < half_y)
            & (points[:, 2] > floor_threshold) & (points[:, 2] < max_height))


def grid_shape(inner_size, cell):
    """(rows along y, columns along x) of the top-view grid over the inner floor."""
    return int(np.ceil(inner_size[1] / cell)), int(np.ceil(inner_size[0] / cell))


def grid_indices(points, inner_size, cell):
    """(row, column) of each point in the top-view grid; points must lie inside the bin."""
    rows, cols = grid_shape(inner_size, cell)
    col = np.clip(((points[:, 0] + inner_size[0] / 2.0) / cell).astype(np.intp), 0, cols - 1)
    row = np.clip(((points[:, 1] + inner_size[1] / 2.0) / cell).astype(np.intp), 0, rows - 1)
    return row, col


def height_map(points, inner_size, cell):
    """(mean height per cell, NaN where empty; points per cell) for object points."""
    shape = grid_shape(inner_size, cell)
    row, col = grid_indices(points, inner_size, cell)
    flat = row * shape[1] + col
    count = np.bincount(flat, minlength=shape[0] * shape[1])
    total = np.bincount(flat, weights=points[:, 2], minlength=shape[0] * shape[1])
    with np.errstate(invalid='ignore', divide='ignore'):
        mean = np.where(count > 0, total / count, np.nan)
    return mean.reshape(shape), count.reshape(shape)


def segment(count, min_cells, close_cells=3, open_cells=3):
    """Label connected blobs of occupied cells: (labels with 0 = background, blob ids).

    Closing bridges the cells that a sparse view of a slanted face leaves empty, opening removes
    specks (isolated floor pixels pushed above the threshold by depth noise); blobs smaller
    than `min_cells` are dropped.
    """
    occupied = (count > 0).astype(np.uint8)
    if close_cells > 1:
        occupied = cv2.morphologyEx(occupied, cv2.MORPH_CLOSE,
                                    np.ones((close_cells, close_cells), np.uint8))
    if open_cells > 1:
        occupied = cv2.morphologyEx(occupied, cv2.MORPH_OPEN,
                                    np.ones((open_cells, open_cells), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    keep = [label for label in range(1, n) if stats[label, cv2.CC_STAT_AREA] >= min_cells]
    return labels, keep
