"""Frontier extraction over a nav_msgs/OccupancyGrid raster.

"""

from collections import deque

import numpy as np

OCCUPIED_THRESHOLD = 50


def _frontier_mask(grid):
    """Free cells that touch unknown space in 4-connectivity."""
    unknown = grid < 0
    free = (grid >= 0) & (grid < OCCUPIED_THRESHOLD)

    touches_unknown = np.zeros_like(unknown)
    touches_unknown[1:, :] |= unknown[:-1, :]
    touches_unknown[:-1, :] |= unknown[1:, :]
    touches_unknown[:, 1:] |= unknown[:, :-1]
    touches_unknown[:, :-1] |= unknown[:, 1:]

    return free & touches_unknown


def _clusters(mask, min_cells):
    """8-connected connected components over a boolean mask."""
    visited = np.zeros(mask.shape, dtype=bool)
    height, width = mask.shape
    out = []

    for row0, col0 in zip(*np.nonzero(mask)):
        if visited[row0, col0]:
            continue
        visited[row0, col0] = True
        queue = deque([(row0, col0)])
        cells = []
        while queue:
            row, col = queue.popleft()
            cells.append((row, col))
            for d_row in (-1, 0, 1):
                for d_col in (-1, 0, 1):
                    n_row, n_col = row + d_row, col + d_col
                    if (
                        0 <= n_row < height
                        and 0 <= n_col < width
                        and mask[n_row, n_col]
                        and not visited[n_row, n_col]
                    ):
                        visited[n_row, n_col] = True
                        queue.append((n_row, n_col))
        if len(cells) >= min_cells:
            out.append(np.array(cells))
    return out


def has_clearance(grid, row, col, radius_cells):
    """True when no occupied cell lies within radius_cells of (row, col)."""
    height, width = grid.shape
    r_lo = max(0, row - radius_cells)
    r_hi = min(height, row + radius_cells + 1)
    c_lo = max(0, col - radius_cells)
    c_hi = min(width, col + radius_cells + 1)
    return not np.any(grid[r_lo:r_hi, c_lo:c_hi] >= OCCUPIED_THRESHOLD)


def find_frontiers(grid, resolution, origin_x, origin_y, min_cluster_cells=6,
                   clearance_cells=0):
    """Locate frontier clusters and return them as world-frame candidates.

    Each candidate is a dict with `x`, `y` (metres, grid frame) and `size`
    (cell count). The representative point is the cluster member closest to
    the cluster centroid, so it always lands on a real free cell even when the
    frontier curves around a corner and its centroid does not.
    """
    candidates = []
    for cells in _clusters(_frontier_mask(grid), min_cluster_cells):
        mean_row = cells[:, 0].mean()
        mean_col = cells[:, 1].mean()

        if clearance_cells > 0:
            keep = np.array(
                [has_clearance(grid, r, c, clearance_cells) for r, c in cells]
            )
            if not keep.any():
                continue
            cells = cells[keep]

        distances = (cells[:, 0] - mean_row) ** 2 + (cells[:, 1] - mean_col) ** 2
        row, col = cells[int(np.argmin(distances))]

        candidates.append(
            {
                "x": origin_x + (col + 0.5) * resolution,
                "y": origin_y + (row + 0.5) * resolution,
                "size": int(cells.shape[0]),
            }
        )
    return candidates
