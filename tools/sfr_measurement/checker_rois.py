"""Checkerboard detection and stable axis-aligned slanted-edge ROI selection."""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import cv2
import numpy as np


DEFAULT_TARGETS = {
    "center": (0.5, 0.5),
    "top_left": (0.2, 0.2),
    "top_right": (0.8, 0.2),
    "bottom_left": (0.2, 0.8),
    "bottom_right": (0.8, 0.8),
}


def _canonicalize_grid(points: np.ndarray) -> np.ndarray:
    """Resolve the 180-degree OpenCV ordering ambiguity for a near-5° chart."""
    grid = np.asarray(points, dtype=np.float64)
    if grid.ndim != 3 or grid.shape[2] != 2 or min(grid.shape[:2]) < 2:
        raise ValueError("corner grid must have shape (rows, columns, 2)")
    choices = (grid, grid[:, ::-1], grid[::-1, :], grid[::-1, ::-1])
    scored = []
    for choice in choices:
        col_step = np.median(choice[:, 1:, :] - choice[:, :-1, :], axis=(0, 1))
        row_step = np.median(choice[1:, :, :] - choice[:-1, :, :], axis=(0, 1))
        col_norm = max(1e-12, float(np.linalg.norm(col_step)))
        row_norm = max(1e-12, float(np.linalg.norm(row_step)))
        score = float(col_step[0] / col_norm + row_step[1] / row_norm)
        scored.append((score, choice))
    result = max(scored, key=lambda item: item[0])[1].copy()
    col_step = np.median(result[:, 1:, :] - result[:, :-1, :], axis=(0, 1))
    row_step = np.median(result[1:, :, :] - result[:-1, :, :], axis=(0, 1))
    if col_step[0] <= 0 or row_step[1] <= 0:
        raise ValueError("cannot canonicalize grid to columns right and rows down")
    return result


def detect_grid(image_bgr: np.ndarray, pattern: tuple[int, int] = (12, 8)) -> np.ndarray:
    """Detect and canonicalize an inner-corner grid; `pattern` is (cols, rows)."""
    image = np.asarray(image_bgr)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("imageBGR must be an 8-bit three-channel image")
    if len(pattern) != 2 or int(pattern[0]) < 2 or int(pattern[1]) < 2:
        raise ValueError("pattern must be (columns, rows), each at least 2")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
    found, corners = cv2.findChessboardCornersSB(gray, (int(pattern[0]), int(pattern[1])), flags=flags)
    if not found or corners is None:
        raise ValueError(f"could not detect checkerboard pattern {pattern}")
    grid = np.asarray(corners, dtype=np.float64).reshape(int(pattern[1]), int(pattern[0]), 2)
    return _canonicalize_grid(grid)


def _shape_hw(image_shape: Sequence[int]) -> tuple[int, int]:
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    height, width = int(image_shape[0]), int(image_shape[1])
    if height <= 0 or width <= 0:
        raise ValueError("image_shape dimensions must be positive")
    return height, width


def _normal_spacing(grid: np.ndarray, row: int, col: int, axis: str) -> float:
    rows, cols = grid.shape[:2]
    distances: list[float] = []
    if axis == "x":
        for rr in (row, row + 1):
            for cc in (col - 1, col):
                if 0 <= cc < cols - 1:
                    distances.append(float(np.linalg.norm(grid[rr, cc + 1] - grid[rr, cc])))
    else:
        for cc in (col, col + 1):
            for rr in (row - 1, row):
                if 0 <= rr < rows - 1:
                    distances.append(float(np.linalg.norm(grid[rr + 1, cc] - grid[rr, cc])))
    if not distances:
        return 0.0
    return float(np.median(distances))


def safe_edge_window(start: Sequence[float], end: Sequence[float], normal_spacing: float,
                     image_shape: Sequence[int], *, min_size: int = 48,
                     safety_margin: float = 2.0) -> dict[str, Any]:
    """Return a native axis-aligned crop whose corners stay inside one cell.

    Keep the established 64%-edge / 50%-normal preferred crop. Expand only a
    dimension below ``min_size``, then project every pixel-corner into the
    edge tangent/normal frame and reject windows that cross the cell margins.
    """
    a, b = np.asarray(start, dtype=np.float64), np.asarray(end, dtype=np.float64)
    spacing = float(normal_spacing)
    image_height, image_width = _shape_hw(image_shape)
    if a.shape != (2,) or b.shape != (2,) or not np.isfinite([*a, *b]).all():
        raise ValueError("edge endpoints must be finite 2D points")
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("normal cell spacing must be finite and positive")
    if isinstance(min_size, bool) or int(min_size) != min_size or min_size < 1:
        raise ValueError("min_size must be a positive integer")
    delta = b - a
    length = float(np.linalg.norm(delta))
    if length <= 1e-6:
        raise ValueError("edge segment is degenerate")
    tangent = delta / length
    normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
    center = 0.5 * (a + b)
    half_length = 0.32 * length
    half_width = min(0.25 * spacing, 30.0)
    extent = half_length * np.abs(tangent) + half_width * np.abs(normal)
    roi_width = max(int(min_size), int(math.ceil(2.0 * extent[0])) + 1)
    roi_height = max(int(min_size), int(math.ceil(2.0 * extent[1])) + 1)
    x0 = int(round(center[0] - 0.5 * (roi_width - 1)))
    y0 = int(round(center[1] - 0.5 * (roi_height - 1)))
    x1, y1 = x0 + roi_width, y0 + roi_height
    if x0 < 0 or y0 < 0 or x1 > image_width or y1 > image_height:
        raise ValueError("safe edge window would cross the source-image boundary")

    pixel_corners = np.array([[x0, y0], [x1 - 1, y0],
                              [x0, y1 - 1], [x1 - 1, y1 - 1]], dtype=np.float64)
    offsets = pixel_corners - center
    along_extent = float(np.max(np.abs(offsets @ tangent)))
    normal_extent = float(np.max(np.abs(offsets @ normal)))
    if (along_extent > length * 0.5 - safety_margin + 1e-9 or
            normal_extent > spacing * 0.5 - safety_margin + 1e-9):
        raise ValueError("minimum native window cannot fit inside one checker cell with safety margin")
    return {"x": x0, "y": y0, "width": roi_width, "height": roi_height,
            "center": center.tolist(), "grid_endpoints": [a.tolist(), b.tolist()],
            "edge_length_px": length, "normal_cell_spacing_px": spacing,
            "roi_geometry": "axis_aligned_original_pixel_crop_no_pc_resampling",
            "safe_projection_along_px": along_extent,
            "safe_projection_normal_px": normal_extent}


def candidate_rois_with_diagnostics(grid: np.ndarray, image_shape: Sequence[int]):
    """Return safe candidate windows and counts for geometry-based rejections."""
    points = np.asarray(grid, dtype=np.float64)
    if points.ndim != 3 or points.shape[2] != 2 or min(points.shape[:2]) < 2:
        raise ValueError("grid shape must be (rows, columns, 2)")
    if not np.isfinite(points).all():
        raise ValueError("grid coordinates must be finite")
    points = _canonicalize_grid(points)
    image_height, image_width = _shape_hw(image_shape)
    rows, cols = points.shape[:2]
    output: list[dict[str, Any]] = []
    diagnostics = {"totalEdges": 0, "accepted": 0, "tooSmallForCell": 0,
                   "outsideImage": 0, "otherGeometryRejects": 0}

    def add(edge_id: str, orientation: str, start: np.ndarray, end: np.ndarray,
            normal_spacing: float) -> None:
        diagnostics["totalEdges"] += 1
        try:
            window = safe_edge_window(start, end, normal_spacing, image_shape)
        except ValueError as exc:
            if "source-image boundary" in str(exc):
                diagnostics["outsideImage"] += 1
            elif "one checker cell" in str(exc):
                diagnostics["tooSmallForCell"] += 1
            else:
                diagnostics["otherGeometryRejects"] += 1
            return
        delta = np.asarray(end, dtype=np.float64) - np.asarray(start, dtype=np.float64)
        geometric_orientation = "vertical" if abs(delta[1]) >= abs(delta[0]) else "horizontal"
        window.update(edge_id=edge_id, orientation=geometric_orientation,
                      normalized_position=[float(window["center"][0] / max(1, image_width - 1)),
                                           float(window["center"][1] / max(1, image_height - 1))])
        output.append(window)
        diagnostics["accepted"] += 1

    for row in range(rows - 1):
        for col in range(cols):
            add(f"v_{row}_{col}", "vertical", points[row, col], points[row + 1, col],
                _normal_spacing(points, row, col, "x"))
    for row in range(rows):
        for col in range(cols - 1):
            add(f"h_{row}_{col}", "horizontal", points[row, col], points[row, col + 1],
                _normal_spacing(points, row, col, "y"))
    return sorted(output, key=lambda item: item["edge_id"]), diagnostics


def candidate_rois(grid: np.ndarray, image_shape: Sequence[int]) -> list[dict[str, Any]]:
    """Build axis-aligned, cell-contained native-pixel edge crops."""
    return candidate_rois_with_diagnostics(grid, image_shape)[0]


def select_common_edges(
    candidates_by_mode: Mapping[str, Sequence[Mapping[str, Any]]],
    targets: Mapping[str, Sequence[float]] = DEFAULT_TARGETS,
) -> dict[str, dict[str, Any]]:
    """Choose deterministic, unique same-edge ROIs for each target/orientation.

    Selection minimizes normalized-image distance from each target using the
    mean position across modes. It does not inspect SFR/MTF values. Coverage
    records describe sampled chart positions only; they do not claim the full
    sensor field was covered.
    """
    if not candidates_by_mode or any(not name for name in candidates_by_mode):
        raise ValueError("common-edge selection requires named modes")
    modes = sorted(candidates_by_mode)
    indexed: dict[str, dict[str, Mapping[str, Any]]] = {}
    for mode in modes:
        index: dict[str, Mapping[str, Any]] = {}
        for candidate in candidates_by_mode[mode]:
            edge_id = str(candidate.get("edge_id", ""))
            if edge_id:
                index[edge_id] = candidate
        indexed[mode] = index
    common = set.intersection(*(set(indexed[mode]) for mode in modes))
    if not common:
        raise ValueError("no common edge IDs across modes")
    for edge_id in tuple(common):
        orientations = {str(indexed[mode][edge_id].get("orientation")) for mode in modes}
        if len(orientations) != 1 or next(iter(orientations)) not in ("vertical", "horizontal"):
            common.remove(edge_id)
    if not common:
        raise ValueError("no common edges with consistent orientation")

    chart_bbox: dict[str, list[float] | None] = {}
    for mode in modes:
        positions = [np.asarray(item.get("normalized_position"), dtype=np.float64)
                     for item in indexed[mode].values() if item.get("normalized_position") is not None]
        if not positions:
            chart_bbox[mode] = None
        else:
            matrix = np.vstack(positions)
            chart_bbox[mode] = [float(matrix[:, 0].min()), float(matrix[:, 1].min()),
                                float(matrix[:, 0].max()), float(matrix[:, 1].max())]

    output: dict[str, dict[str, Any]] = {}
    for target_name, target_xy in targets.items():
        target = np.asarray(target_xy, dtype=np.float64)
        if target.shape != (2,) or not np.isfinite(target).all() or np.any(target < 0) or np.any(target > 1):
            raise ValueError(f"target {target_name!r} must be a finite normalized (x,y) pair")
        for orientation in ("vertical", "horizontal"):
            key = f"{target_name}_{orientation}"
            options = []
            for edge_id in common:
                first = indexed[modes[0]][edge_id]
                if first.get("orientation") != orientation:
                    continue
                positions = {mode: np.asarray(indexed[mode][edge_id]["normalized_position"], dtype=np.float64)
                             for mode in modes}
                mean_position = np.mean(np.vstack(list(positions.values())), axis=0)
                options.append((float(np.linalg.norm(mean_position - target)), edge_id, positions, mean_position))
            options.sort(key=lambda option: (option[0], option[1]))
            chosen = next((item for item in options
                           if item[1] not in {entry["edge_id"] for out_key, entry in output.items()
                                              if out_key.endswith("_" + orientation) and entry["edge_id"]}), None)
            if chosen is None:
                output[key] = {"target": target.tolist(), "orientation": orientation, "edge_id": None,
                               "status": "no_unique_common_edge", "coverage": {
                                   "within_chart_only": True, "candidate_chart_bbox_by_mode": chart_bbox}}
                continue
            distance, edge_id, positions, mean_position = chosen
            samples = np.vstack(list(positions.values()))
            output[key] = {
                "target": target.tolist(),
                "orientation": orientation,
                "edge_id": edge_id,
                "status": "selected_common_edge",
                "mode_positions": {mode: positions[mode].tolist() for mode in modes},
                "mean_position": mean_position.tolist(),
                "target_distance_normalized": distance,
                "coverage": {
                    "within_chart_only": True,
                    "selected_edge_bbox_across_modes": [float(samples[:, 0].min()), float(samples[:, 1].min()),
                                                         float(samples[:, 0].max()), float(samples[:, 1].max())],
                    "candidate_chart_bbox_by_mode": chart_bbox,
                },
            }
    return output
