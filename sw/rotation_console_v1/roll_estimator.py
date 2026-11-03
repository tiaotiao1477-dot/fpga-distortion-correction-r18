"""Conservative visual roll estimation; no gravity sensor and no hardware IO.

Angles here are line slopes in image coordinates (x right, y down). The FPGA's
positive output rotation is counterclockwise, so add this residual to its known
current absolute angle. A chessboard is a reference chosen by the user.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math

import cv2
import numpy as np

cv2.setNumThreads(2)


@dataclass
class RollEstimate:
    accepted: bool
    method: str
    roll_deg: float | None
    confidence: float
    reason: str
    details: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


def _wrap(angle: float, period: float) -> float:
    return (angle + period / 2.0) % period - period / 2.0


def _distance(values: np.ndarray, center: float, period: float) -> np.ndarray:
    return (values - center + period / 2.0) % period - period / 2.0


def _circular_mean(values: list[float] | np.ndarray,
                   weights: list[float] | np.ndarray, period: float) -> tuple[float, float]:
    angles = np.asarray(values, dtype=float) * (2 * math.pi / period)
    w = np.asarray(weights, dtype=float)
    z = np.sum(w * np.exp(1j * angles)) / float(w.sum())
    return _wrap(float(np.angle(z)) * period / (2 * math.pi), period), float(abs(z))


def _tls(points: np.ndarray) -> tuple[float, float, float]:
    centered = points.astype(float) - points.mean(axis=0)
    _, _, vectors = np.linalg.svd(centered, full_matrices=False)
    direction = vectors[0]
    theta = _wrap(math.degrees(math.atan2(direction[1], direction[0])), 180.0)
    rms = float(np.sqrt(np.mean((centered @ vectors[-1]) ** 2)))
    extent = float(np.ptp(centered @ direction))
    return theta, rms, extent


def _chessboard(gray: np.ndarray) -> dict:
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, (12, 8), flags=flags)
    rec = {"found": bool(found), "accepted": False, "method": "chessboard"}
    if not found or corners is None or len(corners) != 96:
        rec["reason"] = "no_complete_12x8_grid"
        return rec
    grid = corners.reshape(8, 12, 2).astype(float)
    span = np.ptp(grid.reshape(-1, 2), axis=0)
    rows = [_tls(row) for row in grid]
    columns = [_tls(col) for col in grid.transpose(1, 0, 2)]
    row_angle, row_strength = _circular_mean([r[0] for r in rows], [r[2] for r in rows], 180)
    col_angle, col_strength = _circular_mean([_wrap(c[0] - 90, 180) for c in columns],
                                            [c[2] for c in columns], 180)
    disagreement = abs(_wrap(row_angle - col_angle, 180))
    mean_rms = float(np.mean([x[1] for x in rows + columns]))
    rec.update(corners=96, span_pixels=span.tolist(), row_roll_deg=row_angle,
               column_roll_deg=col_angle, family_disagreement_deg=disagreement,
               mean_tls_rms_pixels=mean_rms)
    if span[0] < 140 or span[1] < 80:
        rec["reason"] = "reference_too_small"
        return rec
    if disagreement > 10 or min(row_strength, col_strength) < .97 or mean_rms > 1.2:
        rec["reason"] = "grid_perspective_or_line_families_inconsistent"
        return rec
    roll, strength = _circular_mean([row_angle, col_angle], [1.0, 1.0], 180)
    if abs(roll) >= 89:
        rec["reason"] = "reference_near_90_degree_orientation_ambiguity"
        return rec
    rec.update(accepted=True, roll_deg=roll,
               confidence=float(min(.99, strength * min(row_strength, col_strength))),
               reason="full_grid_consistent_axes")
    return rec


def _scene(gray: np.ndarray) -> dict:
    height, width = gray.shape
    x0, y0 = round(width * .1), round(height * .1)
    roi = gray[y0:height-y0, x0:width-x0]
    rec = {"found": False, "accepted": False, "method": "scene_lines"}
    if float(np.std(roi)) < 8:
        rec["reason"] = "insufficient_scene_contrast"
        return rec
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    median = float(np.median(blur))
    low = max(30, min(80, .66 * median))
    high = max(80, min(200, 1.33 * median))
    edges = cv2.Canny(blur, low, high, L2gradient=True)
    raw = cv2.HoughLinesP(edges, 1, math.pi / 720, threshold=45,
                          minLineLength=max(60, round(roi.shape[1] * .1)), maxLineGap=8)
    if raw is None:
        rec["reason"] = "no_long_scene_lines"
        return rec
    # Collapse close parallel segments of the same physical edge before voting.
    unique: dict[tuple[int, int], dict] = {}
    for x1, y1, x2, y2 in raw.reshape(-1, 4):
        dx, dy = float(x2-x1), float(y2-y1)
        length = math.hypot(dx, dy)
        theta = _wrap(math.degrees(math.atan2(dy, dx)), 180)
        roll = _wrap(theta, 90)
        mid = ((float(x1+x2)/2), (float(y1+y2)/2))
        t = math.radians(theta)
        normal_distance = -math.sin(t) * mid[0] + math.cos(t) * mid[1]
        key = (round(theta / 3), round(normal_distance / 12))
        line = dict(roll=roll, length=length, midpoint=mid,
                    family="horizontal" if abs(theta) < 45 else "vertical")
        if key not in unique or unique[key]["length"] < length:
            unique[key] = line
    lines = list(unique.values())
    rec.update(found=True, distinct_long_lines=len(lines))
    if len(lines) < 8:
        rec["reason"] = "too_few_distinct_scene_edges"
        return rec
    angles = np.array([x["roll"] for x in lines], dtype=float)
    weights = np.array([x["length"] for x in lines], dtype=float)
    centers = np.arange(-44.5, 45.0, 1.0)
    votes = np.array([float(weights[np.abs(_distance(angles, c, 90)) <= 3].sum())
                      for c in centers])
    peak_index = int(np.argmax(votes))
    peak = float(centers[peak_index])
    support_mask = np.abs(_distance(angles, peak, 90)) <= 3
    support = [line for line, keep in zip(lines, support_mask) if keep]
    support_weight = float(weights[support_mask].sum())
    fraction = support_weight / float(weights.sum())
    competitor_mask = np.abs(_distance(centers, peak, 90)) > 8
    competitor = float(votes[competitor_mask].max()) if competitor_mask.any() else 0.0
    horizontal = sum(x["family"] == "horizontal" for x in support)
    vertical = len(support) - horizontal
    points = np.array([x["midpoint"] for x in support], dtype=float)
    tiles = {(min(3, int(p[0] / roi.shape[1] * 4)),
              min(2, int(p[1] / roi.shape[0] * 3))) for p in points}
    spans = np.ptp(points, axis=0) if len(points) else np.zeros(2)
    rec.update(inlier_lines=len(support), horizontal_lines=horizontal, vertical_lines=vertical,
               weighted_inlier_fraction=fraction, competing_peak_ratio=competitor/max(support_weight, 1),
               occupied_tiles=len(tiles), midpoint_span_pixels=spans.tolist())
    if (len(support) < 8 or horizontal < 2 or vertical < 2 or len(tiles) < 4 or
            fraction < .60 or competitor > support_weight * .65 or
            support_weight < .7 * sum(roi.shape) or
            spans[0] < .35 * roi.shape[1] or spans[1] < .35 * roi.shape[0]):
        rec["reason"] = "scene_reference_ambiguous_or_poorly_distributed"
        return rec
    roll, strength = _circular_mean(angles[support_mask], weights[support_mask], 90)
    if abs(roll) >= 40 or strength < .97:
        rec["reason"] = "scene_orientation_ambiguous_near_45_degrees"
        return rec
    rec.update(accepted=True, roll_deg=roll, confidence=float(min(.95, fraction * strength)),
               reason="distributed_horizontal_vertical_consensus")
    return rec


def _combine(records: list[dict], method: str, period: float) -> RollEstimate:
    good = [x for x in records if x.get("accepted")]
    details = {"frame_candidates": records, "required_consistent_frames": 3,
               "maximum_frame_spread_deg": 1.5,
               "reference_is_not_gravity": True, "upside_down_180_not_resolved": True}
    if len(good) < 3:
        return RollEstimate(False, method, None, 0.0, "fewer_than_three_reliable_frames", details)
    angles = np.array([x["roll_deg"] for x in good], dtype=float)
    weights = np.array([x["confidence"] for x in good], dtype=float)
    medoid = min(angles, key=lambda c: float(np.sum(weights * np.abs(_distance(angles, c, period)))))
    keep = np.abs(_distance(angles, float(medoid), period)) <= 1.5
    if int(keep.sum()) < 3:
        return RollEstimate(False, method, None, 0.0, "reference_moving_or_temporally_inconsistent", details)
    angle, strength = _circular_mean(angles[keep], weights[keep], period)
    spread = float(np.max(np.abs(_distance(angles[keep], angle, period))))
    details.update(consistent_frames=int(keep.sum()), accepted_frame_count=len(good),
                   spread_deg=spread, period_degrees=period)
    if spread > 1.5:
        return RollEstimate(False, method, None, 0.0, "temporal_spread_exceeds_limit", details)
    confidence = float(np.median(weights[keep]) * strength * (.9 + .1 * int(keep.sum()) / len(records)))
    return RollEstimate(True, method, angle, confidence, "consistent_visual_reference", details)


def estimate_roll(frames: list[np.ndarray]) -> RollEstimate:
    """Prefer a complete chessboard; refuse unstable boards instead of using background."""
    if len(frames) < 3:
        return RollEstimate(False, "none", None, 0.0, "need_at_least_three_fresh_frames")
    grays = []
    for image in frames:
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape != (1080, 1920, 3):
            raise ValueError("Expected full-HD1920x1080 uint8 BGR capture frames")
        resized = cv2.resize(image, (960, 540), interpolation=cv2.INTER_AREA)
        grays.append(cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY))
    boards = [_chessboard(gray) for gray in grays]
    if any(x["found"] for x in boards):
        return _combine(boards, "chessboard", 180)
    scenes = [_scene(gray) for gray in grays]
    result = _combine(scenes, "scene_lines", 90)
    result.details["chessboard_found_frames"] = 0
    return result
