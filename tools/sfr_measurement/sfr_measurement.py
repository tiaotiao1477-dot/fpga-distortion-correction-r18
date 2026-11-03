"""Engineering slanted-edge SFR from encoded-output grayscale ROIs.

This is not an ISO certification implementation and does not calibrate OECF.
Input pixels are used as received; no denoising or resampling is applied.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


SCOPE = "output_domain_engineering_esfr_not_iso_certified"


def _reject(reason: str) -> None:
    raise ValueError(reason)


def _fit_edge(image: np.ndarray) -> tuple[float, float, np.ndarray, np.ndarray, dict[str, Any]]:
    """Fit edge x=a*y+b from row-local half-level derivative centroids."""
    height, width = image.shape
    columns = np.arange(width, dtype=np.float64)
    edge_rows: list[float] = []
    edge_columns: list[float] = []
    ambiguous_rows = 0

    for row_index, row in enumerate(image):
        side = max(4, int(round(width * 0.15)))
        left = float(np.median(row[:side]))
        right = float(np.median(row[-side:]))
        contrast = abs(right - left)
        if contrast < 1e-9:
            continue
        midpoint = 0.5 * (left + right)
        delta = np.diff(row)
        straddles = ((row[:-1] - midpoint) * (row[1:] - midpoint) <= 0.0)
        # Keep only local derivative energy close to the measured half level.
        near_mid = (np.minimum(np.abs(row[:-1] - midpoint), np.abs(row[1:] - midpoint))
                    <= 0.55 * contrast)
        valid = straddles & near_mid & (np.abs(delta) > 1e-12)
        crossing_indices = np.flatnonzero(valid)
        if crossing_indices.size == 0:
            continue
        # Interpolate the half-level crossing inside each sampled pair. Using
        # pair centers quantizes row locations to half pixels and biases the
        # fitted line/slope as its phase walks across columns.
        fractions = ((midpoint - row[crossing_indices]) /
                     (row[crossing_indices + 1] - row[crossing_indices]))
        positions = crossing_indices.astype(np.float64) + np.clip(fractions, 0.0, 1.0)
        weights = np.abs(delta[crossing_indices])
        order = np.argsort(positions)
        positions, weights = positions[order], weights[order]
        groups: list[list[int]] = [[0]]
        for position_index in range(1, positions.size):
            if positions[position_index] - positions[groups[-1][-1]] <= 1.0:
                groups[-1].append(position_index)
            else:
                groups.append([position_index])
        group_weights = [float(weights[group].sum()) for group in groups]
        strongest = max(group_weights)
        if sum(weight >= 0.25 * strongest for weight in group_weights) > 1:
            ambiguous_rows += 1
            continue
        primary = groups[int(np.argmax(group_weights))]
        local_weights = weights[primary]
        local_positions = positions[primary]
        total = float(local_weights.sum())
        if total <= max(1e-9, contrast * 0.015):
            continue
        position = float(np.dot(local_positions, local_weights) / total)
        edge_rows.append(float(row_index))
        edge_columns.append(position)

    if ambiguous_rows > max(2, int(math.ceil(height * 0.05))):
        _reject("multiple ambiguous half-level transitions in ROI")
    if len(edge_rows) < max(20, int(math.ceil(height * 0.70))):
        _reject("edge fit has insufficient row coverage")

    ys = np.asarray(edge_rows, dtype=np.float64)
    xs = np.asarray(edge_columns, dtype=np.float64)
    keep = np.ones(len(ys), dtype=bool)
    slope, intercept = np.polyfit(ys, xs, 1)
    for _ in range(4):
        residual = xs - (slope * ys + intercept)
        center = float(np.median(residual[keep]))
        mad = float(np.median(np.abs(residual[keep] - center)))
        limit = max(0.45, 3.5 * 1.4826 * mad)
        updated = np.abs(residual - center) <= limit
        if int(updated.sum()) < max(20, int(math.ceil(height * 0.70))):
            _reject("edge fit has insufficient inlier coverage")
        if np.array_equal(updated, keep):
            break
        keep = updated
        slope, intercept = np.polyfit(ys[keep], xs[keep], 1)

    residual = xs[keep] - (slope * ys[keep] + intercept)
    fit_rms = float(np.sqrt(np.mean(residual * residual)))
    if fit_rms > 0.45:
        _reject(f"edge fit residual too large ({fit_rms:.3f}px)")
    return float(slope), float(intercept), ys[keep], xs[keep], {
        "fit_residual_px": fit_rms,
        "fit_rows": int(keep.sum()),
        "fit_row_fraction": float(keep.sum() / height),
        "fit_row_positions": ys[keep].tolist(),
        "fit_edge_positions": xs[keep].tolist(),
    }


def _first_crossing(frequencies: np.ndarray, sfr: np.ndarray, level: float) -> float | None:
    crossings = np.flatnonzero((sfr[:-1] >= level) & (sfr[1:] <= level))
    if not crossings.size:
        return None
    index = int(crossings[0])
    y0, y1 = float(sfr[index]), float(sfr[index + 1])
    x0, x1 = float(frequencies[index]), float(frequencies[index + 1])
    if y0 == y1:
        return x0
    return x0 + (level - y0) * (x1 - x0) / (y1 - y0)


def analyze_edge(roi: np.ndarray, *, orientation: str = "vertical", oversample: int = 4) -> dict[str, Any]:
    """Estimate SFR and MTF crossings for a near-vertical or near-horizontal edge.

    `roi` is a 2D grayscale array in its existing pixel domain. For
    `orientation='vertical'`, the edge runs mostly top-to-bottom; for
    `orientation='horizontal'`, it runs mostly left-to-right and the ROI is
    transposed internally. The ROI itself is never resampled or filtered.
    """
    if orientation not in ("vertical", "horizontal"):
        _reject("orientation must be 'vertical' or 'horizontal'")
    if isinstance(oversample, bool) or int(oversample) != oversample or not 2 <= int(oversample) <= 8:
        _reject("oversample must be an integer from 2 through 8")
    image = np.asarray(roi, dtype=np.float64)
    if image.ndim != 2:
        _reject("ROI must be a 2D grayscale array")
    if not np.isfinite(image).all():
        _reject("ROI contains nonfinite samples")
    if orientation == "horizontal":
        image = image.T
    height, width = image.shape
    if min(height, width) < 48:
        _reject("ROI size must be at least 48 by 48 pixels")

    side = max(4, int(round(width * 0.15)))
    left_median = float(np.median(image[:, :side]))
    right_median = float(np.median(image[:, -side:]))
    contrast = abs(right_median - left_median)
    if contrast < 20.0:
        _reject(f"edge contrast too low ({contrast:.3f} gray levels)")
    polarity = "increasing" if right_median > left_median else "decreasing"

    slope, intercept, fit_y, fit_x, fit_info = _fit_edge(image)
    angle_deg = math.degrees(math.atan(slope))
    if not 1.5 <= abs(angle_deg) <= 15.0:
        _reject(f"edge slant {angle_deg:.3f} degrees outside 1.5..15 degree range")

    center_x = slope * ((height - 1) * 0.5) + intercept
    if not (0.15 * width <= center_x <= 0.85 * width):
        _reject("edge fit is too close to an ROI boundary; plateau/core coverage missing")

    y_grid, x_grid = np.mgrid[0:height, 0:width]
    signed_distance = (x_grid - slope * y_grid - intercept) / math.sqrt(1.0 + slope * slope)
    core = np.abs(signed_distance) <= 2.0
    clipped = (image <= 1.0) | (image >= 254.0)
    core_clipped_fraction = float(clipped[core].mean()) if np.any(core) else 1.0
    if core_clipped_fraction > 0.20:
        _reject(f"edge core is clipped ({core_clipped_fraction:.1%})")

    dx = 1.0 / int(oversample)
    radius = min(24.0, 0.25 * min(height, width))
    first_bin = math.floor(-radius / dx)
    last_bin = math.ceil(radius / dx)
    bin_indices = np.rint(signed_distance / dx).astype(np.int64)
    selected = (bin_indices >= first_bin) & (bin_indices <= last_bin)
    counts = np.bincount(bin_indices[selected] - first_bin,
                         minlength=last_bin - first_bin + 1)
    sums = np.bincount(bin_indices[selected] - first_bin,
                       weights=image[selected], minlength=last_bin - first_bin + 1)
    coverage = counts >= 3
    if not bool(np.all(coverage)):
        _reject("ESF core coverage has missing or sparsely occupied bins")
    esf = sums / counts
    distance = (np.arange(first_bin, last_bin + 1, dtype=np.float64) * dx)
    core_occupancy = float(counts[distance.__abs__() <= 4.0].min() /
                           max(1.0, counts[distance.__abs__() <= 4.0].max()))

    # Central finite difference, then a symmetric Hamming window around the edge.
    lsf = np.gradient(esf, dx)
    lsf_position = distance.copy()
    window_mask = np.abs(lsf_position) <= min(20.0, radius - 1.0)
    lsf = lsf[window_mask]
    lsf_position = lsf_position[window_mask]
    if lsf.size < 32 or not np.isfinite(lsf).all():
        _reject("insufficient finite LSF samples")
    window = np.hamming(lsf.size)
    windowed_lsf = lsf * window
    nfft = max(4096, 8 * int(2 ** math.ceil(math.log2(windowed_lsf.size))))
    spectrum = np.abs(np.fft.rfft(windowed_lsf, n=nfft))
    dc = float(spectrum[0])
    if not math.isfinite(dc) or dc <= 1e-9:
        _reject("LSF has no usable DC component")
    frequencies = np.fft.rfftfreq(nfft, d=dx)
    # Correct the central-difference derivative response; bound near Nyquist.
    argument = 2.0 * math.pi * frequencies * dx
    denominator = np.sin(argument)
    correction = np.ones_like(frequencies)
    nonzero = np.abs(argument) > 1e-12
    stable = nonzero & (np.abs(denominator) > 1e-9)
    correction[stable] = argument[stable] / denominator[stable]
    correction = np.clip(correction, 1.0, 2.0)
    sfr_all = spectrum / dc * correction
    nyquist = frequencies <= 0.5 + 1e-12
    frequencies = frequencies[nyquist]
    sfr = sfr_all[nyquist]
    if frequencies.size < 3:
        _reject("insufficient frequency samples")
    sfr[0] = 1.0

    mtf50 = _first_crossing(frequencies, sfr, 0.5)
    mtf10 = _first_crossing(frequencies, sfr, 0.1)
    qc = {
        "passed": True,
        "engineeringThresholdsOnly": True,
        "edgeAngleDegrees": [1.5, 15.0],
        "maximumFitResidualPx": 0.45,
        "minimumContrastGrayLevels": 20.0,
        "coreClippedFraction": core_clipped_fraction,
        "minimumCoreBinOccupancy": int(counts[distance.__abs__() <= 4.0].min()),
    }
    return {
        "measurement_scope": SCOPE,
        "oecf_calibrated": False,
        "input_resampled": False,
        "orientation": orientation,
        "angle_deg": angle_deg,
        "fit_residual": fit_info["fit_residual_px"],
        "fit_info": fit_info,
        "contrast": contrast,
        "polarity": polarity,
        "core_occupancy": core_occupancy,
        "qc": qc,
        "freq_cpp": frequencies.tolist(),
        "sfr": sfr.tolist(),
        "mtf50_cpp": mtf50,
        "mtf50_censored": mtf50 is None,
        "mtf10_cpp": mtf10,
        "mtf10_censored": mtf10 is None,
        "esf_distance_px": distance.tolist(),
        "esf": esf.tolist(),
        "lsf_distance_px": lsf_position.tolist(),
        "lsf_windowed": windowed_lsf.tolist(),
    }


def _cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True, help="Input image (e.g. HDMI capture PNG)")
    parser.add_argument("--rois", type=Path, required=True,
                        help="JSON list: [{name,x,y,width,height,orientation}, ...]")
    parser.add_argument("--output", type=Path, required=True, help="New output directory (must not exist)")
    parser.add_argument("--oversample", type=int, default=4)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        parser.error(f"output directory already exists: {out}")
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        parser.error(f"unable to read image: {args.image}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float64)
    rois = json.loads(args.rois.read_text(encoding="utf-8"))
    if not isinstance(rois, list) or not rois:
        parser.error("--rois must be a nonempty JSON list")
    out.mkdir(parents=True, exist_ok=False)
    preview = image.copy()
    rows: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    for index, spec in enumerate(rois):
        name = str(spec.get("name", f"roi_{index + 1}"))
        x, y = int(spec["x"]), int(spec["y"])
        width, height = int(spec["width"]), int(spec["height"])
        orientation = str(spec.get("orientation", "vertical"))
        crop = gray[y:y + height, x:x + width]
        cv2.rectangle(preview, (x, y), (x + width - 1, y + height - 1), (0, 255, 0), 2)
        cv2.putText(preview, name[:32], (x, max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 1, cv2.LINE_AA)
        try:
            result = analyze_edge(crop, orientation=orientation, oversample=args.oversample)
            result.update({"name": name, "x": x, "y": y, "width": width, "height": height})
            results.append(result)
            rows.append({"roi": name, "freq_cpp": result["freq_cpp"], "sfr": result["sfr"]})
        except (ValueError, KeyError, TypeError) as exc:
            results.append({"name": name, "x": x, "y": y, "width": width, "height": height,
                            "error": str(exc), "qc": {"passed": False}})
    cv2.imwrite(str(out / "roi_preview.png"), preview)
    with (out / "sfr_curves.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["roi", "freq_cpp", "sfr"])
        for row in rows:
            writer.writerows((row["roi"], frequency, value)
                             for frequency, value in zip(row["freq_cpp"], row["sfr"]))
    summary = {"measurement_scope": SCOPE, "oecf_calibrated": False,
               "inputImage": str(args.image.resolve()), "roiDefinition": str(args.rois.resolve()),
               "inputImageEncodingNote": "Output-domain decoded image; encoding/capture pipeline can affect SFR.",
               "isoCertificationClaim": False, "results": results}
    (out / "results.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                                      encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")
    for result in results:
        if result.get("qc", {}).get("passed"):
            ax.plot(result["freq_cpp"], result["sfr"], label=result["name"])
    ax.axhline(0.5, color="gray", linewidth=0.8, linestyle="--")
    ax.set(xlim=(0, 0.5), ylim=(0, 1.2), xlabel="Spatial frequency (cycles/pixel)",
           ylabel="Normalized SFR", title="Output-domain engineering eSFR (not ISO certified)")
    ax.grid(True, alpha=0.25)
    if rows:
        ax.legend()
    fig.savefig(out / "sfr_curves.png", dpi=140)
    plt.close(fig)
    return 0 if rows else 2


if __name__ == "__main__":
    raise SystemExit(_cli())
