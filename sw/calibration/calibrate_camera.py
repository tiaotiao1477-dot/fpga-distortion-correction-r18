"""Calibrate a pinhole camera from checkerboard images.

The tool deliberately requires real input images.  It never creates a default
or synthetic camera profile when calibration data is missing.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np


SUPPORTED_SUFFIXES = frozenset({".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"})


class CalibrationError(RuntimeError):
    """Raised when calibration cannot produce a trustworthy profile."""


def discover_images(directory: Path | str) -> list[Path]:
    """Return supported image files in deterministic name order."""

    root = Path(directory)
    if not root.is_dir():
        raise CalibrationError(f"input directory does not exist: {root}")
    return sorted(
        (path for path in root.iterdir() if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES),
        key=lambda path: path.name.lower(),
    )


def split_indices(count: int) -> tuple[list[int], list[int]]:
    """Split usable views into a deterministic 80/20 train/validation split."""

    if count < 2:
        return list(range(count)), []
    train_count = max(2, int(count * 0.8))
    train_count = min(count - 1, train_count)
    return list(range(train_count)), list(range(train_count, count))


def detect_checkerboard(image: np.ndarray, pattern_size: tuple[int, int]) -> np.ndarray | None:
    """Detect and refine checkerboard inner corners, or return ``None``."""

    if image is None or image.size == 0:
        return None
    if len(image.shape) == 2:
        gray = image
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # The SB detector is more tolerant of perspective and uneven lighting.
    find_sb = getattr(cv2, "findChessboardCornersSB", None)
    if find_sb is not None:
        flags = int(getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0)) | int(
            getattr(cv2, "CALIB_CB_ACCURACY", 0)
        )
        found, corners = find_sb(gray, pattern_size, flags=flags)
        if found:
            return np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return np.asarray(refined, dtype=np.float32).reshape(-1, 1, 2)


def make_profile(
    *,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    image_size: tuple[int, int],
    board_size: tuple[int, int],
    square_size: float,
    train_count: int,
    validation_count: int,
    train_rms: float,
    validation_mean: float,
    validation_max: float,
    source_images: Sequence[str],
) -> dict:
    """Build the JSON-serializable camera profile used by later stages."""

    matrix = np.asarray(camera_matrix, dtype=np.float64)
    coeffs = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if matrix.shape != (3, 3):
        raise CalibrationError("camera matrix must be 3x3")
    if coeffs.size < 4:
        raise CalibrationError("calibration returned fewer than four distortion coefficients")
    if np.any(coeffs[4:] != 0.0):
        raise CalibrationError("unsupported distortion terms must be zero for the FPGA second-order model")

    width, height = (int(image_size[0]), int(image_size[1]))
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    k1, k2, p1, p2 = (float(value) for value in coeffs[:4])
    return {
        "schema_version": 1,
        "model": "pinhole_brown_conrady_second_order",
        "resolution": {"width": width, "height": height},
        "crop": {"x": 0, "y": 0, "width": width, "height": height},
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "distortion": {"k1": k1, "k2": k2, "p1": p1, "p2": p2},
        # Flat aliases keep the profile convenient for the UART/config layer.
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "k1": k1,
        "k2": k2,
        "p1": p1,
        "p2": p2,
        "calibration": {
            "fixed_distortion_terms": {"k3": 0.0, "higher_terms": 0.0},
            "board": {
                "inner_corners": {"columns": int(board_size[0]), "rows": int(board_size[1])},
                "square_size": float(square_size),
            },
            "train_count": int(train_count),
            "validation_count": int(validation_count),
            "train_rms_px": float(train_rms),
            "validation_mean_error_px": float(validation_mean),
            "validation_max_error_px": float(validation_max),
            "source_images": [str(name) for name in source_images],
        },
    }


def _object_points(board_size: tuple[int, int], square_size: float) -> np.ndarray:
    columns, rows = board_size
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points *= float(square_size)
    return points


def _read_image(path: Path) -> np.ndarray | None:
    """Read an image through bytes so Windows paths with non-ASCII names work."""

    try:
        encoded = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if encoded.size == 0:
        return None
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def _validation_errors(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    indices: Iterable[int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> list[float]:
    errors: list[float] = []
    for index in indices:
        ok, rvec, tvec = cv2.solvePnP(
            object_points[index], image_points[index], camera_matrix, distortion
        )
        if not ok:
            raise CalibrationError(f"solvePnP failed for validation view {index}")
        projected, _ = cv2.projectPoints(
            object_points[index], rvec, tvec, camera_matrix, distortion
        )
        delta = projected.reshape(-1, 2) - image_points[index].reshape(-1, 2)
        errors.append(float(np.linalg.norm(delta, axis=1).mean()))
    return errors


def calibrate_directory(
    directory: Path | str,
    *,
    pattern_size: tuple[int, int] = (9, 6),
    square_size: float = 1.0,
    min_views: int = 8,
) -> tuple[dict, list[str]]:
    """Calibrate from a directory and return ``(profile, skipped_images)``."""

    if pattern_size[0] <= 0 or pattern_size[1] <= 0:
        raise CalibrationError("checkerboard dimensions must be positive")
    if square_size <= 0:
        raise CalibrationError("square size must be positive")
    if min_views < 2:
        raise CalibrationError("minimum usable views must be at least 2")

    image_paths = discover_images(directory)
    if not image_paths:
        raise CalibrationError("no supported calibration images")

    object_template = _object_points(pattern_size, square_size)
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    source_names: list[str] = []
    skipped: list[str] = []
    image_size: tuple[int, int] | None = None

    for image_path in image_paths:
        image = _read_image(image_path)
        if image is None:
            skipped.append(f"{image_path.name}: unreadable")
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            skipped.append(f"{image_path.name}: resolution differs from {image_size[0]}x{image_size[1]}")
            continue
        corners = detect_checkerboard(image, pattern_size)
        if corners is None:
            skipped.append(f"{image_path.name}: checkerboard not found")
            continue
        object_points.append(object_template.copy())
        image_points.append(corners)
        source_names.append(image_path.name)

    if len(image_points) < min_views:
        detail = f"; skipped: {', '.join(skipped)}" if skipped else ""
        raise CalibrationError(
            f"need at least {min_views} usable calibration views, got {len(image_points)}{detail}"
        )

    train_indices, validation_indices = split_indices(len(image_points))
    assert image_size is not None
    try:
        train_objects = [object_points[index] for index in train_indices]
        train_images = [image_points[index] for index in train_indices]
        train_rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
            train_objects, train_images, image_size, None, None,
            flags=cv2.CALIB_FIX_K3,
        )
        validation_errors = _validation_errors(
            object_points,
            image_points,
            validation_indices,
            camera_matrix,
            distortion,
        )
    except cv2.error as exc:
        raise CalibrationError(f"OpenCV calibration failed: {exc}") from exc

    validation_mean = float(np.mean(validation_errors)) if validation_errors else 0.0
    validation_max = float(np.max(validation_errors)) if validation_errors else 0.0
    profile = make_profile(
        camera_matrix=camera_matrix,
        distortion=distortion,
        image_size=image_size,
        board_size=pattern_size,
        square_size=square_size,
        train_count=len(train_indices),
        validation_count=len(validation_indices),
        train_rms=float(train_rms),
        validation_mean=validation_mean,
        validation_max=validation_max,
        source_images=source_names,
    )
    return profile, skipped


def write_profile(profile: dict, output_path: Path | str) -> None:
    """Write a profile as stable, human-readable UTF-8 JSON."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(profile, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True, type=Path, help="directory containing checkerboard images")
    parser.add_argument("--output", required=True, type=Path, help="camera_profile.json output path")
    parser.add_argument("--columns", type=int, default=9, help="checkerboard inner-corner columns")
    parser.add_argument("--rows", type=int, default=6, help="checkerboard inner-corner rows")
    parser.add_argument("--square-size", type=float, default=1.0, help="physical square size in user units")
    parser.add_argument("--min-views", type=int, default=8, help="minimum usable images")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        profile, skipped = calibrate_directory(
            args.input_dir,
            pattern_size=(args.columns, args.rows),
            square_size=args.square_size,
            min_views=args.min_views,
        )
        write_profile(profile, args.output)
    except CalibrationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"calibration succeeded: {profile['calibration']['train_count']} train / "
          f"{profile['calibration']['validation_count']} validation views")
    print(f"profile: {args.output}")
    if skipped:
        print("skipped images:")
        for reason in skipped:
            print(f"  - {reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
