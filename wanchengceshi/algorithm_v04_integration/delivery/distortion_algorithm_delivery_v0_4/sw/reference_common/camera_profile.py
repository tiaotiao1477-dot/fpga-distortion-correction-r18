"""Validated camera profile for the k1/k2 + p1/p2 pinhole model."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping


@dataclass(frozen=True)
class CameraProfile:
    """Calibration parameters for one fixed source-image resolution.

    Pixel coordinates use the project convention: the center of the top-left
    pixel is (0, 0), x grows rightward, and y grows downward.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0

    def __post_init__(self) -> None:
        if (
            isinstance(self.width, bool)
            or not isinstance(self.width, int)
            or isinstance(self.height, bool)
            or not isinstance(self.height, int)
        ):
            raise ValueError("width and height must be positive integers")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive integers")
        for name in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("fx and fy must be positive")

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "CameraProfile":
        required = ("width", "height", "fx", "fy", "cx", "cy")
        missing = [name for name in required if name not in values]
        if missing:
            raise ValueError(f"camera profile is missing fields: {', '.join(missing)}")
        if values.get("model", "radial_tangential_k1_k2_p1_p2") != (
            "radial_tangential_k1_k2_p1_p2"
        ):
            raise ValueError("unsupported camera distortion model")
        for name in ("width", "height"):
            value = values[name]
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer")
        try:
            return cls(
                width=values["width"],
                height=values["height"],
                fx=float(values["fx"]),
                fy=float(values["fy"]),
                cx=float(values["cx"]),
                cy=float(values["cy"]),
                k1=float(values.get("k1", 0.0)),
                k2=float(values.get("k2", 0.0)),
                p1=float(values.get("p1", 0.0)),
                p2=float(values.get("p2", 0.0)),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"invalid camera profile: {exc}") from exc

    def to_mapping(self) -> dict[str, int | float | str]:
        return {"model": "radial_tangential_k1_k2_p1_p2", **asdict(self)}


def load_camera_profile(path: str | Path) -> CameraProfile:
    """Load a JSON camera profile without inventing calibration values."""

    profile_path = Path(path)
    try:
        payload = json.loads(profile_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load camera profile {profile_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("camera profile root must be a JSON object")
    return CameraProfile.from_mapping(payload)


def save_camera_profile(profile: CameraProfile, path: str | Path) -> None:
    """Write the exact supported model and its calibration parameters."""

    profile_path = Path(path)
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    profile_path.write_text(
        json.dumps(profile.to_mapping(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
