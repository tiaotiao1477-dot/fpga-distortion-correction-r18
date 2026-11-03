"""Capture a short host-side visual sample from the HDMI capture device.

This records five fresh frames for PC-side roll estimation. It does not prove
that the source produces distinct frames or establish end-to-end FPGA timing.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
from typing import Any

import cv2
import numpy as np


@dataclass
class CaptureBatch:
    frames: list[np.ndarray]
    image_paths: list[Path]
    record_path: Path
    record: dict


_FRAME_INDICES = (15, 23, 30, 38, 45)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_record(path: Path, record: dict) -> None:
    path.write_text(json.dumps(record, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def capture_batch(output: Path, device: str = "USB Video") -> CaptureBatch:
    """Capture and decode five independent 1920x1080 BGR frames.

    ``output`` must name a new directory on an ASCII filesystem path. On a
    capture/decode failure, the record and FFmpeg log are preserved and the
    original exception is raised.
    """
    output = Path(output)
    if output.exists():
        raise FileExistsError(f"Capture output directory already exists: {output}")
    if not str(output).isascii():
        raise ValueError(f"Capture output path must be ASCII: {output}")

    output.mkdir(parents=True, exist_ok=False)
    video = output / "capture.mkv"
    log_path = output / "ffmpeg.log"
    record_path = output / "record.json"
    command = [
        "ffmpeg", "-hide_banner", "-nostdin", "-f", "dshow",
        "-video_size", "1920x1080", "-framerate", "30", "-vcodec", "mjpeg",
        "-i", f"video={device}", "-map", "0:v:0", "-an", "-c:v", "copy",
        "-t", "2", str(video),
    ]
    record: dict[str, Any] = {
        "scope": "Host visual sample only; not proof of source-distinct frames or end-to-end FPGA timing",
        "device": device,
        "inputMode": {"width": 1920, "height": 1080, "frameRate": 30, "codec": "mjpeg"},
        "selectedFrameIndices": list(_FRAME_INDICES),
        "startedUtc": _utc_now(),
        "command": command,
        "videoPath": str(video),
        "logPath": str(log_path),
        "passed": False,
    }
    images: list[Path] = []
    frames: list[np.ndarray] = []
    try:
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0) if os.name == "nt" else 0
        with log_path.open("xb") as log:
            result = subprocess.run(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=30,
                check=False,
                creationflags=creationflags,
            )
        if result.returncode != 0:
            details = f"FFmpeg capture failed (exit {result.returncode}); see {log_path}"
            if not video.exists() or video.stat().st_size == 0:
                details += ". The camera may be missing or busy; close any competing camera app and retry."
            raise RuntimeError(details)

        cap = cv2.VideoCapture(str(video))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open captured video for decoding: {video}")
        try:
            decoded: dict[int, np.ndarray] = {}
            index = 0
            while index <= _FRAME_INDICES[-1]:
                ok, frame = cap.read()
                if not ok:
                    break
                if index in _FRAME_INDICES:
                    decoded[index] = frame
                index += 1
            if index < 46 or len(decoded) != len(_FRAME_INDICES):
                raise RuntimeError(f"Capture decode yielded {index} frames; at least 46 are required")
            for frame_index in _FRAME_INDICES:
                frame = decoded[frame_index]
                if frame.dtype != np.uint8 or frame.shape != (1080, 1920, 3):
                    raise RuntimeError(
                        f"Frame {frame_index} has invalid format {frame.shape}/{frame.dtype}; "
                        "expected BGR uint8 1920x1080"
                    )
                independent = frame.copy()
                image_path = output / f"frame_{frame_index:03d}.png"
                if not cv2.imwrite(str(image_path), independent):
                    raise RuntimeError(f"Could not save selected image: {image_path}")
                frames.append(independent)
                images.append(image_path)
        finally:
            cap.release()

        record.update(
            passed=True,
            actualChosenIndices=list(_FRAME_INDICES),
            imagePaths=[str(path) for path in images],
            videoSha256=_sha256(video),
            imageSha256={path.name: _sha256(path) for path in images},
        )
        return CaptureBatch(frames, images, record_path, record)
    except Exception as exc:
        record["error"] = f"{type(exc).__name__}: {exc}"
        if isinstance(exc, subprocess.TimeoutExpired):
            record["error"] += "; FFmpeg exceeded the 30-second timeout"
        raise
    finally:
        record["endedUtc"] = _utc_now()
        if log_path.exists():
            record["logSha256"] = _sha256(log_path)
        if video.exists():
            record.setdefault("videoSha256", _sha256(video))
        if images:
            record.setdefault("imageSha256", {path.name: _sha256(path) for path in images if path.exists()})
        _write_record(record_path, record)
