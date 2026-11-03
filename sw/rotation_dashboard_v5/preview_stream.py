"""Threaded FFmpeg preview and fresh-frame capture for the rotation dashboard."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Any
import uuid

import cv2
import numpy as np


WIDTH = 1920
HEIGHT = 1080
CHANNELS = 3
FRAME_BYTES = WIDTH * HEIGHT * CHANNELS
MAX_MJPEG_LINE_BYTES = 4096
MAX_MJPEG_HEADERS = 16
MAX_MJPEG_PACKET_BYTES = 16 * 1024 * 1024
MJPEG_BOUNDARY = b"--fpga-preview"
RING_CAPACITY = 24
SAMPLE_COUNT = 5
SAMPLE_SETTLE_SECONDS = 0.6
SAMPLE_INTERVAL_SECONDS = 0.2
FPS_WINDOW_SECONDS = 2.0
MAX_CAPTURE_SECONDS = 12.0
MAX_READER_JOIN_SECONDS = 2.0
MAX_PROCESS_WAIT_SECONDS = 2.0


@dataclass(frozen=True)
class CaptureBatch:
    """Immutable references to a set of fresh full-HD frames and its record."""

    frames: tuple[np.ndarray, ...]
    image_paths: tuple[Path, ...]
    record_path: Path
    record: dict[str, Any]


@dataclass(frozen=True)
class _Frame:
    sequence: int
    monotonic: float
    generation: int
    image: np.ndarray


class PreviewStream:
    """Own one FFmpeg process, a bounded frame ring, and fresh capture batches."""

    def __init__(self, device: str, log_dir: Path):
        self.device = str(device)
        self.log_dir = Path(log_dir)
        self._condition = threading.Condition(threading.RLock())
        self._capture_lock = threading.Lock()
        self._ring: deque[_Frame] = deque(maxlen=RING_CAPACITY)
        self._proc: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._log_handle: Any = None
        self._log_path: Path | None = None
        self._state = "stopped"
        self._error: str | None = None
        self._generation = 0
        self._sequence = 0
        self._frames_read = 0
        self._compressed_packets_received = 0
        self._decoded_frame_times: deque[float] = deque(maxlen=240)
        self._started_monotonic: float | None = None
        self._last_frame_monotonic: float | None = None
        self._last_selection_monotonic: float | None = None
        self._last_capture_dir: Path | None = None
        self._stop_lock = threading.Lock()

    @staticmethod
    def _argv(device: str) -> list[str]:
        return [
            "ffmpeg",
            "-hide_banner",
            "-nostdin",
            "-f", "dshow",
            "-video_size", f"{WIDTH}x{HEIGHT}",
            "-framerate", "30",
            "-vcodec", "mjpeg",
            "-i", f"video={device}",
            "-map", "0:v:0",
            "-an",
            "-c:v", "copy",
            "-f", "mpjpeg",
            "-boundary_tag", "fpga-preview",
            "-flush_packets", "1",
            "pipe:1",
        ]

    @property
    def argv(self) -> tuple[str, ...]:
        """The exact FFmpeg argument vector used for the active generation."""
        return tuple(self._argv(self.device))

    def start(self) -> None:
        """Start FFmpeg and its reader thread without waiting for a frame."""
        with self._condition:
            if self._proc is not None and self._proc.poll() is None:
                return
            if self._reader is not None and self._reader.is_alive():
                self._error = "Previous FFmpeg reader has not exited"
                self._state = "error"
                return
            if self._log_handle is not None:
                try:
                    self._log_handle.close()
                except OSError:
                    pass
                self._log_handle = None
            self._proc = None
            self._reader = None

            self.log_dir.mkdir(parents=True, exist_ok=True)
            self._generation += 1
            self._sequence = 0
            self._compressed_packets_received = 0
            self._decoded_frame_times.clear()
            self._ring.clear()
            self._error = None
            self._started_monotonic = time.monotonic()
            self._last_frame_monotonic = None
            self._last_selection_monotonic = None
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            self._log_path = self.log_dir / f"ffmpeg_{stamp}_{self._generation}.log"
            self._state = "starting"
            try:
                self._log_handle = self._log_path.open("ab", buffering=0)
                flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
                self._proc = subprocess.Popen(
                    self._argv(self.device),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=self._log_handle,
                    creationflags=flags,
                    bufsize=0,
                )
            except Exception as exc:
                self._state = "error"
                self._error = f"Unable to start FFmpeg: {type(exc).__name__}: {exc}"
                if self._log_handle is not None:
                    self._log_handle.close()
                    self._log_handle = None
                self._condition.notify_all()
                return

            self._state = "starting"
            generation = self._generation
            reader = threading.Thread(
                target=self._read_stdout,
                args=(self._proc, generation),
                name=f"PreviewStreamReader-{generation}",
                daemon=True,
            )
            self._reader = reader
            reader.start()
            self._condition.notify_all()

    @staticmethod
    def _read_exact(pipe: Any, size: int) -> bytes | None:
        parts: list[bytes] = []
        remaining = size
        while remaining:
            chunk = pipe.read(remaining)
            if not chunk:
                if remaining != size:
                    raise EOFError(f"Truncated MPJPEG payload: expected {size} bytes")
                return None
            parts.append(chunk)
            remaining -= len(chunk)
        return b"".join(parts)

    @staticmethod
    def _read_bounded_line(pipe: Any) -> bytes | None:
        line = pipe.readline(MAX_MJPEG_LINE_BYTES + 1)
        if not line:
            return None
        if len(line) > MAX_MJPEG_LINE_BYTES or not line.endswith(b"\n"):
            raise ValueError("MPJPEG header line exceeds 4096 bytes or is unterminated")
        return line

    @classmethod
    def _read_mjpeg_packet(cls, pipe: Any) -> bytes | None:
        boundary_line = cls._read_bounded_line(pipe)
        if boundary_line is None:
            return None
        boundary = boundary_line.rstrip(b"\r\n")
        if boundary == MJPEG_BOUNDARY + b"--":
            return None
        if boundary != MJPEG_BOUNDARY:
            raise ValueError(f"Unexpected MPJPEG boundary: {boundary[:64]!r}")

        content_length: int | None = None
        content_type: str | None = None
        header_count = 0
        while True:
            line = cls._read_bounded_line(pipe)
            if line is None:
                raise EOFError("Unexpected EOF in MPJPEG headers")
            if line in (b"\r\n", b"\n"):
                break
            header_count += 1
            if header_count > MAX_MJPEG_HEADERS:
                raise ValueError("MPJPEG packet exceeds 16 headers")
            try:
                key, value = line.decode("ascii").strip().split(":", 1)
            except (UnicodeDecodeError, ValueError) as exc:
                raise ValueError("Malformed MPJPEG header") from exc
            key = key.strip().lower()
            value = value.strip()
            if key == "content-length":
                try:
                    content_length = int(value, 10)
                except ValueError as exc:
                    raise ValueError("Invalid MPJPEG Content-length") from exc
            elif key == "content-type":
                content_type = value.lower()

        if content_type != "image/jpeg":
            raise ValueError(f"Unexpected MPJPEG Content-type: {content_type!r}")
        if content_length is None or not (0 < content_length <= MAX_MJPEG_PACKET_BYTES):
            raise ValueError(f"MPJPEG Content-length must be within 1..{MAX_MJPEG_PACKET_BYTES}")
        payload = cls._read_exact(pipe, content_length)
        if payload is None:
            raise EOFError("MPJPEG stream ended before packet payload")
        trailer = cls._read_exact(pipe, 2)
        if trailer != b"\r\n":
            raise ValueError("MPJPEG packet is missing its CRLF trailer")
        return payload

    def _read_stdout(self, proc: subprocess.Popen[bytes], generation: int) -> None:
        pipe = proc.stdout
        if pipe is None:
            self._reader_finished(generation, "FFmpeg stdout pipe is unavailable")
            return
        error: str | None = None
        try:
            while True:
                packet = self._read_mjpeg_packet(pipe)
                if packet is None:
                    break
                with self._condition:
                    if generation != self._generation or self._state in ("stopping", "stopped"):
                        break
                    self._compressed_packets_received += 1
                image = cv2.imdecode(np.frombuffer(packet, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None or image.dtype != np.uint8 or image.shape != (HEIGHT, WIDTH, CHANNELS):
                    shape = None if image is None else image.shape
                    dtype = None if image is None else image.dtype
                    raise ValueError(f"Decoded preview frame has invalid format {shape}/{dtype}; expected uint8 1920x1080 BGR")
                image.setflags(write=False)
                now = time.monotonic()
                with self._condition:
                    if generation != self._generation or self._state in ("stopping", "stopped"):
                        break
                    self._sequence += 1
                    self._frames_read += 1
                    self._last_frame_monotonic = now
                    self._decoded_frame_times.append(now)
                    while self._decoded_frame_times and now - self._decoded_frame_times[0] > FPS_WINDOW_SECONDS:
                        self._decoded_frame_times.popleft()
                    self._ring.append(_Frame(self._sequence, now, generation, image))
                    self._state = "running"
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                stopping = self._state in ("stopping", "stopped")
            if not stopping:
                error = f"FFmpeg frame reader failed: {type(exc).__name__}: {exc}"
        finally:
            try:
                pipe.close()
            except Exception:
                pass
            self._reader_finished(generation, error)

    def _reader_finished(self, generation: int, reader_error: str | None) -> None:
        with self._condition:
            if generation == self._generation:
                proc = self._proc
                return_code = proc.poll() if proc is not None else None
                stopping = self._state in ("stopping", "stopped")
                if reader_error:
                    self._error = reader_error
                    self._state = "error"
                elif not stopping and return_code is not None:
                    self._error = f"FFmpeg exited with code {return_code}; see {self._log_path}"
                    self._state = "error"
                elif not stopping:
                    self._error = "FFmpeg stdout ended unexpectedly"
                    self._state = "error"
                self._condition.notify_all()

    def stop(self) -> None:
        """Terminate and reap only this stream's FFmpeg; repeated calls are safe."""
        with self._stop_lock:
            with self._condition:
                proc = self._proc
                reader = self._reader
                if proc is None and reader is None:
                    if self._state not in ("error",):
                        self._state = "stopped"
                    return
                if proc is not None and proc.poll() is None:
                    self._state = "stopping"
                self._condition.notify_all()

            if proc is not None and proc.poll() is None:
                try:
                    proc.terminate()
                except OSError:
                    pass
                try:
                    proc.wait(timeout=MAX_PROCESS_WAIT_SECONDS)
                except subprocess.TimeoutExpired:
                    try:
                        proc.kill()
                    except OSError:
                        pass
                    try:
                        proc.wait(timeout=MAX_PROCESS_WAIT_SECONDS)
                    except subprocess.TimeoutExpired:
                        with self._condition:
                            self._error = "Timed out reaping owned FFmpeg process"
                            self._state = "error"
                            self._condition.notify_all()

            if reader is not None and reader is not threading.current_thread():
                reader.join(timeout=MAX_READER_JOIN_SECONDS)
            with self._condition:
                process_alive = proc is not None and proc.poll() is None
                reader_alive = reader is not None and reader.is_alive()
                if process_alive:
                    self._error = "Owned FFmpeg process is still running after bounded reap"
                    self._state = "error"
                elif reader_alive:
                    self._error = "Timed out joining FFmpeg frame reader"
                    self._state = "error"
                elif self._state != "error":
                    self._state = "stopped"
                self._proc = proc if process_alive else None
                self._reader = reader if reader_alive else None
                if self._log_handle is not None and not process_alive:
                    try:
                        self._log_handle.close()
                    except OSError:
                        pass
                    self._log_handle = None
                self._condition.notify_all()

    def latest(self) -> tuple[int, float, np.ndarray] | None:
        """Return newest frame only while it is no more than one second old."""
        identified = self.latest_with_identity()
        if identified is None:
            return None
        _generation, sequence, stamp, image = identified
        return sequence, stamp, image

    def latest_with_identity(self) -> tuple[int, int, float, np.ndarray] | None:
        """Return the newest fresh frame and its generation from one locked snapshot."""
        now = time.monotonic()
        with self._condition:
            if not self._ring:
                return None
            frame = self._ring[-1]
            if frame.generation != self._generation or now - frame.monotonic > 1.0:
                return None
            if self._state not in ("starting", "running"):
                return None
            return frame.generation, frame.sequence, frame.monotonic, frame.image

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._condition:
            alive = self._proc is not None and self._proc.poll() is None
            fresh = self._last_frame_monotonic is not None and now - self._last_frame_monotonic <= 1.0
            state = self._state
            if state == "running" and not fresh:
                state = "stale"
            elif state == "starting" and self._started_monotonic is not None and now - self._started_monotonic > 1.0:
                state = "stale"
            fps = self._recent_fps_locked(now)
            return {
                "state": state,
                "error": self._error,
                "fps": fps,
                "frames": self._frames_read,
                "compressed_packets_received": self._compressed_packets_received,
                "mode": "dshow MJPEG 1920x1080@30 -> MJPEG copy -> host BGR decode every received packet (target 30fps)",
                "log_path": str(self._log_path) if self._log_path else None,
                "generation": self._generation,
                "process_alive": alive,
                "reader_alive": self._reader is not None and self._reader.is_alive(),
            }

    def _recent_fps_locked(self, now: float) -> float:
        """Return the decoded-frame rate over the recent timestamp window."""
        with self._condition:
            if self._last_frame_monotonic is None or now - self._last_frame_monotonic > 1.0:
                return 0.0
            recent = [stamp for stamp in self._decoded_frame_times if now - stamp <= FPS_WINDOW_SECONDS]
            if len(recent) < 2:
                return 0.0
            elapsed = recent[-1] - recent[0]
            return (len(recent) - 1) / elapsed if elapsed > 0 else 0.0

    def collect(self, output: Path, timeout: float = MAX_CAPTURE_SECONDS) -> CaptureBatch:
        """Save five fresh, spaced full-HD frames from one stream generation."""
        if not self._capture_lock.acquire(blocking=False):
            raise RuntimeError("A preview capture is already in progress")
        target = Path(output)
        owns_output = False
        try:
            if not str(target).isascii():
                raise ValueError(f"Capture output path must be ASCII: {target}")
            target.mkdir(parents=True, exist_ok=False)
            owns_output = True
            try:
                return self._collect_locked(target, timeout)
            except Exception as exc:
                if owns_output and not (target / "record.json").exists():
                    try:
                        self._write_failed_capture_record(target, exc)
                    except OSError:
                        pass
                raise
        finally:
            self._capture_lock.release()

    def _collect_locked(self, output: Path, timeout: float) -> CaptureBatch:
        timeout = min(MAX_CAPTURE_SECONDS, max(0.0, float(timeout)))
        started_utc = datetime.now(timezone.utc).isoformat()
        deadline = time.monotonic() + timeout
        with self._condition:
            barrier_generation = self._generation
            barrier_sequence = self._sequence
            barrier_monotonic = time.monotonic()
            settle_until = barrier_monotonic + SAMPLE_SETTLE_SECONDS
            barrier_log_path = self._log_path

        selections: list[_Frame] = []
        last_selected_at: float | None = None
        while len(selections) < SAMPLE_COUNT:
            with self._condition:
                now = time.monotonic()
                if now >= deadline:
                    break
                if self._generation != barrier_generation:
                    raise RuntimeError("Preview stream restarted during capture; frames cannot cross generations")
                if self._state in ("error", "stopped", "stopping"):
                    raise RuntimeError(self._error or f"Preview stream is {self._state}")

                eligible = [
                    item for item in self._ring
                    if item.generation == barrier_generation
                    and item.sequence > barrier_sequence
                    and item.monotonic >= settle_until
                    and (last_selected_at is None or item.monotonic - last_selected_at >= SAMPLE_INTERVAL_SECONDS)
                ]
                if eligible:
                    selected = eligible[0]
                    selections.append(selected)
                    last_selected_at = selected.monotonic
                    self._last_selection_monotonic = selected.monotonic
                    continue

                remaining = deadline - now
                self._condition.wait(timeout=min(remaining, 0.1))

        if len(selections) != SAMPLE_COUNT:
            raise TimeoutError(f"Only {len(selections)} fresh frames collected within {timeout:.1f}s")

        ended_monotonic = time.monotonic()
        images: list[np.ndarray] = []
        image_paths: list[Path] = []
        image_sha256: dict[str, str] = {}
        image_info: list[dict[str, Any]] = []
        for index, selected in enumerate(selections, start=1):
            image = selected.image.copy()
            path = output / f"frame_{index:02d}_seq_{selected.sequence:08d}.png"
            ok, encoded = cv2.imencode(".png", image)
            if not ok:
                raise RuntimeError(f"Unable to encode full-HD capture frame {index}")
            path.write_bytes(encoded.tobytes())
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            images.append(image)
            image_paths.append(path)
            image_sha256[path.name] = digest
            image_info.append({
                "path": str(path),
                "sequence": selected.sequence,
                "monotonic": selected.monotonic,
                "sha256": digest,
            })

        source_log = barrier_log_path
        snapshot_path: Path | None = None
        snapshot_hash: str | None = None
        if source_log is not None and source_log.exists():
            snapshot_path = output / "ffmpeg.log.snapshot"
            shutil.copyfile(source_log, snapshot_path)
            snapshot_hash = self._sha256_file(snapshot_path)

        record = {
            "passed": True,
            "generation": barrier_generation,
            "barrierSequence": barrier_sequence,
            "barrierMonotonic": barrier_monotonic,
            "selectionStartMonotonic": selections[0].monotonic,
            "selectionEndMonotonic": selections[-1].monotonic,
            "endedMonotonic": ended_monotonic,
            "startedUtc": started_utc,
            "endedUtc": datetime.now(timezone.utc).isoformat(),
            "mode": self._argv(self.device),
            "device": self.device,
            "width": WIDTH,
            "height": HEIGHT,
            "pixelFormat": "bgr24",
            "previewTargetFps": 30,
            "previewFps": self._recent_fps_locked(time.monotonic()),
            "measuredPreviewFps": self._recent_fps_locked(time.monotonic()),
            "images": image_info,
            "imageSha256": image_sha256,
            "ffmpegLogPath": str(source_log) if source_log else None,
            "ffmpegLogSnapshotPath": str(snapshot_path) if snapshot_path else None,
            "ffmpegLogSnapshotSha256": snapshot_hash,
        }
        with self._condition:
            if self._generation != barrier_generation:
                raise RuntimeError("Preview stream restarted before capture completion")
            if self._state != "running":
                raise RuntimeError(f"Preview stream stopped before capture completion (state={self._state})")
            record_path = output / "record.json"
            record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
            self._last_capture_dir = output
        return CaptureBatch(tuple(images), tuple(image_paths), record_path, record)

    @staticmethod
    def _sha256_file(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        return digest.hexdigest()

    def _write_failed_capture_record(self, output: Path, exc: Exception) -> None:
        """Persist failure details and a fixed copy of this generation's log."""
        record_path = output / "record.json"
        if record_path.exists():
            return
        source_log = self._log_path
        snapshot_path: Path | None = None
        snapshot_hash: str | None = None
        if source_log is not None and source_log.exists():
            snapshot_path = output / "ffmpeg.log.snapshot"
            try:
                shutil.copyfile(source_log, snapshot_path)
                snapshot_hash = self._sha256_file(snapshot_path)
            except OSError:
                snapshot_path = None
        record = {
            "passed": False,
            "generation": self._generation,
            "device": self.device,
            "mode": self._argv(self.device),
            "startedUtc": datetime.now(timezone.utc).isoformat(),
            "endedUtc": datetime.now(timezone.utc).isoformat(),
            "error": f"{type(exc).__name__}: {exc}",
            "ffmpegLogPath": str(source_log) if source_log else None,
            "ffmpegLogSnapshotPath": str(snapshot_path) if snapshot_path else None,
            "ffmpegLogSnapshotSha256": snapshot_hash,
        }
        record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
