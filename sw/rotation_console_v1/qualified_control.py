"""Qualified host-side adapter for the frozen native1080 R8 UART pipeline."""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

sys.dont_write_bytecode = True

BIT_SHA256 = "e8b8d37b8e046069f724119e22006441cd28c8e81a134f1fa99a2c51ba883ffb"
PROFILE_SHA256 = "bb929129b3f6713a7b75225b554be7dea7174b704ec32dd1cde869dc69164a14"
HELPER_SHA256 = "64a6941b0ffaee1068a92fef0b133f9712bfab57db4fadfe7e999c9d5f46fa45"
PRESETS = {0.0: "config_measured_angle0", 13.37: "config_real_13p37_clamp", 45.0: "config_real_45_clamp"}
_TIMEOUT = 360


class ExternalStateChanged(RuntimeError):
    """Fresh FPGA diagnostics no longer match this controller's last ACK."""


def _sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat()


def _angle_distance(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _binding_check(board: Path, profile: Path) -> tuple[dict, Path, Path]:
    acceptance_path = board / "reports" / "physical_acceptance.json"
    acceptance = _load(acceptance_path)
    if acceptance.get("passed") is not True or acceptance.get("candidate") != "algorithm1080_rejoin_r8":
        raise ValueError("Original R8 physical acceptance is not qualified")
    if _sha(Path(acceptance["bitstream"])) != BIT_SHA256 or acceptance.get("bitstreamSha256") != BIT_SHA256:
        raise ValueError("Qualified bitstream binding changed")
    if _sha(profile) != PROFILE_SHA256:
        raise ValueError("Measured native profile binding changed")
    helper = board / "tools" / "algorithm_uart.py"
    if _sha(helper) != HELPER_SHA256:
        raise ValueError("Frozen algorithm_uart.py binding changed")
    # Match the unchanged send CLI: every acceptance source and evidence hash is
    # re-read in full for each send. Never substitute stat metadata here.
    for name, digest in {**acceptance["sourceHashes"], **acceptance["evidenceHashes"]}.items():
        if _sha(Path(name)) != digest:
            raise ValueError(f"Original qualified SHA scope changed: {name}")
    decoder = (board / "tools" / "decode_algorithm_packet.py").resolve()
    accepted_decoder_hash = next((digest for name, digest in {**acceptance["sourceHashes"], **acceptance["evidenceHashes"]}.items()
                                  if Path(name).resolve() == decoder), None)
    if accepted_decoder_hash is None or _sha(decoder) != accepted_decoder_hash:
        raise ValueError("Intended R8 diagnostic decoder is not covered by the original source hash gate")
    return acceptance, acceptance_path, helper


def _next_sequence(state: dict) -> int:
    forbidden = {int(state["receivedSequence"]) & 0xFFFFFFFF,
                 int(state["appliedSequence"]) & 0xFFFFFFFF}
    n = int.from_bytes(os.urandom(4), "little")
    while n in forbidden:
        n = (n + 1) & 0xFFFFFFFF
    return n


class QualifiedController:
    def __init__(self, board_root: Path, profile_path: Path, session_path: Path,
                 port: str, work_dir: Path):
        self.board_root = Path(board_root).resolve()
        self.profile_path = Path(profile_path).resolve()
        self.session_path = Path(session_path).resolve()
        self.port = str(port)
        self.work_dir = Path(work_dir).resolve()
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self.current_angle: float | None = None
        self.last_applied: dict | None = None
        self._acceptance, self._acceptance_path, self._helper = _binding_check(self.board_root, self.profile_path)
        self._session = _load(self.session_path)
        self._guard_session()

    def _guard_session(self) -> None:
        # Import the exact frozen guard by file path, avoiding module-name
        # collisions with other projects' program_session_guard modules.
        import importlib.util
        path = self.board_root / "tools" / "program_session_guard.py"
        if not path.is_file():
            raise FileNotFoundError(path)
        spec = importlib.util.spec_from_file_location("_rotation_r8_session_guard", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.check_session(self._session, self._acceptance)

    def _log_run(self, label: str, argv: list[str], timeout: int = _TIMEOUT) -> subprocess.CompletedProcess:
        folder = self.work_dir / (label + "_" + _dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + uuid.uuid4().hex[:8])
        folder.mkdir(parents=False, exist_ok=False)
        (folder / "command.json").write_text(json.dumps({"argv": argv, "cwd": str(self.board_root), "startedUtc": _utc()}, indent=2), encoding="utf-8")
        try:
            cp = subprocess.run(argv, cwd=self.board_root, capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=timeout,
                                check=False, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
            (folder / "stdout.log").write_text(cp.stdout or "", encoding="utf-8")
            (folder / "stderr.log").write_text(cp.stderr or "", encoding="utf-8")
            (folder / "exit.json").write_text(json.dumps({"returncode": cp.returncode, "endedUtc": _utc()}, indent=2), encoding="utf-8")
            if cp.returncode:
                raise subprocess.CalledProcessError(cp.returncode, argv, cp.stdout, cp.stderr)
            return cp
        except subprocess.TimeoutExpired as exc:
            (folder / "stdout.log").write_text(str(exc.stdout or ""), encoding="utf-8")
            (folder / "stderr.log").write_text(str(exc.stderr or "") + "\nTIMEOUT", encoding="utf-8")
            (folder / "exit.json").write_text(json.dumps({"timeoutSeconds": timeout, "endedUtc": _utc()}, indent=2), encoding="utf-8")
            raise
        except BaseException as exc:
            partial_out = getattr(exc, "stdout", None) or getattr(exc, "output", None) or ""
            partial_err = getattr(exc, "stderr", None) or ""
            (folder / "stdout.log").write_text(str(partial_out), encoding="utf-8")
            (folder / "stderr.log").write_text(str(partial_err) + "\n" + repr(exc), encoding="utf-8")
            (folder / "exit.json").write_text(json.dumps({"exception": repr(exc), "endedUtc": _utc()}, indent=2), encoding="utf-8")
            raise

    def _fresh_state(self) -> dict:
        # Open the one UART for a bounded diagnostic exchange and always close it.
        import serial
        tools_dir = self.board_root / "tools"
        import importlib.util
        # raw1080_uart imports by the top-level module name; bind that name to
        # this board's accepted file during module loading, then restore the
        # process module table so no cross-project module can leak into it.
        decoder_path = tools_dir / "decode_algorithm_packet.py"
        decoder_spec = importlib.util.spec_from_file_location("_rotation_exact_decoder", decoder_path)
        decoder = importlib.util.module_from_spec(decoder_spec)
        decoder_spec.loader.exec_module(decoder)
        old_decoder = sys.modules.get("decode_algorithm_packet")
        sys.modules["decode_algorithm_packet"] = decoder
        try:
            spec = importlib.util.spec_from_file_location("_rotation_raw1080_uart", tools_dir / "raw1080_uart.py")
            raw = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(raw)
        finally:
            if old_decoder is None:
                sys.modules.pop("decode_algorithm_packet", None)
            else:
                sys.modules["decode_algorithm_packet"] = old_decoder
        uart = serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=2)
        uart.port = self.port
        uart.dtr = False
        uart.rts = False
        try:
            uart.open()
            uart.reset_input_buffer()
            uart.write(b"\xa5")
            data = bytearray()
            stamp = time.monotonic()
            offset = -1
            while time.monotonic() - stamp < 4:
                data.extend(uart.read(512))
                offset = data.find(b"\x55\xaa\xd1\xf4")
                if offset >= 0 and len(data) >= offset + 251:
                    break
            if offset < 0 or len(data) < offset + 251:
                raise TimeoutError("No complete fresh native diagnostic packet")
            decoded = raw.decode_raw1080(bytes(data[offset:offset + 251]))
            cfg = decoded.get("algorithmConfig")
            if not isinstance(cfg, dict) or cfg.get("fresh") is not True:
                raise ValueError("Algorithm configuration diagnostics are not fresh")
            # valid/ready are UART configuration-transfer handshake signals,
            # not persistent validity of the active image configuration. Both
            # are normally low after a consumed packet; use ACK identity and
            # version/count observations for application confirmation instead.
            if cfg.get("frameError") is not False:
                raise ValueError("Fresh FPGA diagnostic reports a frame error")
            return decoded
        finally:
            if uart.is_open:
                uart.close()

    def snapshot(self) -> dict:
        state = self._fresh_state()
        return {"capturedUtc": _utc(), "decoded": state,
                "currentAngle": self.current_angle, "lastApplied": self.last_applied}

    def require_unchanged(self) -> dict:
        if self.current_angle is None or not self.last_applied:
            raise RuntimeError("No locally acknowledged angle is known; apply a numeric angle first")
        state = self._fresh_state()
        cfg = state["algorithmConfig"]
        expected = self.last_applied
        for key in ("receivedSequence", "appliedSequence", "activeVersion"):
            if int(cfg[key]) != int(expected[key]):
                self.current_angle = None
                self.last_applied = None
                raise ExternalStateChanged(f"External writer/reset detected: {key} no longer matches")
        return state

    def _config_dir(self, angle: float) -> Path:
        key = angle % 360.0
        repo = self.board_root.parents[2]
        preset_root = repo / "data" / "calibration" / "ov5640_user_photos_20261022" / "rotation_validation_20261023"
        preset = PRESETS.get(key)
        if preset:
            candidates = [self.board_root / "reports" / "algorithm1080_board_rejoin_r8_20261022" / preset,
                          preset_root / preset,
                          repo / "data" / "calibration" / "ov5640_user_photos_20261022" / "board_validation_20261023" / preset]
            for candidate in candidates:
                if candidate.is_dir() and self._valid_config(candidate, angle):
                    return candidate
        cache_root = self.work_dir / "range_cache"
        cache_root.mkdir(exist_ok=True)
        cachekey = hashlib.sha256(f"{key.hex()}|{PROFILE_SHA256}|{HELPER_SHA256}|{BIT_SHA256}".encode()).hexdigest()[:20]
        for existing in cache_root.glob(cachekey + "_*"):
            try:
                qualified = _load(existing / "qualified_binding.json")
                if (qualified.get("bitSha256") == BIT_SHA256 and
                        qualified.get("profileSha256") == PROFILE_SHA256 and
                        qualified.get("helperSha256") == HELPER_SHA256 and
                        qualified.get("bindingSha256") == _sha(existing / "binding.json") and
                        _angle_distance(float(qualified.get("angle")), key) < 1e-12 and
                        self._valid_config(existing, angle)):
                    return existing
            except (OSError, ValueError, json.JSONDecodeError):
                continue
        target = cache_root / f"{cachekey}_{uuid.uuid4().hex}"
        argv = [sys.executable, "-B", str(self._helper), "prepare", "--profile", str(self.profile_path),
                "--angle", format(key, ".17g"), "--border", "clamp", "--mode", "original", "--output", str(target)]
        self._log_run("prepare", argv)
        if not self._valid_config(target, angle):
            raise ValueError("Fresh prepare output failed exact binding/profile/config validation")
        (target / "qualified_binding.json").write_text(json.dumps({
            "bitSha256": BIT_SHA256, "profileSha256": PROFILE_SHA256,
            "helperSha256": HELPER_SHA256, "angle": key,
            "bindingSha256": _sha(target / "binding.json")}, indent=2), encoding="utf-8")
        return target

    def _valid_config(self, config: Path, angle: float) -> bool:
        try:
            binding = _load(config / "binding.json")
            for name, digest in binding.items():
                if _sha(Path(name)) != digest:
                    return False
            if binding.get(str(self.profile_path)) != PROFILE_SHA256 or binding.get(str(self._helper)) != HELPER_SHA256:
                return False
            payload = (config / "config_payload.bin").read_bytes()
            if len(payload) != 92:
                return False
            words = [int.from_bytes(payload[i:i + 4], "little") for i in range(0, 92, 4)]
            if tuple(words[i] for i in (0, 1, 18, 19)) != (1920, 1080, 1920, 1080):
                return False
            cfg = _load(config / "config.json")
            profile = _load(self.profile_path)
            if cfg.get("profile") != {k: profile[k] for k in ("model", "width", "height", "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2")}:
                return False
            words = cfg["words"]
            if (words.get("border_mode") != 1 or words.get("edge_mode") != 0 or
                    cfg.get("output_width") != 1920 or cfg.get("output_height") != 1080):
                return False
            theta = math.radians(math.remainder(angle, 360.0))
            c, s = math.cos(theta), math.sin(theta)
            c = 0.0 if abs(c) < 1e-15 else c
            s = 0.0 if abs(s) < 1e-15 else s
            cx, cy = (1920 - 1) / 2, (1080 - 1) / 2
            affine = cfg["inverse_affine"]
            expected = {"a00": c, "a01": -s, "a02": cx - c * cx + s * cy,
                        "a10": s, "a11": c, "a12": cy - s * cx - c * cy}
            if not all(abs(float(affine[k]) - v) <= 1e-12 for k, v in expected.items()):
                return False
            names = ("source_width", "source_height", "fx_q16", "fy_q16", "cx_q16", "cy_q16",
                     "inv_fx_q24", "inv_fy_q24", "k1_q24", "k2_q24", "p1_q24", "p2_q24",
                     "a00_q16", "a01_q16", "a02_q16", "a10_q16", "a11_q16", "a12_q16",
                     "output_width", "output_height", "border_mode", "edge_threshold", "edge_mode")
            config_words = cfg["words"]
            binary_words = [int.from_bytes(payload[i:i + 4], "little") for i in range(0, 92, 4)]
            if len(binary_words) != len(names) or any((int(config_words[name]) & 0xFFFFFFFF) != word
                                                       for name, word in zip(names, binary_words)):
                return False
            range_report = _load(config / "range_report.json")
            return (range_report.get("full_frame_widths_fit") is True and
                    range_report.get("count") == 1920 * 1080 and
                    range_report.get("affine_overflow_count") == 0 and
                    range_report.get("coordinate_saturation_count") == 0 and
                    binding.get(str((config / "config.json").resolve())) == _sha(config / "config.json") and
                    binding.get(str((config / "config_payload.bin").resolve())) == _sha(config / "config_payload.bin") and
                    binding.get(str((config / "range_report.json").resolve())) == _sha(config / "range_report.json"))
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return False

    def apply_angle(self, angle: float) -> dict:
        value = float(angle)
        if not math.isfinite(value):
            raise ValueError("Angle must be finite")
        normalized = value % 360.0
        try:
            before = self._fresh_state()["algorithmConfig"]
            config = self._config_dir(normalized)
            sequence = _next_sequence(before)
            output = self.work_dir / ("send_" + uuid.uuid4().hex)
            argv = [sys.executable, "-B", str(self._helper), "send", "--config", str(config), "--sequence", str(sequence),
                    "--acceptance", str(self._acceptance_path), "--session", str(self.session_path), "--output", str(output), "--port", self.port]
            self._log_run("send", argv, timeout=360)
            result = _load(output / "result.json")
            if result.get("status") != "APPLIED":
                raise RuntimeError("Original send CLI did not report APPLIED")
            cfg = result.get("applied") or {}
            if int(cfg.get("receivedSequence", -1)) != sequence or int(cfg.get("appliedSequence", -1)) != sequence:
                raise RuntimeError("Original send result sequence confirmation mismatch")
            # Confirm the status after the CLI has closed its own port.
            after = self._fresh_state()["algorithmConfig"]
            if int(after["receivedSequence"]) != sequence or int(after["appliedSequence"]) != sequence:
                raise RuntimeError("Fresh post-send diagnostics do not confirm applied sequence")
            if int(after["activeVersion"]) != int(cfg.get("activeVersion", -1)):
                raise RuntimeError("Fresh post-send activeVersion differs from acknowledged result")
            self.current_angle = normalized
            self.last_applied = {k: int(after[k]) for k in ("receivedSequence", "appliedSequence", "activeVersion")}
            return result
        except BaseException:
            self.current_angle = None
            self.last_applied = None
            raise


def discover_session(board_root: Path) -> Path:
    board = Path(board_root).resolve()
    acceptance, _, _ = _binding_check(board, _default_profile(board))
    report = board.parent / "reports" / "algorithm1080_board_rejoin_r8_20261022"
    candidates = list(report.glob("program*/program_session.json"))
    valid = []
    for path in candidates:
        try:
            session = _load(path)
            guard = _load_guard(board)
            guard.check_session(session, acceptance)
            valid.append((session.get("verifyCompletedUtc", ""), path))
        except Exception:
            continue
    if not valid:
        raise FileNotFoundError("No successful original qualified R8 session found")
    return max(valid, key=lambda item: item[0])[1]


def _default_profile(board: Path) -> Path:
    root = board.parents[2]
    return root / "data" / "calibration" / "ov5640_user_photos_20261022" / "board_validation_20261023" / "measured_native_profile" / "camera_profile_1080_runtime.json"


def _load_guard(board: Path):
    import importlib.util
    path = board / "tools" / "program_session_guard.py"
    spec = importlib.util.spec_from_file_location("_rotation_discover_guard", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def program_new_session(board_root: Path) -> Path:
    board = Path(board_root).resolve()
    acceptance = _load(board / "reports" / "physical_acceptance.json")
    for name, digest in {**acceptance["sourceHashes"], **acceptance["evidenceHashes"]}.items():
        if _sha(Path(name)) != digest:
            raise ValueError(f"Original qualified SHA scope changed: {name}")
    reports = board.parent / "reports" / "algorithm1080_board_rejoin_r8_20261022"
    reports.mkdir(parents=True, exist_ok=True)
    attempts = ["program", *[f"program_retry{i}" for i in range(1, 10000)]]
    attempt = next((name for name in attempts if not (reports / name).exists() and not (reports / f"{name}_cdt_server").exists()), None)
    if attempt is None:
        raise RuntimeError("No unused original programming attempt name remains")
    argv = [sys.executable, "-B", str(board / "tools" / "program.py"), "--attempt", attempt]
    logdir = reports / ("console_" + attempt + "_" + uuid.uuid4().hex)
    logdir.mkdir(exist_ok=False)
    (logdir / "command.json").write_text(json.dumps({"argv": argv, "startedUtc": _utc()}, indent=2), encoding="utf-8")
    # program.py owns the CDT child and guarantees its termination in its
    # finally block. Let that wrapper finish rather than externally timing
    # it out and orphaning its owned service.
    try:
        cp = subprocess.run(argv, cwd=board, capture_output=True, text=True, encoding="utf-8", errors="replace",
                            check=False, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    except BaseException as exc:
        (logdir / "stderr.log").write_text(repr(exc), encoding="utf-8")
        (logdir / "exit.json").write_text(json.dumps({"exception": repr(exc), "endedUtc": _utc()}, indent=2), encoding="utf-8")
        raise
    (logdir / "stdout.log").write_text(cp.stdout or "", encoding="utf-8")
    (logdir / "stderr.log").write_text(cp.stderr or "", encoding="utf-8")
    (logdir / "exit.json").write_text(json.dumps({"returncode": cp.returncode, "endedUtc": _utc()}, indent=2), encoding="utf-8")
    if cp.returncode:
        raise subprocess.CalledProcessError(cp.returncode, argv, cp.stdout, cp.stderr)
    session = reports / attempt / "program_session.json"
    guard = _load_guard(board)
    guard.check_session(_load(session), acceptance)
    return session
