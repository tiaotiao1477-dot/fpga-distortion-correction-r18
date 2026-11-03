"""Daily V4 board interaction bound to fresh live protocols, not certification.

Runtime records produced here deliberately never claim a passed physical
qualification or identify an accepted FPGA bitstream. The strict offline R14
qualification path remains the default in :mod:`auto_correction`.
"""
from __future__ import annotations

import contextlib
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
NN_ROOT = ROOT / "wanchengceshi" / "algorithm_v04_1080p_upgrade" / "algorithm1080_nn_r1"
NN_TOOLS = NN_ROOT / "tools"
R8_TOOLS = NN_ROOT.parent / "algorithm1080_rejoin_r8" / "tools"
INTEGRATION_TOOLS = NN_ROOT.parent.parent / "algorithm_v04_integration" / "tools"
DEFAULT_PROFILE = (ROOT / "data" / "calibration" / "ov5640_user_photos_20261022"
                   / "board_validation_20261023" / "measured_native_profile"
                   / "camera_profile_1080_runtime.json")
EXPECTED_CRC32 = "4ce634dd"
SCOPE = "live_protocol_only"
MAX_LIVE_AGE_SECONDS = 8.0
CNN_QUIET_IDLE_TIMEOUT_SECONDS = 10.0


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _utc() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def _read_json(path: Path) -> dict:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def _round_half_away(value: float) -> int:
    if not math.isfinite(value):
        raise ValueError("K contains a nonfinite value")
    return math.floor(value + .5) if value >= 0 else math.ceil(value - .5)


def profile_k_q11(profile) -> tuple[int, int, int, int]:
    """Map measured 1920x1080 K into 640x480 canvas S16Q11 registers.

    The 1920x1080 camera image is scaled by exactly 1/3 into a 640x360 image,
    centered in the NN's 640x480 canvas (60 rows of vertical offset). K values
    are converted to signed Q11 with round-half-away-from-zero.
    """
    if (int(profile.width), int(profile.height)) != (1920, 1080):
        raise ValueError("Live CNN K setup requires the measured native 1920x1080 profile")
    # NN outputs are normalized over endpoint indices 0..639 and 0..479.
    continuous = (float(profile.fx) / 3.0 / 639.0,
                  float(profile.fy) / 3.0 / 479.0,
                  float(profile.cx) / 3.0 / 639.0,
                  (float(profile.cy) / 3.0 + 60.0) / 479.0)
    q = tuple(_round_half_away(value * 2048.0) for value in continuous)
    for value in q:
        if not -32768 <= value <= 32767:
            raise ValueError("Mapped K does not fit the signed S16Q11 registers")
    if q[0] <= 0 or q[1] <= 0:
        raise ValueError("Mapped focal lengths must be positive")
    return q


def _load_profile(profile_path: Path):
    import fast_geometry
    from sw.reference_common.camera_profile import load_camera_profile
    profile_path = Path(profile_path).resolve(strict=True)
    if _sha(profile_path) != fast_geometry.PROFILE_SHA:
        raise ValueError("Live control requires the exact frozen measured native profile")
    profile = load_camera_profile(profile_path)
    if (profile.width, profile.height) != (1920, 1080):
        raise ValueError("Live control supports only the fixed native 1920x1080 profile")
    return profile


def runtime_source_paths(profile_path: Path) -> tuple[Path, ...]:
    """Files whose exact bytes define the current live-control runtime."""
    paths = (
        HERE / "interactive_control.py", HERE / "auto_correction.py",
        HERE / "fast_geometry.py", HERE / "fast_audit.py", HERE / "fast_sender.py",
        HERE / "telemetry_client.py", HERE / "monitor_model.py",
        NN_TOOLS / "nn_board_observe.py", NN_TOOLS / "nn_host.py",
        NN_ROOT / "data" / "model_memory.json", NN_ROOT / "data" / "weights_le_padded.bin",
        R8_TOOLS / "algorithm_uart.py", R8_TOOLS / "decode_algorithm_packet.py",
        INTEGRATION_TOOLS / "make_uart_packet.py", R8_TOOLS / "raw1080_uart.py",
        R8_TOOLS / "decode_algorithm_packet.py", Path(profile_path).resolve(strict=True),
    )
    unique = []
    seen = set()
    for path in paths:
        path = Path(path).resolve(strict=True)
        if path not in seen:
            unique.append(path)
            seen.add(path)
    return tuple(unique)


def runtime_source_hashes(profile_path: Path) -> dict[str, str]:
    return {str(path): _sha(path) for path in runtime_source_paths(profile_path)}


def valid_capabilities() -> dict:
    """Test fixture helper for a synthetically valid, non-qualification record."""
    return {"a7": {"loaded": True, "enabled": True, "kReady": True,
                    "crc32": EXPECTED_CRC32, "expectedCrc32": EXPECTED_CRC32,
                    "nnError": 0, "protocolError": False, "faultLatched": False,
                    "saturated": False, "capabilityWord": (15 << 16) | 1},
            "a8": {"supported": True, "valid": True, "magicVersion": "474d0001"}}


def _validate_capabilities(capabilities: dict, *, require_ready: bool = False) -> None:
    """Validate CNN model-monitor capability, independently of geometry control."""
    if not isinstance(capabilities, dict):
        raise ValueError("Fresh A7/A8 capability record is unavailable")
    a7, a8 = capabilities.get("a7"), capabilities.get("a8")
    if not isinstance(a7, dict) or not isinstance(a8, dict):
        raise ValueError("Both A7 CNN and A8 monitor protocols are required")
    if (str(a7.get("expectedCrc32", "")).lower() != EXPECTED_CRC32 or
            (a7.get("loaded") is True and str(a7.get("crc32", "")).lower() != EXPECTED_CRC32) or
            int(a7.get("nnError", 1)) != 0 or a7.get("protocolError") is not False or
            a7.get("faultLatched") is not False or a7.get("saturated") is not False):
        raise ValueError("Live A7 does not match the expected CNN CRC/protocol")
    if (a8.get("supported") is not True or a8.get("magicVersion") != "474d0001"):
        raise ValueError("Live A8 monitor protocol magic/version is unavailable")
    capability_word = int(a7.get("capabilityWord", 0))
    if capability_word >> 16 < 15 or not (capability_word & 1):
        raise ValueError("CNN loading interface requires revision 15 with capability bit 0")
    if require_ready and not (a7.get("loaded") is True and a7.get("enabled") is True
                              and a7.get("kReady") is True
                              and str(a7.get("crc32", "")).lower() == EXPECTED_CRC32):
        raise ValueError("Fresh CNN telemetry does not confirm loaded, enabled, expected-CRC, K-ready state")


def _telemetry_k_values(telemetry: dict) -> tuple[int, int, int, int] | None:
    """Decode the four signed K registers from A7 telemetry words 13 and 14."""
    packed = telemetry.get("kPacked")
    if not isinstance(packed, (list, tuple)) or len(packed) != 2:
        return None
    try:
        words = [int(value, 16) if isinstance(value, str) else int(value) for value in packed]
        if any(not 0 <= word <= 0xFFFFFFFF for word in words):
            return None
        raw = b"".join(word.to_bytes(4, "little") for word in words)
        return struct.unpack("<hhhh", raw)
    except (TypeError, ValueError, OverflowError, struct.error):
        return None


class _RuntimeEvidence:
    """Small A7 evidence sink compatible with nn_board_observe.BoardLink."""
    def __init__(self, folder: Path, context: dict):
        folder.mkdir(parents=True, exist_ok=False)
        self.folder = folder
        self.raw = (folder / "a7_packets.bin").open("xb")
        self.index = (folder / "a7_packets.jsonl").open("x", encoding="utf-8")
        self.context = context
        self.packet_count = 0
        self.started_mono = time.monotonic()

    def save_packet(self, packet: bytes, *, action: str, command: dict | None,
                    request_mono: float, response_mono: float,
                    request_utc: str, response_utc: str, decoded: dict | None,
                    decode_error: str | None = None) -> None:
        offset = self.raw.tell()
        self.raw.write(packet)
        self.raw.flush()
        value = {**self.context, "ordinal": self.packet_count, "rawOffset": offset,
                 "rawBytes": len(packet), "action": action, "command": command,
                 "requestUtc": request_utc, "responseUtc": response_utc,
                 "requestSeconds": request_mono - self.started_mono,
                 "responseSeconds": response_mono - self.started_mono,
                 "rawHex": packet.hex(), "telemetry": decoded.get("telemetry") if decoded else None,
                 "geometryChecks": decoded.get("geometryChecks") if decoded else None,
                 "decodeError": decode_error, "qualificationScope": SCOPE}
        self.index.write(json.dumps(value, allow_nan=False) + "\n")
        self.index.flush()
        self.packet_count += 1

    def close(self):
        self.raw.close()
        self.index.close()


def _live_read_a7_a8(port: str) -> dict:
    """One fresh serialized A7 and A8 observation, including raw packets."""
    import serial
    import telemetry_client as telemetry
    from monitor_model import decode_cnn, decode_gamma
    uart = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.2)
    uart.port, uart.dtr, uart.rts = port, False, False
    try:
        uart.open()
        a7_packet = telemetry.read_packet(uart, b"\xa7", telemetry.A7_HEADER, telemetry.A7_SIZE)
        a7 = decode_cnn(a7_packet, NN_TOOLS / "nn_host.py")
        a8_packet = telemetry.read_packet(uart, b"\xa8", b"\x55\xaa\xe4\x14", 27)
        a8 = decode_gamma(a8_packet)
    finally:
        if uart.is_open:
            uart.close()
    decoded = a7["decoded"]
    flags = decoded["telemetry"]["flags"]
    return {"a7": {"loaded": flags.get("loaded"), "enabled": flags.get("enabled"),
                   "kReady": flags.get("k_valid"), "crc32": decoded["telemetry"].get("computedCrc32"),
                   "expectedCrc32": decoded["telemetry"].get("expectedCrc32"),
                   "capabilityWord": int(decoded["telemetry"].get("reservedWord31", 0)),
                   "nnError": flags.get("nn_error"), "protocolError": flags.get("protocol_error"),
                   "faultLatched": flags.get("fault_latched"), "saturated": flags.get("saturated"),
                   "predictionCount": decoded["telemetry"].get("predictionCount"),
                   "predictionFrame": decoded["telemetry"].get("predictionFrame"),
                   "rawHex": a7_packet.hex()},
            "a8": {"supported": True, "valid": a8.get("valid"), "magicVersion": "474d0001",
                   "decoded": a8, "rawHex": a8_packet.hex()},
            "observedUtc": _utc(), "observedMonotonic": time.monotonic()}


def _live_read_a5(port: str) -> dict:
    """One fresh native A5 packet with its raw bytes retained in the record."""
    import serial
    import telemetry_client as telemetry
    decoder_path, raw_path = R8_TOOLS / "decode_algorithm_packet.py", R8_TOOLS / "raw1080_uart.py"
    spec = importlib.util.spec_from_file_location("_v4_live_diag_decoder", decoder_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load native A5 decoder")
    decoder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(decoder)
    prior = sys.modules.get("decode_algorithm_packet")
    sys.modules["decode_algorithm_packet"] = decoder
    try:
        raw_spec = importlib.util.spec_from_file_location("_v4_live_raw_uart", raw_path)
        if raw_spec is None or raw_spec.loader is None:
            raise RuntimeError("Cannot load native A5 parser")
        raw_module = importlib.util.module_from_spec(raw_spec)
        raw_spec.loader.exec_module(raw_module)
    finally:
        if prior is None:
            sys.modules.pop("decode_algorithm_packet", None)
        else:
            sys.modules["decode_algorithm_packet"] = prior
    uart = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.5)
    uart.port, uart.dtr, uart.rts = port, False, False
    try:
        uart.open()
        packet = telemetry.read_packet(uart, b"\xa5", b"\x55\xaa\xd1\xf4", 251, 1.0)
        decoded = raw_module.decode_raw1080(packet)
    finally:
        if uart.is_open:
            uart.close()
    config = decoded.get("algorithmConfig")
    if not isinstance(config, dict) or config.get("fresh") is not True or config.get("frameError") is not False:
        raise ValueError("Fresh A5 native algorithm diagnostics are unavailable")
    decoded["rawPacketHex"] = packet.hex()
    decoded["observedUtc"] = _utc()
    decoded["observedMonotonic"] = time.monotonic()
    return decoded


class LiveBoardController:
    """Interactive controller whose authority comes from this live board only."""
    def __init__(self, port: str, profile_path: Path, work_dir: Path, notify,
                 *, io: dict | None = None):
        self.port = str(port)
        self.profile_path = Path(profile_path).resolve(strict=True)
        self.work_dir = Path(work_dir).resolve()
        self.notify = notify or (lambda _text: None)
        self.io = io or {}
        self.runtime_dir = self.work_dir / "live_control" / uuid.uuid4().hex
        self.runtime_dir.mkdir(parents=True, exist_ok=False)
        self.binding_path = self.runtime_dir / "runtime_binding.json"
        self.connection_path = self.runtime_dir / "runtime_connection.json"
        self.actions_path = self.runtime_dir / "runtime_actions.jsonl"
        self.observations_path = self.runtime_dir / "runtime_observations.jsonl"
        self.acceptance_path = self.binding_path
        self.session_path = self.connection_path
        self.current_angle: float | None = None
        self.last_applied: dict | None = None
        self.crop_enabled: bool | None = None
        self.crop_applied = None
        self._prepared_config = None
        self._capabilities = None
        self._runtime_sources = runtime_source_hashes(self.profile_path)
        self.binding_validator = self.validate_context

    @property
    def candidate_tools(self) -> Path:
        return NN_TOOLS

    def _fresh_a5(self) -> dict:
        if "read_a5" in self.io:
            return self.io["read_a5"](self.port)
        return _live_read_a5(self.port)

    def _fresh_capabilities(self) -> dict:
        if "read_capabilities" in self.io:
            return self.io["read_capabilities"](self.port)
        return _live_read_a7_a8(self.port)

    @staticmethod
    def _ack_from(state: dict) -> dict:
        config = state.get("algorithmConfig")
        if (not isinstance(config, dict) or config.get("fresh") is not True or
                config.get("frameError") is not False):
            raise ValueError("Fresh A5 status/ACK is unavailable")
        ack = {key: int(config[key]) for key in
               ("receivedSequence", "appliedSequence", "activeVersion")}
        return ack

    def _write_binding(self) -> None:
        import fast_geometry
        from auto_correction import _load_nn_host, _model_identity
        profile = _load_profile(self.profile_path)
        model_identity, manifest_path, weights_path = _model_identity(_load_nn_host())
        value = {"schemaVersion": 1, "recordType": "interactive_runtime_binding",
                 "qualificationScope": SCOPE, "passed": False,
                 "acceptedBitstreamSha256": None, "candidateBitstreamIdentity": None,
                 "expectedModelCrc32": EXPECTED_CRC32,
                 "profilePath": str(self.profile_path), "profileSha256": _sha(self.profile_path),
                 "nativeWidth": profile.width, "nativeHeight": profile.height,
                 "sourceHashes": dict(self._runtime_sources),
                 "modelIdentity": model_identity,
                 "modelManifestPath": str(manifest_path), "weightsPath": str(weights_path),
                 "fastGeometryFrozenProfileSha256": fast_geometry.PROFILE_SHA}
        self.binding_path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n",
                                     encoding="utf-8")

    def _write_connection(self, a5: dict | None = None, capabilities: dict | None = None,
                          *, actions: list | None = None) -> None:
        a5 = a5 if a5 is not None else getattr(self, "_last_a5", None)
        capabilities = capabilities if capabilities is not None else self._capabilities
        value = {"schemaVersion": 1, "recordType": "live_board_connection",
                 "qualificationScope": SCOPE, "passed": False,
                 "port": self.port, "connectionId": self.runtime_dir.name,
                 "runtimeBindingPath": str(self.binding_path),
                 "runtimeBindingSha256": _sha(self.binding_path),
                 "observationsPath": str(self.observations_path),
                 "connectedUtc": _utc(), "observedMonotonic": time.monotonic(),
                 "a5": a5, "ack": self.last_applied,
                 "a7a8": capabilities, "actions": list(actions or []),
                 "acceptedBitstreamSha256": None, "physicalQualification": False}
        self.connection_path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n",
                                        encoding="utf-8")

    def connect(self) -> dict:
        _load_profile(self.profile_path)
        a5 = self._fresh_a5()
        ack = self._ack_from(a5)
        # A5 owns geometry control. CNN/A8 telemetry is optional here so
        # compatible older FPGA images remain usable for manual angle/crop.
        try:
            capabilities = self._fresh_capabilities()
        except Exception as exc:
            capabilities = {"supported": False, "error": str(exc)}
        self.last_applied = ack
        self.current_angle = None
        self.crop_enabled = None
        self.crop_applied = None
        self._prepared_config = None
        self._last_a5 = a5
        self._capabilities = capabilities
        self._write_binding()
        self._write_connection(actions=[])
        self._append_observation(a5, capabilities)
        return {"status": "CONNECTED_LIVE_PROTOCOL_ONLY", "ack": dict(ack),
                "qualificationScope": SCOPE, "connectionPath": str(self.connection_path)}

    def _invalidate_ownership(self):
        self.current_angle = None
        self.last_applied = None
        self.crop_enabled = None
        self.crop_applied = None
        self._prepared_config = None

    def _append_observation(self, a5: dict, capabilities: dict) -> None:
        record = {"qualificationScope": SCOPE, "connectionId": self.runtime_dir.name,
                  "observedUtc": _utc(), "observedMonotonic": time.monotonic(),
                  "ack": self.last_applied, "a5": a5, "a7a8": capabilities}
        with self.observations_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def require_unchanged(self):
        if not self.last_applied:
            self._invalidate_ownership()
            raise RuntimeError("No fresh A5 connection ACK; reconnect before control")
        try:
            state = self._fresh_a5()
            ack = self._ack_from(state)
        except Exception:
            self._invalidate_ownership()
            raise
        if any(int(ack[key]) != int(self.last_applied[key]) for key in self.last_applied):
            self._invalidate_ownership()
            raise RuntimeError("Live A5 ACK changed; local configuration ownership is unknown")
        self._last_a5 = state
        self._append_observation(state, self._capabilities)
        return state

    def _validate_records(self, acceptance_path: Path, session_path: Path) -> tuple[dict, dict]:
        binding_path, connection_path = Path(acceptance_path).resolve(), Path(session_path).resolve()
        if binding_path != self.binding_path.resolve() or connection_path != self.connection_path.resolve():
            raise ValueError("Live runtime binding/connection paths do not belong to this controller")
        binding, connection = _read_json(binding_path), _read_json(connection_path)
        if (binding.get("recordType") != "interactive_runtime_binding" or
                connection.get("recordType") != "live_board_connection" or
                binding.get("qualificationScope") != SCOPE or connection.get("qualificationScope") != SCOPE or
                binding.get("passed") is not False or connection.get("passed") is not False or
                binding.get("acceptedBitstreamSha256") is not None or
                connection.get("acceptedBitstreamSha256") is not None or
                connection.get("physicalQualification") is not False):
            raise ValueError("Live record is incomplete or incorrectly claims physical qualification")
        if connection.get("runtimeBindingSha256") != _sha(binding_path):
            raise ValueError("Live connection no longer binds its runtime source record")
        if binding.get("profilePath") != str(self.profile_path) or _sha(self.profile_path) != binding.get("profileSha256"):
            raise ValueError("Measured native profile changed since live connection")
        expected = binding.get("sourceHashes")
        if not isinstance(expected, dict) or expected != self._runtime_sources:
            raise ValueError("Live runtime source set differs from the current controller")
        for name, digest in expected.items():
            if _sha(Path(name)) != digest:
                raise ValueError(f"Live control source/tool changed: {name}")
        if (binding.get("expectedModelCrc32") != EXPECTED_CRC32 or
                binding.get("nativeWidth") != 1920 or binding.get("nativeHeight") != 1080):
            raise ValueError("Live runtime profile/model compatibility has changed")
        return binding, connection

    def validate_context(self, acceptance_path: Path, session_path: Path) -> tuple[dict, dict]:
        """Validate runtime/profile binding and current A5 geometry ownership."""
        binding, connection = self._validate_records(acceptance_path, session_path)
        self.require_unchanged()
        return binding, connection

    def validate_auto_context(self, acceptance_path: Path, session_path: Path) -> tuple[dict, dict]:
        """Strict auto-write callback requiring fresh A5, CNN/A8, and R15 capability."""
        binding, connection = self.validate_context(acceptance_path, session_path)
        capabilities = self._fresh_capabilities()
        _validate_capabilities(capabilities, require_ready=True)
        self._capabilities = capabilities
        return binding, connection

    def record_auto_applied(self, angle, ack, config_dir, crop_report):
        self.current_angle = float(angle)
        self.last_applied = {key: int(ack[key]) for key in
                             ("receivedSequence", "appliedSequence", "activeVersion")}
        self._prepared_config = Path(config_dir)
        self.crop_enabled = bool(crop_report.get("cropEnabled", False))
        self.crop_applied = dict(crop_report) if self.crop_enabled else None
        with self.actions_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"qualificationScope": SCOPE, "action": "geometry_apply_ack",
                "angle": self.current_angle, "ack": self.last_applied,
                "configDir": str(self._prepared_config), "cropEnabled": self.crop_enabled,
                "timeUtc": _utc()}, ensure_ascii=False) + "\n")

    def apply_angle(self, angle):
        from auto_correction import apply_manual_angle
        return apply_manual_angle(self, float(angle), self.binding_path, self.connection_path,
            self.profile_path, self.work_dir / "live_geometry", crop=bool(self.crop_enabled),
            progress=self.notify, allow_unknown_initial=True,
            binding_validator=self.binding_validator)

    def set_crop(self, enabled):
        if self.current_angle is None:
            raise RuntimeError("Set a manual angle first to establish a live ACK")
        from auto_correction import apply_manual_angle
        return apply_manual_angle(self, float(self.current_angle), self.binding_path,
            self.connection_path, self.profile_path, self.work_dir / "live_geometry",
            crop=bool(enabled), progress=self.notify, allow_unknown_initial=False,
            binding_validator=self.binding_validator)

    def crop_state(self):
        known = self.last_applied is not None and self.crop_enabled is not None
        return {"known": known, "enabled": self.crop_enabled if known else None,
                "zoom": (self.crop_applied or {}).get("zoom", 1.0) if known else None,
                "report": (str(self._prepared_config / "crop_report.json")
                           if known and self._prepared_config else None)}

    def prepare_cnn(self, progress=None, *, _link_factory=None) -> dict:
        """Explicit one-shot model/K/enable preparation with fresh A7 ACKs."""
        if not self.connection_path.is_file():
            raise RuntimeError("Connect and validate the live board before CNN preparation")
        self.validate_context(self.binding_path, self.connection_path)
        # Reject incompatible firmware before constructing the model transport.
        # BoardLink rechecks A7 word 31 immediately before any model command.
        _validate_capabilities(self._fresh_capabilities())
        from auto_correction import _load_nn_host, _model_identity
        host = _load_nn_host()
        model, manifest_path, weights_path = _model_identity(host)
        profile = _load_profile(self.profile_path)
        k_values = profile_k_q11(profile)
        action_dir = self.runtime_dir / ("prepare_" + uuid.uuid4().hex)
        evidence_context = {"qualificationScope": SCOPE, "connectionId": self.runtime_dir.name,
                            "modelCrc32": model["expectedCrc32"],
                            "modelWeightsSha256": model["weightsSha256"],
                            "manifestSha256": model["manifestSha256"]}
        evidence = _RuntimeEvidence(action_dir, evidence_context)
        nn_board = _load_board_observer()
        link = None
        actions = []
        stdout = _ProgressForwarder(progress or self.notify)
        try:
            factory = _link_factory or self.io.get("link_factory")
            link = factory(self.port, evidence) if factory else nn_board.BoardLink(self.port, evidence)
            initial = link.read_a7(action="prepare_initial_status", command=None)
            nn_board.BoardLink.check_errors(initial)
            telem = initial["telemetry"]
            expected_crc = int(telem["expectedCrc32"], 16)
            capability = int(telem.get('reservedWord31', 0))
            if (capability >> 16) < 15 or not (capability & 1):
                raise RuntimeError('当前板卡的 CNN 加载接口需要修复版位流；已停止，未发送模型数据。')
            if expected_crc != int(model["expectedCrc32"], 16):
                raise ValueError("FPGA expected model CRC differs from current verified weights")
            flags = telem["flags"]
            already_loaded = (flags["loaded"] and int(telem["loadedBytes"]) == int(host.PADDED_BYTES) and
                              int(telem["computedCrc32"], 16) == expected_crc)
            k_matches = (flags["k_valid"] and _telemetry_k_values(telem) == tuple(k_values))
            current = initial

            if not (already_loaded and k_matches):
                if flags["enabled"]:
                    with contextlib.redirect_stdout(stdout):
                        current = nn_board.set_enabled(link, False, 4.0)
                    actions.append({"action": "disable", "acknowledged": True})
                else:
                    actions.append({"action": "disable", "skipped": True,
                                    "reason": "NN is already disabled"})

                # BEGIN and K both require a quiet inference/cache boundary. Poll
                # fresh A7 data after disable ACK; never clear or reset a busy model.
                expected_seq = int(current["telemetry"]["lastCompletedSeq"]) & 0xFFFFFFFF
                monotonic = self.io.get("monotonic", time.monotonic)
                sleep = self.io.get("sleep", time.sleep)
                idle_deadline = monotonic() + CNN_QUIET_IDLE_TIMEOUT_SECONDS
                stdout.write("正在等待当前 CNN 推理和缓存操作结束…\n")
                quiet = None
                while True:
                    remaining = idle_deadline - monotonic()
                    if remaining <= 0:
                        actions.append({"action": "wait_quiet_idle", "timedOut": True})
                        raise TimeoutError(
                            "CNN stayed busy for 10 seconds; 未写K/未清模型；"
                            f"A7 evidence retained at {action_dir}")
                    try:
                        sample = link.read_a7(action="prepare_wait_quiet_idle", command=None,
                                              timeout=min(4.0, max(0.1, remaining)))
                    except TimeoutError as exc:
                        raise TimeoutError(
                            "等待 CNN 空闲时 A7 响应超时；未写K/未清模型；"
                            f"evidence: {action_dir}") from exc
                    nn_board.BoardLink.check_errors(sample)
                    sample_telem = sample["telemetry"]
                    sample_flags = sample_telem["flags"]
                    if int(sample_telem["expectedCrc32"], 16) != expected_crc:
                        raise RuntimeError("FPGA expected model CRC changed while waiting for quiet idle")
                    if (int(sample_telem["lastCompletedSeq"]) & 0xFFFFFFFF) != expected_seq:
                        raise RuntimeError("A7 command sequence ownership changed while waiting for quiet idle")
                    if (not sample_flags["enabled"] and not sample_flags["nn_busy"] and
                            not sample_flags["capture_busy"] and not sample_flags["image_valid"] and
                            sample_flags["cache_idle"]):
                        quiet = sample
                        break
                    sleep(min(0.1, max(0.0, idle_deadline - monotonic())))
                actions.append({"action": "wait_quiet_idle", "acknowledged": True,
                                "lastCompletedSeq": expected_seq})
                current = quiet
                if already_loaded and not (
                        quiet["telemetry"]["flags"]["loaded"] and
                        int(quiet["telemetry"]["loadedBytes"]) == int(host.PADDED_BYTES) and
                        int(quiet["telemetry"]["computedCrc32"], 16) == expected_crc):
                    raise RuntimeError("Loaded model changed while waiting; stopped without writing K or uploading")

                if already_loaded:
                    actions.append({"action": "upload", "skipped": True,
                                    "reason": "live loadedBytes and computed CRC match verified model"})
                else:
                    with contextlib.redirect_stdout(stdout):
                        current = nn_board.upload_weights(link, 4.0, 1800.0)
                    actions.append({"action": "upload", "acknowledged": True})
                # Upload BEGIN clears K, so program K after any upload even when
                # the pre-upload packed words happened to match the profile.
                with contextlib.redirect_stdout(stdout):
                    current = nn_board.send_k(link, *k_values, 4.0)
                actions.append({"action": "set_k", "valuesS16Q11": list(k_values),
                                "acknowledged": True})

            current_flags = current["telemetry"]["flags"]
            if not current_flags["enabled"]:
                with contextlib.redirect_stdout(stdout):
                    current = nn_board.set_enabled(link, True, 4.0)
                actions.append({"action": "enable", "acknowledged": True})
            final = link.read_a7(action="prepare_final_status", command=None)
            nn_board.BoardLink.check_errors(final)
            flags = final["telemetry"]["flags"]
            if (not flags["loaded"] or not flags["enabled"] or not flags["k_valid"] or
                    int(final["telemetry"]["computedCrc32"], 16) != expected_crc or
                    int(final["telemetry"]["expectedCrc32"], 16) != expected_crc or
                    int(final["telemetry"]["loadedBytes"]) != int(host.PADDED_BYTES) or
                    _telemetry_k_values(final["telemetry"]) != tuple(k_values) or
                    int(final["telemetry"]["lastCompletedSeq"]) !=
                    int(current["telemetry"]["lastCompletedSeq"])):
                raise RuntimeError("Final A7 did not confirm expected loaded/enabled/K-ready state")
            actions.append({"action": "final_status", "acknowledged": True,
                            "computedCrc32": final["telemetry"]["computedCrc32"]})
        except Exception as exc:
            (action_dir / "failure.json").write_text(json.dumps({
                "error": str(exc), "actions": actions, "timeUtc": _utc(),
                "qualificationScope": SCOPE}, ensure_ascii=False, indent=2), encoding="utf-8")
            raise
        finally:
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass
            evidence.close()
        for name, digest in self._runtime_sources.items():
            if _sha(Path(name)) != digest:
                raise ValueError(f"Live runtime source changed during model preparation: {name}")
        action_record = {"folder": str(action_dir), "actions": actions,
                         "evidenceFiles": {path.name: _sha(path) for path in action_dir.iterdir()
                                           if path.is_file()},
                         "qualificationScope": SCOPE}
        with self.actions_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(action_record, ensure_ascii=False) + "\n")
        self._capabilities = self._fresh_capabilities()
        _validate_capabilities(self._capabilities, require_ready=True)
        self.require_unchanged()
        return {"status": "PREPARED_LIVE_PROTOCOL_ONLY", "kS16Q11": list(k_values),
                "model": model, "actions": actions, "qualificationScope": SCOPE,
                "evidenceDir": str(action_dir)}


class _ProgressForwarder:
    def __init__(self, callback):
        self.callback = callback
        self.buffer = ""

    def write(self, text):
        self.buffer += text
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if line.strip():
                self.callback(line.strip())
        return len(text)

    def flush(self):
        if self.buffer.strip():
            self.callback(self.buffer.strip())
        self.buffer = ""


def _load_board_observer():
    spec = importlib.util.spec_from_file_location("_v4_live_nn_board_observe", NN_TOOLS / "nn_board_observe.py")
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load current NN board preparation transport")
    module = importlib.util.module_from_spec(spec)
    old_path = list(sys.path)
    prior_nn_host = sys.modules.pop("nn_host", None)
    sys.path.insert(0, str(NN_TOOLS))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path[:] = old_path
        sys.modules.pop("nn_host", None)
        if prior_nn_host is not None:
            sys.modules["nn_host"] = prior_nn_host
    return module


def validate_live_control_binding(acceptance_path: Path, session_path: Path, *,
                                  controller: LiveBoardController | None = None) -> tuple[dict, dict]:
    """Validate runtime source records and fresh board capability, not a certificate."""
    if controller is None:
        raise ValueError("Live runtime validation requires its connected controller")
    return controller.validate_context(acceptance_path, session_path)


def validate_live_auto_binding(acceptance_path: Path, session_path: Path, *,
                               controller: LiveBoardController | None = None) -> tuple[dict, dict]:
    """Strict CNN-write binding callback; validates fresh readiness and R15 gate."""
    if controller is None:
        raise ValueError("Live runtime validation requires its connected controller")
    return controller.validate_auto_context(acceptance_path, session_path)
