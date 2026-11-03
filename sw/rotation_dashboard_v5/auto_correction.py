"""Low-rate, opt-in CNN to Brown-distortion/rotation configuration loop.

Predictions remain observations until the explicit policy is enabled and a
full native-raster audit plus current R14 candidate ACK succeeds.
"""
from __future__ import annotations

import hashlib
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid
from dataclasses import replace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
NN_BOARD = ROOT / "wanchengceshi" / "algorithm_v04_1080p_upgrade" / "algorithm1080_nn_r1" / "tools" / "nn_board_observe.py"
NN_HOST = NN_BOARD.with_name("nn_host.py")
NN_ROOT = NN_BOARD.parents[1]
R14_SOURCE_MANIFEST = NN_ROOT / "source_inputs_revision14.json"
R14_SOURCE_MANIFEST_SHA256 = "c2bbee1cd32da09385355f413a7043364fa47f22f14dc1ba932644ee65b5431c"
R14_MONITOR_SUFFIXES = (
    "revision14/rtl/algorithm1080_nn_top_monitor_r14.v",
    "revision14/rtl/distortion_storage_engine_monitor_r14.v",
    "revision14/rtl/camera_diag_uart_monitor_r14.v",
    "revision14/algorithm1080_nn.fdc",
)
EXPECTED_CRC32 = "4ce634dd"
FROZEN_UART_SHA256 = "64a6941b0ffaee1068a92fef0b133f9712bfab57db4fadfe7e999c9d5f46fa45"
FROZEN_DECODER_SHA256 = "0af6a5dd20bd9759741d79fa3119c4025c28b4436c939382206f6a069fdb0bef"
FROZEN_PACKET_BUILDER_SHA256 = "512c95d30946cce0dc6ff2381401675d5c345dab70af4047d4a30f65ba5af387"
MAX_AGE_SECONDS = 8.0
MIN_UPDATE_SECONDS = 5.0
R8_TOOLS = NN_ROOT.parent / "algorithm1080_rejoin_r8" / "tools"
FROZEN_DECODER = R8_TOOLS / "decode_algorithm_packet.py"
FROZEN_PACKET_BUILDER = NN_ROOT.parent.parent / "algorithm_v04_integration" / "tools" / "make_uart_packet.py"


class AutoCorrectionError(RuntimeError):
    """Transaction failure with explicit knowledge of whether a write may have started."""

    def __init__(self, message: str, *, uncertain: bool, cause: BaseException | None = None,
                 ownership_lost: bool = False):
        super().__init__(message)
        self.uncertain = bool(uncertain)
        self.cause = cause
        self.ownership_lost = bool(ownership_lost)


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _read_json(path: Path) -> dict:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def _canonical_path(path: str | Path) -> str:
    return os.path.normcase(os.path.abspath(os.path.normpath(str(path)))).replace("/", "\\")


def _r14_monitor_source_hashes() -> dict[str, str]:
    """Read the exact current R14 source manifest and return its four monitor inputs."""
    if _sha(R14_SOURCE_MANIFEST) != R14_SOURCE_MANIFEST_SHA256:
        raise ValueError("R14 source manifest SHA-256 changed")
    manifest = _read_json(R14_SOURCE_MANIFEST)
    if (manifest.get("candidate") != "algorithm1080_nn_r1" or manifest.get("revision") != 14 or
            manifest.get("scope") != "revision14_isolated_gamma_monitor_source_only_project"):
        raise ValueError("Source manifest does not describe the isolated R14 monitor project")
    source_hashes = manifest.get("sourceHashes")
    if not isinstance(source_hashes, dict):
        raise ValueError("R14 source manifest is missing sourceHashes")
    selected = {}
    for suffix in R14_MONITOR_SUFFIXES:
        matches = [(str(name), str(digest)) for name, digest in source_hashes.items()
                   if str(name).replace("\\", "/").lower().endswith(suffix.lower())]
        if len(matches) != 1:
            raise ValueError(f"R14 source manifest must bind exactly one {suffix}")
        name, digest = matches[0]
        path = Path(name)
        if _sha(path) != digest:
            raise ValueError(f"R14 monitor source differs from its source manifest: {name}")
        selected[_canonical_path(path)] = digest
    return selected


def _validate_r14_program_binding(acceptance_path: Path, session_path: Path):
    acceptance, session = _load_r14_guard().validate_program_binding(acceptance_path, session_path)
    accepted_sources = acceptance.get("sourceHashes")
    if not isinstance(accepted_sources, dict):
        raise ValueError("R14 acceptance has no sourceHashes")
    normalized = {_canonical_path(name): str(digest) for name, digest in accepted_sources.items()}
    for name, digest in _r14_monitor_source_hashes().items():
        if normalized.get(name) != digest:
            raise ValueError("Current acceptance does not bind the R14 monitor sources")
    return acceptance, session


def _validate_control_binding(acceptance_path: Path, session_path: Path,
                              binding_validator=None):
    """Use live protocol evidence only when explicitly selected by the caller."""
    validator = binding_validator or _validate_r14_program_binding
    return validator(Path(acceptance_path), Path(session_path))


def _require_fresh_prediction(proposal: dict, now: float | None = None) -> None:
    try:
        observed = float(proposal["observed_at"])
        now = time.monotonic() if now is None else float(now)
        age = now - observed
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise AutoCorrectionError("CNN prediction timestamp is invalid", uncertain=False, cause=exc) from exc
    if not math.isfinite(age) or age < 0 or age > MAX_AGE_SECONDS:
        raise AutoCorrectionError("CNN proposal is stale before geometry update", uncertain=False)


def _require_controller_unchanged(controller) -> None:
    try:
        controller.require_unchanged()
    except BaseException as exc:
        # A failed fresh-state comparison means last ACK ownership can no
        # longer be asserted even though this path has not sent any bytes.
        _clear_controller_ack_state(controller)
        raise AutoCorrectionError("Controller ownership could not be confirmed; angle state is unknown",
                                 uncertain=False, cause=exc, ownership_lost=True) from exc


def _clear_controller_ack_state(controller) -> None:
    """Drop cached settings that depend on an ACK once ownership is unknown."""
    for name, value in (("current_angle", None), ("last_applied", None),
                        ("crop_enabled", None), ("crop_applied", None),
                        ("_prepared_config", None)):
        try:
            setattr(controller, name, value)
        except Exception:
            pass


def _controller_source(controller) -> Path:
    source = Path(inspect.getfile(type(controller))).resolve(strict=True)
    if not source.is_file():
        raise ValueError("Controller implementation source is unavailable for provenance binding")
    return source


def _scan_predicted_geometry(config, crop: bool, progress=None):
    import fast_geometry as geometry
    callback = None
    if progress is not None:
        callback = lambda done, total: progress(f"V4 CNN full-frame audit {done:,}/{total:,} pixels")
    return geometry.scan_crop(config, crop, progress=callback, proof_scope="predicted_brown")


def _persist_transaction_result(target: Path, result: dict) -> None:
    try:
        (target / "transaction_result.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except Exception as exc:
        result["provenance"]["persistenceWarning"] = f"ACK succeeded; result file could not be written: {exc!r}"


def _norm_angle(value: float) -> float:
    return math.remainder(value, 360.0)


def _counter_advanced(new: int, old: int) -> bool:
    delta = (new - old) & 0xffffffff
    return 0 < delta < 0x80000000


def _unwrap_near(angle: float, reference: float) -> float:
    return reference + math.remainder(angle - reference, 360.0)


class AutoCorrectionPolicy:
    """Gate CNN telemetry and issue only stable, distinct latest-target proposals."""

    def __init__(self):
        self.enabled = False
        self._history: list[dict] = []
        self._last_count: int | None = None
        self._last_crc: str | None = None
        self._last_observed: float | None = None
        self._last_applied_at: float | None = None
        self._last_target: dict | None = None
        self._pending: dict | None = None

    def _clear_history(self) -> None:
        self._history.clear()
        self._last_count = None
        self._last_crc = None
        self._last_observed = None
        self._pending = None

    def enable(self) -> None:
        self._clear_history()
        self._last_target = None
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self._clear_history()
        self._last_target = None

    def failed(self) -> None:
        """Pause automatic ownership after an uncertain transaction."""
        self.disable()

    def refused(self) -> None:
        """Discard a preflight proposal while keeping explicit auto mode enabled."""
        if self.enabled:
            self._clear_history()
            self._last_target = None

    @staticmethod
    def _decoded(cnn: dict) -> tuple[dict, bytes | None, float | None]:
        if not isinstance(cnn, dict):
            return {}, None, None
        decoded = cnn.get("decoded", cnn)
        raw = cnn.get("raw_packet", cnn.get("raw_a7", cnn.get("raw_packet_hex")))
        if isinstance(raw, str):
            try:
                raw = bytes.fromhex(raw)
            except ValueError:
                raw = None
        if not isinstance(raw, bytes):
            raw = None
        observed = cnn.get("observed_at", cnn.get("received_monotonic"))
        try:
            observed = float(observed) if observed is not None else None
        except (TypeError, ValueError):
            observed = None
        return decoded if isinstance(decoded, dict) else {}, raw, observed

    @staticmethod
    def _prediction(decoded: dict) -> dict | None:
        telemetry = decoded.get("telemetry", {})
        flags = telemetry.get("flags", {})
        checks = decoded.get("geometryChecks", {})
        distortion = decoded.get("distortionPhysical", {})
        direction = decoded.get("direction", {})
        if not isinstance(telemetry, dict) or not isinstance(flags, dict):
            return None
        if (flags.get("loaded") is not True or flags.get("enabled") is not True or
                flags.get("k_valid") is not True or flags.get("nn_error") != 0 or
                flags.get("protocol_error") is not False or flags.get("fault_latched") is not False or
                flags.get("saturated") is not False or checks.get("valid") is not True):
            return None
        crc = str(telemetry.get("computedCrc32", "")).lower().removeprefix("0x")
        expected = str(telemetry.get("expectedCrc32", "")).lower().removeprefix("0x")
        if crc != EXPECTED_CRC32 or expected != EXPECTED_CRC32:
            return None
        try:
            count = int(telemetry["predictionCount"]) & 0xffffffff
            frame = int(telemetry["predictionFrame"]) & 0xffffffff
            angle = float(direction["angleDegrees"])
            coeffs = {key: float(distortion[key]) for key in ("k1", "k2", "p1", "p2")}
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        bounds = {"k1": .55, "k2": .35, "p1": .006, "p2": .006}
        if count == 0 or not math.isfinite(angle) or not all(
                math.isfinite(coeffs[key]) and abs(coeffs[key]) <= bounds[key] for key in bounds):
            return None
        # A missing raw packet is allowed for pure unit-level policy use; the
        # production transaction requires it before generating bound evidence.
        return {"count": count, "frame": frame, "angle": _norm_angle(angle),
                "coefficients": coeffs, "crc": crc,
                "model_crc32": crc}

    def observe(self, cnn: dict, now: float | None = None) -> dict | None:
        if not self.enabled:
            return None
        now = time.monotonic() if now is None else float(now)
        if not math.isfinite(now):
            self._clear_history()
            return None
        decoded, raw, observed = self._decoded(cnn)
        sample = self._prediction(decoded)
        if sample is None:
            self._clear_history()
            return None
        observed = now if observed is None else observed
        if (not math.isfinite(observed) or now - observed < 0 or now - observed > MAX_AGE_SECONDS):
            self._clear_history()
            return None
        if self._last_observed is not None and observed <= self._last_observed:
            return None
        if self._last_crc is not None and sample["crc"] != self._last_crc:
            self._clear_history()
        if self._last_count is not None and not _counter_advanced(sample["count"], self._last_count):
            # A duplicate packet is ignored. A backward/reset count clears the
            # stability window and starts again from this distinct sample.
            if sample["count"] == self._last_count:
                return None
            self._history.clear()
            self._pending = None
        sample["observed_at"] = observed
        sample["raw_a7_hex"] = raw.hex() if raw is not None else None
        self._history.append(sample)
        self._history = self._history[-3:]
        self._last_count = sample["count"]
        self._last_crc = sample["crc"]
        self._last_observed = observed
        if len(self._history) < 3 or self._pending is not None:
            return None
        angles = [self._history[0]["angle"]]
        for item in self._history[1:]:
            angles.append(_unwrap_near(item["angle"], angles[-1]))
        if max(angles) - min(angles) > 3.0:
            self._history = [sample]
            return None
        for key, tolerance in (("k1", .03), ("k2", .03), ("p1", .002), ("p2", .002)):
            values = [item["coefficients"][key] for item in self._history]
            if max(values) - min(values) > tolerance:
                self._history = [sample]
                return None
        if self._last_applied_at is not None and now - self._last_applied_at < MIN_UPDATE_SECONDS:
            return None
        latest = self._history[-1]
        target = {"angle": -latest["angle"], **latest["coefficients"]}
        if self._last_target is not None:
            previous = self._last_target
            if (abs(math.remainder(target["angle"] - previous["angle"], 360.0)) < .25 and
                    abs(target["k1"] - previous["k1"]) < .002 and
                    abs(target["k2"] - previous["k2"]) < .002 and
                    abs(target["p1"] - previous["p1"]) < .0001 and
                    abs(target["p2"] - previous["p2"]) < .0001):
                return None
        proposal = {
            "prediction_count": latest["count"], "prediction_frame": latest["frame"],
            "observed_at": latest["observed_at"], "raw_a7_hex": latest["raw_a7_hex"],
            "head_angle_deg": latest["angle"], "target_angle_deg": target["angle"],
            "distortion_physical": dict(latest["coefficients"]),
            "crc32": latest["crc"],
        }
        self._last_target = target
        self._pending = proposal
        return proposal

    def applied(self, proposal: dict, now: float | None = None) -> None:
        if proposal != self._pending:
            raise ValueError("Applied proposal does not match the outstanding latest target")
        self._last_applied_at = time.monotonic() if now is None else float(now)
        self._pending = None


def _load_r14_guard():
    spec = importlib.util.spec_from_file_location("_auto_r14_nn_board_observe", NN_BOARD)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load current R14 NN guard: {NN_BOARD}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_nn_host():
    spec = importlib.util.spec_from_file_location("_auto_r14_nn_host", NN_HOST)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load current R14 NN decoder/model manifest: {NN_HOST}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _model_identity(host_module) -> tuple[dict, Path, Path]:
    manifest_path = NN_ROOT / "data" / "model_memory.json"
    weights_path = NN_ROOT / "data" / "weights_le_padded.bin"
    manifest, padded = host_module._verify_model_files()
    actual_padded_sha = hashlib.sha256(padded).hexdigest()
    actual_weights_sha = hashlib.sha256(padded[:int(manifest["weightsBytes"])]).hexdigest()
    actual_crc = f"{int(manifest['weightsCrc32']) & 0xffffffff:08x}"
    if actual_crc != EXPECTED_CRC32:
        raise ValueError("Current CNN weights CRC differs from the expected A7 model")
    identity = {"expectedCrc32": actual_crc,
                "weightsSha256": actual_weights_sha,
                "paddedWeightsSha256": actual_padded_sha,
                "manifestSha256": _sha(manifest_path)}
    return identity, manifest_path, weights_path


def _load_frozen_uart():
    path = R8_TOOLS / "algorithm_uart.py"
    if _sha(path) != FROZEN_UART_SHA256:
        raise RuntimeError("Frozen UART helper SHA-256 mismatch")
    if (_sha(FROZEN_DECODER) != FROZEN_DECODER_SHA256 or
            _sha(FROZEN_PACKET_BUILDER) != FROZEN_PACKET_BUILDER_SHA256):
        raise RuntimeError("Frozen UART helper dependency SHA-256 mismatch")
    spec = importlib.util.spec_from_file_location("_auto_verified_uart_helpers", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load frozen UART helper: {path}")
    module = importlib.util.module_from_spec(spec)
    names = ("decode_algorithm_packet", "make_uart_packet")
    prior_modules = {name: sys.modules.pop(name, None) for name in names}
    prior_path = list(sys.path)
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path[:] = prior_path
        for name in names:
            sys.modules.pop(name, None)
            if prior_modules[name] is not None:
                sys.modules[name] = prior_modules[name]
    return module, path


def _build_predicted_config(proposal: dict, profile_path: Path, output: Path,
                            crop: bool, acceptance_path: Path, session_path: Path,
                            progress=None, controller_source: Path | None = None,
                            binding_validator=None) -> dict:
    import fast_geometry as geometry
    from sw.reference_common.camera_profile import load_camera_profile
    from sw.reference_common.geometry import RemapConfig
    from sw.reference_common.edge_processing import EdgeConfig, BorderMode, edge_config_words, pack_edge_config
    from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES

    acceptance, session = _validate_control_binding(acceptance_path, session_path,
                                                    binding_validator)
    accepted_bit_sha = acceptance.get("bitstreamSha256")
    host = _load_nn_host()
    model_identity, model_manifest_path, model_weights_path = _model_identity(host)
    if _sha(profile_path) != geometry.PROFILE_SHA:
        raise ValueError("Measured native camera profile changed from the frozen V3 profile")
    profile = load_camera_profile(profile_path)
    coefficients = proposal["distortion_physical"]
    for key, bound in (("k1", .55), ("k2", .35), ("p1", .006), ("p2", .006)):
        value = float(coefficients[key])
        if not math.isfinite(value) or abs(value) > bound:
            raise ValueError(f"Predicted {key} exceeds its physical bound")
        profile = replace(profile, **{key: value})
    angle = float(proposal["target_angle_deg"])
    if not math.isfinite(angle) or abs(angle) > 180.0:
        raise ValueError("Compensation angle must be finite and within ±180 degrees")
    scale, scale_report = geometry.choose_scale(profile, angle) if crop else (1.0, {
        "method": "full-view identity scale; no crop search", "inverseScale": 1.0,
        "zoom": 1.0, "floatTargetMarginPx": None, "idealCanvasLinearFraction": 1.0,
        "idealCanvasAreaFraction": 1.0, "globallyMaximumFieldOfViewProven": False})
    config = EdgeConfig(RemapConfig(profile, geometry.affine_for(angle, scale)), BorderMode.CLAMP, 200, 0)
    words = edge_config_words(config)
    output.mkdir(parents=True, exist_ok=False)
    predicted_profile = {**profile.to_mapping(),
                         "coefficientSource": "CNN prediction; not measured calibration",
                         "intrinsicsSource": "measured native 1920x1080 camera profile",
                         "measuredNativeProfileSha256": _sha(profile_path)}
    geometry.write_json(output / "predicted_profile.json", predicted_profile)
    ranges, crop_report = _scan_predicted_geometry(config.remap, crop, progress)
    crop_report.update(angle=angle, profileSha256=_sha(profile_path), bitSha256=accepted_bit_sha,
                       sendHelperSha256=geometry.HELPER_SHA, provenance="CNN predicted Brown coefficients + measured native K",
                       coefficientsSource="CNN prediction; not measured calibration", **scale_report)
    geometry.write_json(output / "range_report.json", ranges)
    geometry.write_json(output / "crop_report.json", crop_report)
    if not ranges.get("full_frame_widths_fit") or crop_report.get("passed") is not True:
        raise ValueError("Predicted configuration failed full 1920x1080 range/fetch audit")
    payload = pack_edge_config(config).to_bytes(92, "little")
    (output / "config_payload.bin").write_bytes(payload)
    (output / "config_payload.hex").write_text(f"{int.from_bytes(payload, 'little'):0184x}\n", encoding="ascii")
    names = (*CONFIG_WORD_NAMES, "border_mode", "edge_threshold", "edge_mode")
    config_doc = {"payload_version": "0.4", "payload_bytes": 92, "byte_order": "little",
                  "coordinate_width": 26, "coordinate_frac_bits": 10, "rtl_max_width": geometry.WIDTH,
                  "profile": profile.to_mapping(), "inverse_affine": config.remap.affine.__dict__,
                  "output_width": geometry.WIDTH, "output_height": geometry.HEIGHT,
                  "crop_enabled": bool(crop), "words": dict(zip(names, words)),
                  "generator": "rotation_dashboard_v4.auto_correction",
                  "qualification": "Full native raster integer width audit; model predictions are not measured calibration"}
    geometry.write_json(output / "config.json", config_doc)
    refs = geometry.reference_hashes()
    dependencies = [Path(__file__), HERE / "fast_geometry.py", HERE / "fast_audit.py",
                    HERE / "content_hash_batch.py", HERE / "fast_sender.py", NN_BOARD, NN_HOST,
                    R14_SOURCE_MANIFEST, model_manifest_path, model_weights_path,
                    FROZEN_DECODER, FROZEN_PACKET_BUILDER]
    if controller_source is not None:
        dependencies.append(Path(controller_source).resolve(strict=True))
    frozen_uart, uart_path = _load_frozen_uart()
    del frozen_uart
    dependencies.append(uart_path)
    dependency_hashes = {str(p.resolve()): _sha(p) for p in dependencies}
    raw_hex = proposal.get("raw_a7_hex")
    if not isinstance(raw_hex, str) or not raw_hex:
        raise ValueError("Raw A7 prediction packet is required for provenance")
    raw = bytes.fromhex(raw_hex)
    sys.path.insert(0, str(NN_BOARD.parent))
    try:
        parsed = host.decode_prediction_packet(raw)
        if parsed["telemetry"]["computedCrc32"] != EXPECTED_CRC32 or not parsed["geometryChecks"]["valid"]:
            raise ValueError("Raw A7 packet no longer validates as a ready geometry prediction")
        telem = parsed["telemetry"]
        predicted_angle = float(parsed["direction"]["angleDegrees"])
        predicted_coeffs = parsed["distortionPhysical"]
        if (int(telem["predictionCount"]) != int(proposal["prediction_count"]) or
                int(telem["predictionFrame"]) != int(proposal["prediction_frame"]) or
                abs(math.remainder(predicted_angle - float(proposal["head_angle_deg"]), 360.0)) > 1e-8 or
                abs(float(proposal["target_angle_deg"]) + predicted_angle) > 1e-8 or
                any(abs(float(predicted_coeffs[key]) - float(coefficients[key])) > 1e-9
                    for key in ("k1", "k2", "p1", "p2"))):
            raise ValueError("Proposal values do not match its raw A7 prediction packet")
    finally:
        try: sys.path.remove(str(NN_BOARD.parent))
        except ValueError: pass
    (output / "raw_a7.bin").write_bytes(raw)
    (output / "prediction.json").write_text(json.dumps(proposal, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    range_sha = _sha(output / "range_report.json")
    crop_sha = _sha(output / "crop_report.json")
    payload_sha = _sha(output / "config_payload.bin")
    bound = {**refs, **dependency_hashes,
             str(profile_path.resolve()): _sha(profile_path),
             str(acceptance_path.resolve()): _sha(acceptance_path),
             str(session_path.resolve()): _sha(session_path),
             str(output / "config.json"): _sha(output / "config.json"),
             str(output / "config_payload.bin"): payload_sha,
             str(output / "range_report.json"): range_sha,
             str(output / "crop_report.json"): crop_sha,
             str(output / "raw_a7.bin"): _sha(output / "raw_a7.bin"),
             str(output / "prediction.json"): _sha(output / "prediction.json"),
             str(output / "predicted_profile.json"): _sha(output / "predicted_profile.json")}
    geometry.write_json(output / "binding.json", bound)
    binding_doc = {
        "qualificationScope": acceptance.get("qualificationScope", "qualified_program_context"),
        "candidate": acceptance.get("candidate"), "acceptedBitstreamSha256": accepted_bit_sha,
        "model": model_identity,
        "acceptanceSha256": _sha(acceptance_path), "programSessionSha256": _sha(session_path),
        "cropEnabled": bool(crop), "predictionCount": proposal["prediction_count"],
        "predictionFrame": proposal["prediction_frame"],
        "rawA7Sha256": _sha(output / "raw_a7.bin"), "bindingSha256": _sha(output / "binding.json"),
        "rangeReportSha256": range_sha, "cropReportSha256": crop_sha,
        "configPayloadSha256": payload_sha}
    if acceptance.get("qualificationScope") == "live_protocol_only":
        binding_doc.pop("acceptanceSha256", None)
        binding_doc.pop("programSessionSha256", None)
        binding_doc["runtimeConnectionId"] = session.get("connectionId")
        binding_doc["runtimeBindingSha256"] = _sha(acceptance_path)
        binding_doc["runtimeConnectionSha256"] = _sha(session_path)
    else:
        binding_doc["programSessionId"] = session.get("sessionId")
    binding_name = ("live_control_context.json" if
                    acceptance.get("qualificationScope") == "live_protocol_only"
                    else "qualified_binding.json")
    geometry.write_json(output / binding_name, binding_doc)
    # Recheck all live dependencies after the expensive 2,073,600-pixel audit.
    for name, digest in {**refs, **dependency_hashes}.items():
        if _sha(Path(name)) != digest:
            raise ValueError(f"Source/reference changed during geometry audit: {name}")
    _validate_control_binding(acceptance_path, session_path, binding_validator)
    if any(_sha(Path(name)) != digest for name, digest in bound.items()):
        raise ValueError("Generated config binding changed before transaction")
    return {"config_dir": str(output), "accepted_bitstream_sha256": accepted_bit_sha,
            "qualification_scope": acceptance.get("qualificationScope", "qualified_program_context"),
            "model_weights_sha256": model_identity["weightsSha256"],
            "model_padded_weights_sha256": model_identity["paddedWeightsSha256"],
            "model_crc32": model_identity["expectedCrc32"],
            "binding_sha256": _sha(output / "binding.json"), "range_report_sha256": range_sha,
            "crop_report_sha256": crop_sha, "raw_a7_sha256": _sha(output / "raw_a7.bin"),
            "inverse_scale": scale}


def _validated_transaction(config_dir: Path, controller, acceptance_path: Path,
                           session_path: Path, port: str, output_root: Path,
                           fake_io=None, cancel_check=None, observed_at=None,
                           allow_unknown_initial=False, binding_validator=None) -> dict:
    helper, helper_path = _load_frozen_uart()
    binding = _read_json(config_dir / "binding.json")
    for name, digest in binding.items():
        if _sha(Path(name)) != digest:
            raise ValueError(f"Bound source/config changed: {name}")
    acceptance, session = _validate_control_binding(acceptance_path, session_path,
                                                     binding_validator)
    if (binding.get(str(acceptance_path.resolve())) != _sha(acceptance_path) or
            binding.get(str(session_path.resolve())) != _sha(session_path)):
        raise ValueError("Current R14 acceptance/session differs from audited binding")
    payload = (config_dir / "config_payload.bin").read_bytes()
    helper.validate_native_payload(payload)
    crop_report = json.loads((config_dir / "crop_report.json").read_text(encoding="utf-8"))
    committed_angle = float(crop_report["angle"])
    if not math.isfinite(committed_angle):
        raise ValueError("Bound crop report has a nonfinite angle")
    precomputed_provenance = {
        "candidate": acceptance.get("candidate"),
        "accepted_bitstream_sha256": acceptance.get("bitstreamSha256"),
        "qualification_scope": acceptance.get("qualificationScope", "qualified_program_context"),
        "helper_sha256": _sha(helper_path),
        "binding_sha256": _sha(config_dir / "binding.json"),
        "config_payload_sha256": _sha(config_dir / "config_payload.bin"),
    }
    if acceptance.get("qualificationScope") == "live_protocol_only":
        precomputed_provenance["runtime_connection_id"] = session.get("connectionId")
        precomputed_provenance["runtime_connection_sha256"] = _sha(session_path)
    else:
        precomputed_provenance["program_session_id"] = session.get("sessionId")
    known_before = bool(getattr(controller, "last_applied", None))
    expected_ack = dict(getattr(controller, "last_applied", {}) or {}) if known_before else None
    if not (allow_unknown_initial and not known_before):
        _require_controller_unchanged(controller)
    output_root.mkdir(parents=True, exist_ok=False)
    packet_path = output_root / "sent_packet.bin"
    # Sequence is unique against current fresh diagnostics. The R8 helper's
    # packet/query/confirmation primitives are used only after exact SHA pinning.
    serial_module = None
    uart = None
    uncertain = False
    try:
        if fake_io is None:
            import serial as serial_module
            uart = serial_module.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=2)
            uart.port, uart.dtr, uart.rts = port, False, False
        else:
            uart = fake_io["serial_factory"]()
        uart.open()
        baseline_packet, baseline_decoded = (fake_io["query"](uart) if fake_io else helper.query(uart))
        before = baseline_decoded["algorithmConfig"]
        if expected_ack is not None and any(
                int(before[key]) != int(expected_ack[key])
                for key in ("receivedSequence", "appliedSequence", "activeVersion")):
            _clear_controller_ack_state(controller)
            raise AutoCorrectionError("Fresh A5 baseline differs from the controller's last ACK; angle state is unknown",
                                      uncertain=False, ownership_lost=True)
        if cancel_check is not None and not cancel_check():
            raise AutoCorrectionError("Automatic target canceled before send", uncertain=False)
        forbidden = {int(before["receivedSequence"]) & 0xffffffff, int(before["appliedSequence"]) & 0xffffffff}
        sequence = int.from_bytes(os.urandom(4), "little")
        while sequence in forbidden:
            sequence = (sequence + 1) & 0xffffffff
        wire = (fake_io["packet"](payload, sequence) if fake_io else helper.packet(payload, sequence))
        packet_path.write_bytes(wire)
        if observed_at is not None:
            _require_fresh_prediction({"observed_at": observed_at})
        if cancel_check is not None and not cancel_check():
            raise AutoCorrectionError("Automatic target canceled immediately before send", uncertain=False)
        uncertain = True
        uart.write(wire)
        if hasattr(uart, "flush"):
            uart.flush()
        start = time.monotonic()
        applied = None
        applied_packet = None
        applied_diagnostic = None
        while time.monotonic() - start < 15:
            diagnostic_packet, diagnostic = (fake_io["query"](uart) if fake_io else helper.query(uart))
            state = diagnostic["algorithmConfig"]
            confirmed = (fake_io["application_confirmed"](before, state, sequence) if fake_io
                         else helper.application_confirmed(before, state, sequence))
            if confirmed:
                applied = state
                applied_packet = diagnostic_packet
                applied_diagnostic = diagnostic
                break
            time.sleep(.01 if fake_io else .25)
        if applied is None:
            raise TimeoutError("No fresh sequence/version/count ACK for geometry update")
        committed_ack = {k: int(applied[k]) for k in ("receivedSequence", "appliedSequence", "activeVersion")}
        controller.current_angle = committed_angle
        controller.last_applied = committed_ack
        crop_enabled = bool(crop_report.get("cropEnabled", False))
        crop_warning = None
        record_auto = getattr(controller, "record_auto_applied", None)
        try:
            if callable(record_auto):
                record_auto(committed_angle, committed_ack, config_dir, crop_report)
            else:
                controller._prepared_config = config_dir
                controller.crop_enabled = crop_enabled
                controller.crop_applied = crop_report if crop_enabled else None
        except Exception:
            try:
                controller._prepared_config = config_dir
                controller.crop_enabled = crop_enabled
                controller.crop_applied = crop_report if crop_enabled else None
            except Exception as exc:
                crop_warning = f"ACK succeeded; crop state could not be updated: {exc!r}"
        uncertain = False
        provenance = precomputed_provenance
        if crop_warning:
            provenance["cropStateWarning"] = crop_warning
        return {"status": "APPLIED", "applied": dict(applied), "sequence": sequence,
                "baseline_a5_hex": bytes(baseline_packet).hex(),
                "baseline_diagnostic": baseline_decoded,
                "ack_a5_hex": bytes(applied_packet).hex(),
                "ack_diagnostic": applied_diagnostic,
                "provenance": provenance}
    except BaseException as exc:
        # Once bytes may have been sent, do not claim rollback or retain stale
        # ownership. A failure during preflight leaves confirmed state untouched.
        if uncertain:
            _clear_controller_ack_state(controller)
        raise AutoCorrectionError(str(exc), uncertain=uncertain, cause=exc,
                                  ownership_lost=bool(getattr(exc, "ownership_lost", False))) from exc
    finally:
        if uart is not None and getattr(uart, "is_open", False):
            try:
                uart.close()
            except Exception:
                pass


def apply_proposal(controller, proposal: dict, acceptance_path: Path,
                   session_path: Path, profile_path: Path, output_root: Path,
                   crop: bool = False, progress=None, *, _fake_io=None,
                   cancel_check=None, binding_validator=None) -> dict:
    """Audit and send one proposal using the current R14 acceptance/session.

    `_fake_io` exists solely for offline transaction tests and is never used by
    the V4 application. Hardware access remains owned by the calling worker.
    """
    acceptance_path, session_path = Path(acceptance_path).resolve(), Path(session_path).resolve()
    profile_path, output_root = Path(profile_path).resolve(), Path(output_root).resolve()
    if not output_root.as_posix().isascii():
        raise ValueError("ASCII output path required by the frozen UART tools")
    if not isinstance(proposal, dict) or proposal.get("crc32") != EXPECTED_CRC32:
        raise ValueError("Proposal is not bound to the expected CNN model CRC")
    _require_fresh_prediction(proposal)
    _require_controller_unchanged(controller)
    _validate_control_binding(acceptance_path, session_path, binding_validator)
    target = output_root / ("auto_" + uuid.uuid4().hex)
    audit_provenance = _build_predicted_config(proposal, profile_path, target, bool(crop),
                                               acceptance_path, session_path, progress,
                                               controller_source=_controller_source(controller),
                                               binding_validator=binding_validator)
    _require_fresh_prediction(proposal)
    if cancel_check is not None and not cancel_check():
        raise AutoCorrectionError("Automatic target canceled after geometry audit", uncertain=False)
    # Required second ownership check after the full mapping audit.
    _require_controller_unchanged(controller)
    port = str(getattr(controller, "port", "COM14"))
    result = _validated_transaction(target, controller, acceptance_path, session_path,
                                    port, target / "transaction", fake_io=_fake_io,
                                    cancel_check=cancel_check,
                                    observed_at=proposal["observed_at"],
                                    binding_validator=binding_validator)
    result["provenance"].update(audit_provenance)
    result["provenance"].update({"prediction_count": proposal["prediction_count"],
                                 "prediction_frame": proposal["prediction_frame"],
                                 "raw_a7_sha256": audit_provenance["raw_a7_sha256"],
                                 "head_angle_deg": proposal["head_angle_deg"],
                                 "target_angle_deg": proposal["target_angle_deg"],
                                 "distortion_physical": proposal["distortion_physical"]})
    _persist_transaction_result(target, result)
    return result


def apply_manual_angle(controller, angle_deg: float, acceptance_path: Path,
                       session_path: Path, profile_path: Path, output_root: Path,
                       crop: bool = False, progress=None,
                       allow_unknown_initial: bool = True, *, _fake_io=None,
                       binding_validator=None) -> dict:
    """Apply a manual angle using strict R14 by default or explicit live binding.

    The first user-entered numeric angle may establish local ownership from a
    fresh native A5 baseline. Every later operation requires the controller's
    last ACK to remain unchanged.
    """
    import fast_geometry as geometry
    from sw.reference_common.camera_profile import load_camera_profile
    from sw.reference_common.geometry import RemapConfig
    from sw.reference_common.edge_processing import EdgeConfig, BorderMode, edge_config_words, pack_edge_config
    from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES

    acceptance_path, session_path = Path(acceptance_path).resolve(), Path(session_path).resolve()
    profile_path, output_root = Path(profile_path).resolve(), Path(output_root).resolve()
    if not output_root.as_posix().isascii():
        raise ValueError("ASCII output path required by the frozen UART tools")
    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError("Manual angle must be finite")
    angle %= 360.0
    acceptance, session = _validate_control_binding(acceptance_path, session_path,
                                                    binding_validator)
    is_known = bool(getattr(controller, "last_applied", None))
    if is_known:
        _require_controller_unchanged(controller)
    elif not allow_unknown_initial:
        raise AutoCorrectionError("No ACK-confirmed angle is known", uncertain=False, ownership_lost=True)
    if _sha(profile_path) != geometry.PROFILE_SHA:
        raise ValueError("Measured native camera profile changed from the frozen V3 profile")
    profile = load_camera_profile(profile_path)
    scale, scale_report = geometry.choose_scale(profile, angle) if crop else (1.0, {
        "method": "full-view identity scale; no crop search", "inverseScale": 1.0,
        "zoom": 1.0, "floatTargetMarginPx": None, "idealCanvasLinearFraction": 1.0,
        "idealCanvasAreaFraction": 1.0, "globallyMaximumFieldOfViewProven": False})
    config = EdgeConfig(RemapConfig(profile, geometry.affine_for(angle, scale)), BorderMode.CLAMP, 200, 0)
    words = edge_config_words(config)
    target = output_root / ("manual_" + uuid.uuid4().hex)
    target.mkdir(parents=True, exist_ok=False)
    geometry.write_json(target / "measured_profile.json", {
        "sourceProfileSha256": _sha(profile_path),
        "intrinsicsAndDistortionSource": "measured native camera profile",
        "profile": profile.to_mapping()})
    ranges, crop_report = geometry.scan_crop(config.remap, bool(crop), progress=(
        (lambda done, total: progress(f"V4 full-frame audit {done:,}/{total:,} pixels"))
        if progress is not None else None))
    crop_report.update(angle=angle, profileSha256=_sha(profile_path),
                       bitSha256=acceptance.get("bitstreamSha256"),
                       sendHelperSha256=geometry.HELPER_SHA,
                       provenance="measured Brown calibration + explicit manual angle",
                       coefficientsSource="measured native profile; not CNN predicted", **scale_report)
    geometry.write_json(target / "range_report.json", ranges)
    geometry.write_json(target / "crop_report.json", crop_report)
    if not ranges.get("full_frame_widths_fit") or crop_report.get("passed") is not True:
        raise ValueError("Manual configuration failed full 1920x1080 range/fetch audit")
    payload = pack_edge_config(config).to_bytes(92, "little")
    (target / "config_payload.bin").write_bytes(payload)
    (target / "config_payload.hex").write_text(f"{int.from_bytes(payload, 'little'):0184x}\n", encoding="ascii")
    names = (*CONFIG_WORD_NAMES, "border_mode", "edge_threshold", "edge_mode")
    geometry.write_json(target / "config.json", {
        "payload_version": "0.4", "payload_bytes": 92, "byte_order": "little",
        "coordinate_width": 26, "coordinate_frac_bits": 10, "rtl_max_width": geometry.WIDTH,
        "profile": profile.to_mapping(), "inverse_affine": config.remap.affine.__dict__,
        "output_width": geometry.WIDTH, "output_height": geometry.HEIGHT,
        "crop_enabled": bool(crop), "words": dict(zip(names, words)),
        "generator": "rotation_dashboard_v4.auto_correction.apply_manual_angle",
        "qualification": "Full native raster integer width audit; profile parameters are measured"})
    refs = geometry.reference_hashes()
    frozen_uart, uart_path = _load_frozen_uart()
    del frozen_uart
    dependencies = [Path(__file__), HERE / "fast_geometry.py", HERE / "fast_audit.py",
                    HERE / "content_hash_batch.py", HERE / "fast_sender.py", NN_BOARD, NN_HOST,
                    R14_SOURCE_MANIFEST, FROZEN_DECODER, FROZEN_PACKET_BUILDER, uart_path,
                    _controller_source(controller)]
    dependency_hashes = {str(path.resolve()): _sha(path) for path in dependencies}
    bound = {**refs, **dependency_hashes,
             str(profile_path): _sha(profile_path),
             str(acceptance_path): _sha(acceptance_path), str(session_path): _sha(session_path),
             str(target / "measured_profile.json"): _sha(target / "measured_profile.json"),
             str(target / "config.json"): _sha(target / "config.json"),
             str(target / "config_payload.bin"): _sha(target / "config_payload.bin"),
             str(target / "range_report.json"): _sha(target / "range_report.json"),
             str(target / "crop_report.json"): _sha(target / "crop_report.json")}
    geometry.write_json(target / "manual_target.json", {
        "source": "explicit user angle", "angleDeg": angle, "cropEnabled": bool(crop),
        "qualificationScope": acceptance.get("qualificationScope", "qualified_program_context"),
        "acceptedBitstreamSha256": acceptance.get("bitstreamSha256")})
    bound[str(target / "manual_target.json")] = _sha(target / "manual_target.json")
    geometry.write_json(target / "binding.json", bound)
    binding_doc = {
        "qualificationScope": acceptance.get("qualificationScope", "qualified_program_context"),
        "candidate": acceptance.get("candidate"),
        "acceptedBitstreamSha256": acceptance.get("bitstreamSha256"),
        "acceptanceSha256": _sha(acceptance_path),
        "cropEnabled": bool(crop), "angle": angle,
        "bindingSha256": _sha(target / "binding.json")}
    if acceptance.get("qualificationScope") == "live_protocol_only":
        binding_doc.pop("acceptanceSha256", None)
        binding_doc["runtimeConnectionId"] = session.get("connectionId")
        binding_doc["runtimeBindingSha256"] = _sha(acceptance_path)
        binding_doc["runtimeConnectionSha256"] = _sha(session_path)
    else:
        binding_doc["programSessionId"] = session.get("sessionId")
        binding_doc["programSessionSha256"] = _sha(session_path)
    geometry.write_json(target / ("live_control_context.json" if
                                  acceptance.get("qualificationScope") == "live_protocol_only"
                                  else "qualified_binding.json"), binding_doc)
    _require_controller_unchanged(controller) if is_known else None
    _validate_control_binding(acceptance_path, session_path, binding_validator)
    for name, digest in {**refs, **dependency_hashes}.items():
        if _sha(Path(name)) != digest:
            raise ValueError(f"Source/reference changed during manual mapping audit: {name}")
    if any(_sha(Path(name)) != digest for name, digest in bound.items()):
        raise ValueError("Manual config binding changed before transaction")
    manual_provenance = {
        "manualAngleDeg": angle, "acceptedBitstreamSha256": acceptance.get("bitstreamSha256"),
        "qualificationScope": acceptance.get("qualificationScope", "qualified_program_context"),
        "config_dir": str(target),
        "rangeReportSha256": _sha(target / "range_report.json"),
        "cropReportSha256": _sha(target / "crop_report.json"),
        "measuredProfileSha256": _sha(target / "measured_profile.json"),
        "inverseScale": scale}
    if acceptance.get("qualificationScope") == "live_protocol_only":
        manual_provenance["runtimeConnectionId"] = session.get("connectionId")
    result = _validated_transaction(target, controller, acceptance_path, session_path,
                                    str(getattr(controller, "port", "COM14")),
                                    target / "transaction", fake_io=_fake_io,
                                    allow_unknown_initial=(allow_unknown_initial and not is_known),
                                    binding_validator=binding_validator)
    result["provenance"].update(manual_provenance)
    _persist_transaction_result(target, result)
    return result
