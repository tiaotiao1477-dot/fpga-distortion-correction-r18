"""Prepare SHA-bound R14 configs for paired MTF/SFR measurements.

This module prepares configuration and static full-raster audit artifacts only.
It never sends a packet or claims camera, image-quality, or physical qualification.
"""
from __future__ import annotations

from dataclasses import replace
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
V5 = ROOT / "sw" / "rotation_dashboard_v5"
if str(V5) not in sys.path:
    sys.path.insert(0, str(V5))

import auto_correction
import fast_geometry
from sw.reference_common.camera_profile import load_camera_profile
from sw.reference_common.geometry import RemapConfig
from sw.reference_common.edge_processing import (
    EdgeConfig, BorderMode, edge_config_words, pack_edge_config,
)
from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES

DEFAULT_PROFILE = (ROOT / "data" / "calibration" / "ov5640_user_photos_20261022"
                   / "board_validation_20261023" / "measured_native_profile"
                   / "camera_profile_1080_runtime.json")
BROWN_WORDS = ("k1_q24", "k2_q24", "p1_q24", "p2_q24")
MEASUREMENT_KINDS = {"identity_baseline", "measured0"}
MEASUREMENT_TARGET = {
    "purpose": "paired MTF/SFR image capture",
    "pattern": "5-degree slanted checkerboard",
    "patternSlantDegrees": 5.0,
    "configurationAngleDegrees": 0.0,
    "cropEnabled": False,
    "requiredPair": ["identity_baseline", "measured0"],
    "qualificationClaim": False,
}


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def _validate_live_context(controller):
    validate = getattr(controller, "validate_context", None)
    binding_path = getattr(controller, "binding_path", None)
    connection_path = getattr(controller, "connection_path", None)
    if not callable(validate) or binding_path is None or connection_path is None:
        raise ValueError("An R14 live controller with validate_context and bound context paths is required")
    binding_path, connection_path = Path(binding_path).resolve(strict=True), Path(connection_path).resolve(strict=True)
    context = validate(binding_path, connection_path)
    if (not isinstance(context, tuple) or len(context) != 2 or
            not isinstance(context[0], dict) or not isinstance(context[1], dict)):
        raise ValueError("Controller validate_context did not return live binding and connection records")
    return binding_path, connection_path, context


def _controller_source(controller) -> Path:
    path = Path(inspect.getfile(type(controller))).resolve(strict=True)
    if not path.is_file():
        raise ValueError("Controller source is unavailable for config provenance")
    return path


def _tool_dependencies(controller, binding_path: Path, connection_path: Path,
                       profile_path: Path, live_sources: dict[str, str]) -> dict[str, str]:
    uart, uart_path = auto_correction._load_frozen_uart()
    del uart
    paths = [
        Path(__file__).resolve(),
        V5 / "auto_correction.py", V5 / "fast_geometry.py", V5 / "fast_audit.py",
        V5 / "fast_control.py", V5 / "content_hash_batch.py", V5 / "fast_sender.py",
        V5 / "interactive_control.py", V5.parent / "rotation_console_v1" / "qualified_control.py",
        auto_correction.NN_BOARD, auto_correction.NN_HOST, auto_correction.R14_SOURCE_MANIFEST,
        auto_correction.FROZEN_DECODER, auto_correction.FROZEN_PACKET_BUILDER, uart_path,
        profile_path, binding_path, connection_path, _controller_source(controller),
    ]
    result = fast_geometry.reference_hashes()
    result.update({str(path.resolve(strict=True)): _sha(path) for path in paths})
    for name, digest in live_sources.items():
        path = Path(name).resolve(strict=True)
        if _sha(path) != digest:
            raise ValueError(f"Live R14 binding source changed during geometry preparation: {path}")
        result[str(path)] = digest
    return result


def _model_for_kind(measured, kind: str):
    if kind == "measured0":
        return measured
    if kind == "identity_baseline":
        return replace(measured, k1=0.0, k2=0.0, p1=0.0, p2=0.0)
    raise ValueError("kind must be identity_baseline or measured0")


def prepare_geometry(controller, kind, outdir) -> Path:
    """Prepare an identity-baseline or measured-profile config at angle 0.

    The returned directory is content-bound for ``auto_correction._validated_transaction``.
    This function performs no UART writes. The caller owns any later transaction.
    """
    if not isinstance(kind, str) or kind not in MEASUREMENT_KINDS:
        raise ValueError("kind must be identity_baseline or measured0")
    if not __debug__:
        raise ValueError("Assertions must remain enabled for geometry preparation")

    profile_path = Path(getattr(controller, "profile_path", "")).resolve(strict=True)
    if profile_path != DEFAULT_PROFILE.resolve(strict=True):
        raise ValueError("Only the frozen V5 DEFAULT_PROFILE may define measurement K")
    profile_sha = _sha(profile_path)
    if profile_sha != fast_geometry.PROFILE_SHA:
        raise ValueError("Frozen V5 measured DEFAULT_PROFILE SHA changed")
    measured = load_camera_profile(profile_path)
    if (measured.width, measured.height) != (1920, 1080):
        raise ValueError("Frozen measured profile must describe native 1920x1080")
    effective = _model_for_kind(measured, kind)

    binding_path, connection_path, live_context = _validate_live_context(controller)
    live_source_hashes = live_context[0].get("sourceHashes", {})
    if not isinstance(live_source_hashes, dict):
        raise ValueError("Live runtime binding has no sourceHashes inventory")
    context_hashes_before = {str(binding_path): _sha(binding_path),
                             str(connection_path): _sha(connection_path)}
    dependencies_before = _tool_dependencies(controller, binding_path, connection_path,
                                              profile_path, live_source_hashes)
    if not str(Path(outdir).resolve()).isascii():
        raise ValueError("Output path must be ASCII for the frozen native transaction tool")
    target = Path(outdir).resolve()
    target.mkdir(parents=True, exist_ok=False)

    angle = 0.0
    crop_enabled = False
    affine = fast_geometry.affine_for(angle, 1.0)
    remap = RemapConfig(effective, affine)
    config = EdgeConfig(remap, BorderMode.CLAMP, 200, 0)
    words = edge_config_words(config)
    if len(words) != len(CONFIG_WORD_NAMES) + 3:
        raise AssertionError("Native v0.4 payload word inventory changed")

    # Reuse the exact V5 full-raster integer proof and taps. The proof's bounded
    # Brown domain is a range guard here, not a CNN prediction or qualification.
    range_report, crop_report = fast_geometry.scan_crop(
        remap, crop_enabled, proof_scope="predicted_brown")
    if (range_report.get("count") != fast_geometry.WIDTH * fast_geometry.HEIGHT or
            range_report.get("full_frame_widths_fit") is not True or
            range_report.get("affine_overflow_count") != 0 or
            range_report.get("coordinate_saturation_count") != 0):
        raise ValueError("Full native-raster range audit failed; config cannot be prepared")

    error = {"rmsPx": float(range_report["error_rms_px"]),
             "p95Px": float(range_report["error_p95_px"]),
             "maxPx": float(range_report["error_max_px"])}
    if not all(math.isfinite(value) and value >= 0 for value in error.values()):
        raise ValueError("Full-raster fixed-versus-float error is not finite")
    range_report.update({
        "kind": kind,
        "profileSha256": profile_sha,
        "geometryAngleDegrees": angle,
        "cropEnabled": crop_enabled,
        "proofProvenance": "measured K/intrinsics with zero Brown coefficients; not CNN prediction",
        "proofScopeImplementation": "V5 fast_geometry.scan_crop bounded Brown-coefficient width proof",
        "v5UnderlyingProofScope": "predicted_brown bounds reused solely as a bounded coefficient-domain proof",
        "int64PreflightScope": "measurement_bounded_coefficient_domain",
        "qualificationClaim": False,
    })
    if kind == "identity_baseline":
        range_report["fixedVsFloatIdentityErrorPx"] = error
        range_report["identityDefinition"] = "measured intrinsics, zero k1/k2/p1/p2, zero angle, no crop, same pixel pipeline"
    range_report["qualification"] = "No MTF/SFR, image-quality, sensor, or physical qualification claimed."

    crop_report.update({
        "angle": angle, "cropEnabled": crop_enabled, "kind": kind,
        "profileSha256": profile_sha,
        "provenance": "explicit paired measurement config using frozen V5 measured DEFAULT_PROFILE; not CNN predicted",
        "coefficientsSource": "measured DEFAULT_PROFILE" if kind == "measured0" else "zero-coefficient identity baseline; measured intrinsics retained",
        "measurementTarget": dict(MEASUREMENT_TARGET),
        "qualificationClaim": False,
        "int64PreflightScope": "bounded_coefficient_domain; proof implementation reused predicted_brown bounds",
    })
    crop_report["passed"] = bool(crop_report.get("passed")) and range_report["full_frame_widths_fit"]
    _write_json(target / "range_report.json", range_report)
    _write_json(target / "crop_report.json", crop_report)

    payload = pack_edge_config(config).to_bytes(92, "little")
    (target / "config_payload.bin").write_bytes(payload)
    (target / "config_payload.hex").write_text(f"{int.from_bytes(payload, 'little'):0184x}\n", encoding="ascii")
    words_doc = dict(zip((*CONFIG_WORD_NAMES, "border_mode", "edge_threshold", "edge_mode"), words))
    effective_mapping = effective.to_mapping()
    measured_mapping = measured.to_mapping()
    config_doc = {
        "payload_version": "0.4", "payload_bytes": 92, "byte_order": "little",
        "coordinate_width": 26, "coordinate_frac_bits": 10,
        "rtl_max_width": fast_geometry.WIDTH,
        "kind": kind, "angle": angle, "crop_enabled": crop_enabled,
        "profile": effective_mapping, "measured_profile": measured_mapping,
        "profile_path": str(profile_path), "profile_sha256": profile_sha,
        "inverse_affine": affine.__dict__,
        "output_width": fast_geometry.WIDTH, "output_height": fast_geometry.HEIGHT,
        "words": words_doc,
        "measurementTarget": dict(MEASUREMENT_TARGET),
        "generator": "tools/sfr_measurement/measurement_config.py",
        "provenance": ("measured native V5 DEFAULT_PROFILE at 0 degrees" if kind == "measured0"
                       else "same pixel pipeline and measured intrinsics with k1/k2/p1/p2 set to zero; not a CNN proposal"),
        "qualificationClaim": False,
    }
    _write_json(target / "config.json", config_doc)
    _write_json(target / "measured_profile.json", {
        "path": str(profile_path), "sha256": profile_sha,
        "measuredProfile": measured_mapping, "effectiveProfile": effective_mapping,
        "coefficientSource": "measured profile" if kind == "measured0" else "zeroed only for identity baseline",
        "qualificationClaim": False,
    })
    _write_json(target / "measurement_target.json", {
        **MEASUREMENT_TARGET, "kind": kind, "configAngleDegrees": angle,
        "profileSha256": profile_sha,
        "sourceBindingPath": str(binding_path), "sourceBindingSha256": context_hashes_before[str(binding_path)],
        "connectionPath": str(connection_path), "connectionSha256": context_hashes_before[str(connection_path)],
        "qualificationClaim": False,
    })

    # Fresh live-context check after the expensive full-raster proof. No state
    # transition is initiated here; the later _validated_transaction owns ACK.
    binding_path_after, connection_path_after, context_after = _validate_live_context(controller)
    context_hashes_after = {str(binding_path_after): _sha(binding_path_after),
                            str(connection_path_after): _sha(connection_path_after)}
    if context_hashes_after != context_hashes_before:
        raise ValueError("Live R14 binding/connection documents changed during measurement preparation")
    if context_after[0].get("sourceHashes", {}) != live_source_hashes:
        raise ValueError("Live source binding changed during measurement preparation")
    if _sha(profile_path) != profile_sha:
        raise ValueError("Frozen V5 measured profile changed during geometry preparation")
    dependencies_after = _tool_dependencies(controller, binding_path, connection_path,
                                            profile_path, live_source_hashes)
    if dependencies_before != dependencies_after:
        raise ValueError("Frozen V5 tool/reference/live dependency changed during geometry preparation")
    generated = [target / name for name in (
        "config.json", "config_payload.bin", "config_payload.hex", "range_report.json",
        "crop_report.json", "measured_profile.json", "measurement_target.json")]
    binding = dict(dependencies_before)
    binding.update({str(path.resolve()): _sha(path) for path in generated})
    _write_json(target / "binding.json", binding)
    return target
