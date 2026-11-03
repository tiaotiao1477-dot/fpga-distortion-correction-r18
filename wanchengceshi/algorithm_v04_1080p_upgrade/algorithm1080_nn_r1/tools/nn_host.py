#!/usr/bin/env python3
"""Offline UART helpers for the NN observation/loader protocol.

This tool never opens a serial port. `export-stream` only creates a binary
command stream and a JSON description for a later, separately authorized step.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import zlib
from pathlib import Path
from typing import Any, Iterable

BASE_DIR = Path(__file__).resolve().parents[1]
MEMORY_MANIFEST = BASE_DIR / "data" / "model_memory.json"
PADDED_WEIGHTS = BASE_DIR / "data" / "weights_le_padded.bin"
MODEL_METADATA = BASE_DIR / "vendor" / "nn_v0_1" / "data" / "model.json"
WEIGHTS_BASE = 0x03000000
PADDED_BYTES = 601632
PAYLOAD_BYTES = 601616
BLOCK_BYTES = 32
UART_PACKET_BYTES = 46
PREDICTION_PACKET_BYTES = 135
PREDICTION_HEADER = bytes((0x55, 0xAA, 0xE3, 0x80))
TELEMETRY_MAGIC = 0x4E4E0001
Q20_SCALE = 1 << 20


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return value


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection/xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_command(op: int, seq: int, address: int, data: bytes) -> bytes:
    """Encode one 46-byte command frame; data is exactly 32 payload bytes."""
    if not isinstance(data, bytes) or len(data) != BLOCK_BYTES:
        raise ValueError("command data must be exactly 32 bytes")
    if not 0 <= op <= 0xFF:
        raise ValueError("op must fit in one byte")
    if not 0 <= seq <= 0xFFFFFFFF:
        raise ValueError("seq must fit in uint32")
    if not 0 <= address <= 0xFFFFFFFF:
        raise ValueError("address must fit in uint32")
    body = bytes((0x01, op)) + struct.pack("<II", seq, address) + data
    if len(body) != 42:
        raise AssertionError("internal protocol body size error")
    crc = crc16_ccitt_false(body)
    packet = b"\xC5\x5C" + body + struct.pack("<H", crc)
    if len(packet) != UART_PACKET_BYTES:
        raise AssertionError("internal protocol packet size error")
    return packet


def _signed_i32(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def _cross(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])


def _polygon_area(points: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )


def decode_prediction_packet(packet: bytes) -> dict[str, Any]:
    """Validate and decode an offline 135-byte A7 telemetry packet.

    Malformed framing, checksum, or telemetry magic is rejected with ValueError.
    Geometric/status failures are returned in `validity` and never qualify model
    accuracy or imply that R8 geometry was changed.
    """
    if not isinstance(packet, bytes) or len(packet) != PREDICTION_PACKET_BYTES:
        raise ValueError(f"prediction packet must be exactly {PREDICTION_PACKET_BYTES} bytes")
    if packet[:4] != PREDICTION_HEADER:
        raise ValueError("prediction packet header/status is not 55 AA E3 80")
    payload = packet[4:132]
    checksum = 0
    for byte in payload:
        checksum ^= byte
    if packet[132] != checksum:
        raise ValueError("prediction payload XOR mismatch")
    if packet[133:135] != b"\x0D\x0A":
        raise ValueError("prediction packet is missing CRLF")
    words = list(struct.unpack("<32I", payload))
    if words[0] != TELEMETRY_MAGIC:
        raise ValueError(f"telemetry word 0 magic mismatch: 0x{words[0]:08x}")

    flags = words[1]
    flag_map = {
        "loaded": bool(flags & (1 << 0)),
        "enabled": bool(flags & (1 << 1)),
        "k_valid": bool(flags & (1 << 2)),
        "nn_busy": bool(flags & (1 << 3)),
        "image_valid": bool(flags & (1 << 4)),
        "cache_idle": bool(flags & (1 << 5)),
        "capture_busy": bool(flags & (1 << 6)),
        "saturated": bool(flags & (1 << 7)),
        "nn_error": (flags >> 8) & 0x3,
        "protocol_error": bool(flags & (1 << 10)),
        "fault_latched": bool(flags & (1 << 11)),
    }
    output_words = [_signed_i32(word) for word in words[16:30]]
    normalized = [word / Q20_SCALE for word in output_words]
    d_scales = (0.55, 0.35, 0.006, 0.006)
    distortion = {
        name: normalized[index] * d_scales[index]
        for index, name in enumerate(("k1", "k2", "p1", "p2"))
    }
    corners_norm = [
        (normalized[index], normalized[index + 1])
        for index in range(4, 12, 2)
    ]
    corner_names = ("TL", "TR", "BR", "BL")
    corners = [
        {"name": name, "x": x * 639.0, "y": y * 479.0}
        for name, (x, y) in zip(corner_names, corners_norm)
    ]
    sin_theta, cos_theta = normalized[12], normalized[13]
    direction_length = math.hypot(sin_theta, cos_theta)
    angle_radians = math.atan2(sin_theta, cos_theta)
    points = [(point["x"], point["y"]) for point in corners]
    crosses = [_cross(points[i], points[(i + 1) % 4], points[(i + 2) % 4]) for i in range(4)]
    area = _polygon_area(points)

    numeric_values = normalized + list(distortion.values()) + [
        point[axis] for point in corners for axis in ("x", "y")
    ] + [direction_length, angle_radians, area]
    reasons: list[str] = []
    if not flag_map["loaded"] or not flag_map["k_valid"]:
        reasons.append("model_or_intrinsics_not_ready")
    if words[8] == 0:
        reasons.append("no_completed_prediction")
    if words[5] != words[30]:
        reasons.append("model_crc_mismatch")
    if not all(math.isfinite(value) for value in numeric_values):
        reasons.append("non_finite_value")
    if not all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for x, y in corners_norm):
        reasons.append("corner_outside_640x480_canvas")
    # Screen coordinates use y-down; TL->TR->BR->BL therefore has positive area.
    if not all(value > 0.0 for value in crosses):
        reasons.append("corner_order_or_convexity_invalid")
    if not (1.0 < area <= 639.0 * 479.0):
        reasons.append("corner_area_invalid")
    if not (0.5 <= direction_length <= 1.5):
        reasons.append("direction_length_invalid")
    if flag_map["saturated"]:
        reasons.append("saturated")
    if flag_map["nn_error"] != 0:
        reasons.append("nn_error")
    if flag_map["protocol_error"]:
        reasons.append("protocol_error")
    if flag_map["fault_latched"]:
        reasons.append("fault_latched")
    if words[3] & 0xFF:
        reasons.append("loader_error")

    telemetry = {
        "flags": flag_map,
        "lastCompletedSeq": words[2],
        "loaderError": words[3] & 0xFF,
        "loadedBytes": words[4],
        "computedCrc32": f"{words[5]:08x}",
        "predictionFrame": words[6],
        "predictionCycles": words[7],
        "predictionCount": words[8],
        "captureErrors": words[9],
        "skippedFrames": words[10],
        "imageFrame": words[11],
        "nnCycles": words[12],
        "kPacked": [f"{words[13]:08x}", f"{words[14]:08x}"],
        "geometryMode": words[15],
        "expectedCrc32": f"{words[30]:08x}",
        "reservedWord31": words[31],
    }
    return {
        "packetType": "nn_prediction_observation",
        "statusByte": packet[3],
        "telemetry": telemetry,
        "outputsQ20Signed": output_words,
        "outputsNormalized": normalized,
        "distortionPhysical": distortion,
        "corners640x480": corners,
        "direction": {
            "sin": sin_theta,
            "cos": cos_theta,
            "length": direction_length,
            "angleRadians": angle_radians,
            "angleDegrees": math.degrees(angle_radians),
        },
        "geometryChecks": {
            "signedAreaPixels": area,
            "edgeCrossProducts": crosses,
            "modelAccuracyQualified": False,
            "valid": not reasons,
            "reasons": reasons,
        },
        "modelAccuracyQualified": False,
        "automaticR8GeometryUpdate": False,
    }


def _verify_model_files() -> tuple[dict[str, Any], bytes]:
    manifest = _read_json(MEMORY_MANIFEST)
    padded = PADDED_WEIGHTS.read_bytes()
    required = ("weightsBytes", "paddedBytes", "weightsCrc32", "weightsSha256", "paddedSha256", "weightsBase")
    missing = [key for key in required if key not in manifest]
    if missing:
        raise ValueError(f"model_memory.json missing: {', '.join(missing)}")
    if manifest["weightsBase"] != WEIGHTS_BASE:
        raise ValueError("model weight base is not the fixed NN weight region")
    if manifest["weightsBytes"] != PAYLOAD_BYTES or manifest["paddedBytes"] != PADDED_BYTES:
        raise ValueError("model sizes do not match fixed transfer contract")
    if len(padded) != manifest["paddedBytes"]:
        raise ValueError(f"padded weight length mismatch: {len(padded)}")
    padded_sha = hashlib.sha256(padded).hexdigest()
    if padded_sha.lower() != str(manifest["paddedSha256"]).lower():
        raise ValueError("padded weight SHA-256 mismatch")
    actual = padded[:manifest["weightsBytes"]]
    actual_sha = hashlib.sha256(actual).hexdigest()
    if actual_sha.lower() != str(manifest["weightsSha256"]).lower():
        raise ValueError("unpadded weight SHA-256 mismatch")
    crc = zlib.crc32(actual) & 0xFFFFFFFF
    if crc != manifest["weightsCrc32"]:
        raise ValueError(f"weight CRC-32 mismatch: got {crc:08x}")
    return manifest, padded


def _iter_upload_stream(padded: bytes) -> Iterable[bytes]:
    seq = 1
    yield encode_command(1, seq, 0, bytes(BLOCK_BYTES))
    seq += 1
    for offset in range(0, PADDED_BYTES, BLOCK_BYTES):
        block = padded[offset:offset + BLOCK_BYTES]
        if len(block) != BLOCK_BYTES:
            raise ValueError("padded weight block is not 32 bytes")
        yield encode_command(2, seq, WEIGHTS_BASE + offset, block)
        seq += 1
    yield encode_command(3, seq, 0, bytes(BLOCK_BYTES))


def _default_stream_description_path(output: Path) -> Path:
    return output.with_name(output.name + ".json")


def export_stream(output: Path) -> dict[str, Any]:
    output = output.expanduser()
    description_path = _default_stream_description_path(output)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing stream: {output}")
    if description_path.exists():
        raise FileExistsError(f"refusing to overwrite existing description: {description_path}")
    manifest, padded = _verify_model_files()
    digest = hashlib.sha256()
    stream_bytes = 0
    command_count = 0
    first_seq = 1
    last_seq = 0
    with output.open("xb") as stream_file:
        try:
            for command in _iter_upload_stream(padded):
                stream_file.write(command)
                digest.update(command)
                stream_bytes += len(command)
                command_count += 1
                last_seq = struct.unpack_from("<I", command, 4)[0]
        except Exception:
            stream_file.close()
            output.unlink(missing_ok=True)
            raise
    stream_sha = digest.hexdigest()
    description = {
        "format": "nn-uart-upload-stream-v1",
        "mode": "offline_only_not_transmitted",
        "streamFile": output.name,
        "streamBytes": stream_bytes,
        "streamSha256": stream_sha,
        "commandPacketBytes": UART_PACKET_BYTES,
        "commands": {
            "begin": {"op": 1, "seq": first_seq, "address": 0},
            "writeCount": PADDED_BYTES // BLOCK_BYTES,
            "writeSeqStart": first_seq + 1,
            "writeSeqEnd": last_seq - 1,
            "commit": {"op": 3, "seq": last_seq, "address": 0},
            "total": command_count,
        },
        "expectedModel": {
            "weightsBase": manifest["weightsBase"],
            "weightsBytes": manifest["weightsBytes"],
            "paddedBytes": manifest["paddedBytes"],
            "weightsCrc32": f"{manifest['weightsCrc32']:08x}",
            "weightsSha256": manifest["weightsSha256"],
            "paddedSha256": manifest["paddedSha256"],
        },
        "automaticKOrEnable": False,
        "hardwareDownloadPerformed": False,
        "modelAccuracyQualified": False,
    }
    with description_path.open("x", encoding="utf-8", newline="\n") as desc_file:
        json.dump(description, desc_file, indent=2)
        desc_file.write("\n")
    return description


def inspect_model() -> dict[str, Any]:
    memory = _read_json(MEMORY_MANIFEST)
    model = _read_json(MODEL_METADATA) if MODEL_METADATA.exists() else {}
    summary = {
        "modelMemory": memory,
        "model": {
            "format": model.get("format", "unknown"),
            "sourceCheckpointSha256": model.get("source_checkpoint_sha256"),
            "layerCount": len(model.get("layers", [])),
            "classifier": model.get("classifier"),
            "fpgaImplementationVerified": model.get("FPGA_implementation_verified", False),
        },
        "status": {
            "fpgaImplementationVerified": False,
            "trainingNativeImageToCanvasPreprocessingVerified": False,
            "modelAccuracyQualified": False,
            "automaticR8GeometryUpdate": False,
            "hardwareDownloadPerformed": False,
        },
    }
    print(f"weightsBytes: {memory['weightsBytes']}")
    print(f"paddedBytes: {memory['paddedBytes']}")
    print(f"weightsBase: 0x{memory['weightsBase']:08x}")
    print(f"weightsCrc32: 0x{memory['weightsCrc32']:08x}")
    print(f"weightsSha256: {memory['weightsSha256']}")
    print(f"paddedSha256: {memory['paddedSha256']}")
    print(f"model: {summary['model']['format']} ({summary['model']['layerCount']} layers)")
    print("not verified: FPGA implementation, native-image-to-canvas preprocessing, and model accuracy")
    print("observation stage only: no automatic R8 geometry update; no hardware download")
    return summary


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline NN UART protocol helper; never opens a serial port.")
    sub = parser.add_subparsers(dest="action")
    sub.add_parser("inspect", help="show model memory metadata and verification limits")
    export = sub.add_parser("export-stream", help="write an offline BEGIN/WRITE/COMMIT stream")
    export.add_argument("--output", required=True, type=Path, help="new binary stream path (must not exist)")
    decode = sub.add_parser("decode", help="decode one offline 135-byte A7 packet")
    decode.add_argument("--packet", required=True, type=Path, help="packet file; no serial input is used")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    action = args.action or "inspect"
    try:
        if action == "inspect":
            inspect_model()
        elif action == "export-stream":
            result = export_stream(args.output)
            print(json.dumps(result, indent=2))
            print("offline stream created; it was not transmitted")
        elif action == "decode":
            result = decode_prediction_packet(args.packet.read_bytes())
            print(json.dumps(result, indent=2, allow_nan=False))
        else:
            parser.error(f"unknown action {action}")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
