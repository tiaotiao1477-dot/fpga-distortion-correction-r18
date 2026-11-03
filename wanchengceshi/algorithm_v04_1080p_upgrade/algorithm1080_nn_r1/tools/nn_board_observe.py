#!/usr/bin/env python3
"""Guarded board host for R8+CNN A7 diagnostics and explicit NN setup.

All board actions require a qualified acceptance for this exact candidate and
the matching SRAM program/readback session. The default action is one
diagnostic-only A7 status read. No A5 legacy packets or geometry updates are
issued. Each raw A7 response is appended to a binary evidence file and indexed
with timestamps and session/command context in JSONL.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib
import json
import sys
import time
import zlib
from pathlib import Path
from typing import Any

BASE_DIR = Path(__file__).resolve().parents[1]
HOST_DIR = BASE_DIR / "tools"
sys.path.insert(0, str(HOST_DIR))
import nn_host  # noqa: E402

EXPECTED_CANDIDATE = "algorithm1080_nn_r1"
R8_GUARD_DIR = BASE_DIR.parent / "algorithm1080_rejoin_r8" / "tools"
BAUDRATE = 115200
A7_REQUEST = b"\xa7"
MAX_STATUS_TIMEOUT = 30.0
MAX_COMMAND_TIMEOUT = 900.0


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def validate_program_binding(acceptance_path: Path, session_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Reject old R8 sessions and changed/unqualified R8+CNN evidence."""
    acceptance = read_object(acceptance_path)
    session = read_object(session_path)
    if acceptance.get("passed") is not True or acceptance.get("candidate") != EXPECTED_CANDIDATE:
        raise ValueError(f"acceptance must be passed and candidate={EXPECTED_CANDIDATE!r}")
    for key in ("bitstream", "bitstreamSha256", "mask", "maskSha256", "sourceHashes", "evidenceHashes"):
        if key not in acceptance:
            raise ValueError(f"qualified acceptance missing {key}")
    if not isinstance(acceptance["sourceHashes"], dict) or not acceptance["sourceHashes"]:
        raise ValueError("qualified acceptance must bind candidate source hashes")
    if not isinstance(acceptance["evidenceHashes"], dict) or not acceptance["evidenceHashes"]:
        raise ValueError("qualified acceptance must bind qualification evidence hashes")
    bitstream = Path(acceptance["bitstream"])
    mask = Path(acceptance["mask"])
    if sha256_file(bitstream) != acceptance["bitstreamSha256"]:
        raise ValueError("accepted candidate bitstream SHA-256 changed")
    if sha256_file(mask) != acceptance["maskSha256"]:
        raise ValueError("accepted candidate mask SHA-256 changed")
    for section in ("sourceHashes", "evidenceHashes"):
        for filename, expected in acceptance[section].items():
            if sha256_file(Path(filename)) != expected:
                raise ValueError(f"{section} hash mismatch: {filename}")

    if session.get("candidateBitstreamSha256") != acceptance["bitstreamSha256"]:
        raise ValueError("program session is not bound to the accepted R8+CNN bitstream")
    sys.path.insert(0, str(R8_GUARD_DIR))
    try:
        guard = importlib.import_module("program_session_guard")
        guard.check_session(session, acceptance)
    finally:
        sys.path.remove(str(R8_GUARD_DIR))
        sys.modules.pop("program_session_guard", None)
    return acceptance, session


class Evidence:
    def __init__(self, output: Path, acceptance_path: Path, session_path: Path,
                 acceptance: dict[str, Any], session: dict[str, Any]):
        self.output = output
        output.mkdir(parents=True, exist_ok=False)
        self.raw = (output / "a7_packets.bin").open("xb")
        self.index = (output / "a7_packets.jsonl").open("x", encoding="utf-8", newline="\n")
        self.started_mono = time.monotonic()
        self.context = {
            "candidate": EXPECTED_CANDIDATE,
            "acceptance": str(acceptance_path.resolve()),
            "acceptanceSha256": sha256_file(acceptance_path),
            "programSession": str(session_path.resolve()),
            "programSessionSha256": sha256_file(session_path),
            "programSessionId": session.get("sessionId"),
            "bitstreamSha256": acceptance["bitstreamSha256"],
        }
        self.packet_count = 0

    def save_packet(self, packet: bytes, *, action: str, command: dict[str, Any] | None,
                    request_mono: float, response_mono: float,
                    request_utc: str, response_utc: str, decoded: dict[str, Any] | None,
                    decode_error: str | None = None) -> None:
        offset = self.raw.tell()
        self.raw.write(packet)
        self.raw.flush()
        record = {
            **self.context,
            "ordinal": self.packet_count,
            "rawOffset": offset,
            "rawBytes": len(packet),
            "action": action,
            "command": command,
            "requestUtc": request_utc,
            "responseUtc": response_utc,
            "requestSeconds": request_mono - self.started_mono,
            "responseSeconds": response_mono - self.started_mono,
            "rawHex": packet.hex(),
            "telemetry": decoded["telemetry"] if decoded else None,
            "geometryChecks": decoded["geometryChecks"] if decoded else None,
            "decodeError": decode_error,
            "modelAccuracyQualified": False,
            "automaticR8GeometryUpdate": False,
        }
        self.index.write(json.dumps(record, allow_nan=False) + "\n")
        self.index.flush()
        self.packet_count += 1

    def close(self) -> None:
        self.raw.close()
        self.index.close()


class BoardLink:
    def __init__(self, port: str, evidence: Evidence):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for board actions; install it before use") from exc
        self.serial = serial.Serial(
            port=port, baudrate=BAUDRATE, timeout=0.05, write_timeout=2.0,
            rtscts=False, dsrdtr=False, exclusive=True if sys.platform != "win32" else None,
        )
        self.serial.dtr = False
        self.serial.rts = False
        self.evidence = evidence

    def close(self) -> None:
        self.serial.close()

    def read_a7(self, *, action: str, command: dict[str, Any] | None,
                timeout: float = MAX_STATUS_TIMEOUT) -> dict[str, Any]:
        if not 0 < timeout <= MAX_STATUS_TIMEOUT:
            raise ValueError(f"status timeout must be in (0,{MAX_STATUS_TIMEOUT}] seconds")
        self.serial.reset_input_buffer()
        request_mono = time.monotonic()
        request_utc = utc_now()
        self.serial.write(A7_REQUEST)
        self.serial.flush()
        deadline = request_mono + timeout
        buffer = bytearray()
        packet = None
        while time.monotonic() < deadline:
            buffer.extend(self.serial.read(256))
            offset = buffer.find(nn_host.PREDICTION_HEADER)
            if offset >= 0 and len(buffer) >= offset + nn_host.PREDICTION_PACKET_BYTES:
                packet = bytes(buffer[offset:offset + nn_host.PREDICTION_PACKET_BYTES])
                break
            if offset > 0:
                del buffer[:offset]
        response_mono = time.monotonic()
        response_utc = utc_now()
        if packet is None:
            raise TimeoutError("no complete fresh 135-byte A7 telemetry response")
        try:
            decoded = nn_host.decode_prediction_packet(packet)
        except ValueError as exc:
            self.evidence.save_packet(packet, action=action, command=command,
                                      request_mono=request_mono, response_mono=response_mono,
                                      request_utc=request_utc, response_utc=response_utc,
                                      decoded=None, decode_error=str(exc))
            raise
        self.evidence.save_packet(packet, action=action, command=command,
                                  request_mono=request_mono, response_mono=response_mono,
                                  request_utc=request_utc, response_utc=response_utc,
                                  decoded=decoded)
        return decoded

    @staticmethod
    def check_errors(decoded: dict[str, Any]) -> None:
        telemetry = decoded["telemetry"]
        flags = telemetry["flags"]
        if telemetry["loaderError"]:
            raise RuntimeError(f"loader error {telemetry['loaderError']}; aborting")
        if flags["protocol_error"] or flags["fault_latched"] or flags["nn_error"]:
            raise RuntimeError("FPGA reports protocol, NN, or latched fault; aborting")

    def wait_command(self, *, action: str, command: dict[str, Any], seq: int,
                     expected_loaded_bytes: int | None = None,
                     expected_crc32: int | None = None,
                     require_loaded: bool | None = None,
                     require_k_valid: bool | None = None,
                     require_enabled: bool | None = None,
                     clear_predictions: bool = False,
                     timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while True:
            decoded = self.read_a7(action=action, command=command,
                                   timeout=min(MAX_STATUS_TIMEOUT, max(0.1, deadline - time.monotonic())))
            self.check_errors(decoded)
            telemetry = decoded["telemetry"]
            flags = telemetry["flags"]
            completed = telemetry["lastCompletedSeq"]
            if completed == seq:
                if expected_loaded_bytes is not None and telemetry["loadedBytes"] != expected_loaded_bytes:
                    raise RuntimeError(f"command {seq}: loadedBytes {telemetry['loadedBytes']} != {expected_loaded_bytes}")
                if expected_crc32 is not None and int(telemetry["computedCrc32"], 16) != expected_crc32:
                    raise RuntimeError(f"command {seq}: computed CRC {telemetry['computedCrc32']} != {expected_crc32:08x}")
                if require_loaded is not None and flags["loaded"] is not require_loaded:
                    raise RuntimeError(f"command {seq}: loaded flag did not become {require_loaded}")
                if require_k_valid is not None and flags["k_valid"] is not require_k_valid:
                    raise RuntimeError(f"command {seq}: K-valid flag did not become {require_k_valid}")
                if require_enabled is not None and flags["enabled"] is not require_enabled:
                    raise RuntimeError(f"command {seq}: enabled flag did not become {require_enabled}")
                if clear_predictions and telemetry["predictionCount"] != 0:
                    raise RuntimeError("BEGIN did not clear the prediction counter")
                return decoded
            if completed != ((seq - 1) & 0xFFFFFFFF):
                raise RuntimeError(f"command {seq}: unexpected completed sequence {completed}; aborting")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"command {seq} was not acknowledged within {timeout:g}s")


def require_quiet_idle(decoded: dict[str, Any]) -> None:
    flags = decoded["telemetry"]["flags"]
    if flags["enabled"] or flags["nn_busy"] or flags["capture_busy"] or not flags["cache_idle"]:
        raise RuntimeError("loader/K setup requires disabled NN, idle inference/capture, and idle cache")


def require_expected_model(decoded: dict[str, Any]) -> None:
    manifest, _ = nn_host._verify_model_files()
    telemetry = decoded["telemetry"]
    expected_crc = int(manifest["weightsCrc32"])
    if not telemetry["flags"]["loaded"] or telemetry["loadedBytes"] != nn_host.PADDED_BYTES:
        raise RuntimeError("the complete expected model is not loaded")
    if int(telemetry["expectedCrc32"], 16) != expected_crc:
        raise RuntimeError("FPGA expected CRC does not match this candidate's model")
    if int(telemetry["computedCrc32"], 16) != expected_crc:
        raise RuntimeError("loaded model CRC does not match this candidate's model")


def upload_weights(link: BoardLink, timeout: float, total_timeout: float) -> dict[str, Any]:
    manifest, padded = nn_host._verify_model_files()
    goal_crc = int(manifest["weightsCrc32"])
    initial = link.read_a7(action="pre_upload_status", command=None)
    link.check_errors(initial)
    require_quiet_idle(initial)
    expected = int(initial["telemetry"]["expectedCrc32"], 16)
    if expected != goal_crc:
        raise RuntimeError(f"FPGA EXPECTED_CRC32 {expected:08x} does not match model {goal_crc:08x}")

    upload_deadline = time.monotonic() + total_timeout
    seq = (initial["telemetry"]["lastCompletedSeq"] + 1) & 0xFFFFFFFF
    begin = nn_host.encode_command(1, seq, 0, bytes(nn_host.BLOCK_BYTES))
    link.serial.write(begin)
    link.serial.flush()
    command = {"op": 1, "seq": seq, "address": 0}
    result = link.wait_command(action="begin_ack", command=command, seq=seq,
                               expected_loaded_bytes=0, expected_crc32=0,
                               require_loaded=False, require_k_valid=False,
                               require_enabled=False, clear_predictions=True,
                               timeout=min(timeout, max(0.1, upload_deadline - time.monotonic())))
    print(f"BEGIN acknowledged at seq={seq}; loader and prediction state cleared", flush=True)
    seq = (seq + 1) & 0xFFFFFFFF
    crc = 0

    for offset in range(0, nn_host.PADDED_BYTES, nn_host.BLOCK_BYTES):
        remaining = upload_deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("overall model upload timeout expired; no retry was attempted")
        block = padded[offset:offset + nn_host.BLOCK_BYTES]
        if len(block) != nn_host.BLOCK_BYTES:
            raise ValueError("padded model does not contain a complete final block")
        command_bytes = nn_host.encode_command(2, seq, nn_host.WEIGHTS_BASE + offset, block)
        link.serial.write(command_bytes)
        link.serial.flush()
        payload_count = max(0, min(nn_host.BLOCK_BYTES, nn_host.PAYLOAD_BYTES - offset))
        crc = zlib.crc32(block[:payload_count], crc) & 0xFFFFFFFF
        command = {"op": 2, "seq": seq, "address": nn_host.WEIGHTS_BASE + offset,
                   "offset": offset, "bytes": nn_host.BLOCK_BYTES}
        link.wait_command(action="write_ack", command=command, seq=seq,
                          expected_loaded_bytes=offset + nn_host.BLOCK_BYTES,
                          expected_crc32=crc, require_loaded=False, require_enabled=False,
                          timeout=min(timeout, remaining))
        if (offset // nn_host.BLOCK_BYTES + 1) % 250 == 0 or offset + nn_host.BLOCK_BYTES == nn_host.PADDED_BYTES:
            print(f"uploaded {offset + nn_host.BLOCK_BYTES}/{nn_host.PADDED_BYTES} bytes", flush=True)
        seq = (seq + 1) & 0xFFFFFFFF

    if time.monotonic() >= upload_deadline:
        raise TimeoutError("overall model upload timeout expired before COMMIT")
    command_bytes = nn_host.encode_command(3, seq, 0, bytes(nn_host.BLOCK_BYTES))
    link.serial.write(command_bytes)
    link.serial.flush()
    command = {"op": 3, "seq": seq, "address": 0}
    result = link.wait_command(action="commit_ack", command=command, seq=seq,
                               expected_loaded_bytes=nn_host.PADDED_BYTES,
                               expected_crc32=goal_crc, require_loaded=True,
                               require_enabled=False,
                               timeout=min(timeout, max(0.1, upload_deadline - time.monotonic())))
    return result


def send_k(link: BoardLink, fx: int, fy: int, cx: int, cy: int, timeout: float) -> dict[str, Any]:
    for name, value in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)):
        if not -32768 <= value <= 32767:
            raise ValueError(f"{name} must fit a signed 16-bit FPGA register")
    if fx <= 0 or fy <= 0:
        raise ValueError("fx and fy must be positive signed 16-bit register values")
    before = link.read_a7(action="pre_k_status", command=None)
    link.check_errors(before)
    require_quiet_idle(before)
    require_expected_model(before)
    seq = (before["telemetry"]["lastCompletedSeq"] + 1) & 0xFFFFFFFF
    # Values are raw signed 16-bit register words in little-endian order;
    # this host intentionally applies no scale or calibration conversion.
    import struct
    data = struct.pack("<hhhh", fx, fy, cx, cy) + bytes(24)
    packet = nn_host.encode_command(5, seq, 0, data)
    link.serial.write(packet)
    link.serial.flush()
    return link.wait_command(action="set_k_ack", command={"op": 5, "seq": seq,
        "fx": fx, "fy": fy, "cx": cx, "cy": cy}, seq=seq,
        expected_loaded_bytes=nn_host.PADDED_BYTES, require_loaded=True,
        require_k_valid=True, require_enabled=False, timeout=timeout)


def set_enabled(link: BoardLink, enabled: bool, timeout: float) -> dict[str, Any]:
    before = link.read_a7(action="pre_enable_status", command=None)
    link.check_errors(before)
    if enabled:
        require_expected_model(before)
        if not before["telemetry"]["flags"]["k_valid"]:
            raise RuntimeError("enable requires valid K")
    seq = (before["telemetry"]["lastCompletedSeq"] + 1) & 0xFFFFFFFF
    data = bytes((int(enabled),)) + bytes(31)
    packet = nn_host.encode_command(4, seq, 0, data)
    link.serial.write(packet)
    link.serial.flush()
    return link.wait_command(action="enable_ack" if enabled else "disable_ack",
                             command={"op": 4, "seq": seq, "enabled": enabled}, seq=seq,
                             require_enabled=enabled, timeout=timeout)


def list_ports() -> int:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial is required to enumerate ports") from exc
    for port in list_ports.comports():
        print(json.dumps({"device": port.device, "description": port.description,
                          "hwid": port.hwid}, ensure_ascii=False))
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Guarded R8+CNN A7 diagnostics and explicit model setup")
    p.add_argument("--list-ports", action="store_true", help="read-only serial port enumeration; opens no port")
    p.add_argument("--acceptance", type=Path, help="qualified acceptance JSON for algorithm1080_nn_r1")
    p.add_argument("--session", type=Path, help="matching SRAM program_session.json")
    p.add_argument("--port", help="serial port (for example COM14); never inferred or auto-selected")
    p.add_argument("--output", type=Path, help="new evidence directory; existing directories are refused")
    p.add_argument("--status-count", type=int, default=1, help="bounded read-only A7 packet count (1..100)")
    p.add_argument("--interval", type=float, default=1.0, help="seconds between status reads (0..30)")
    p.add_argument("--upload-weights", action="store_true", help="explicitly BEGIN/WRITE/COMMIT model weights")
    p.add_argument("--set-k", action="store_true", help="send raw little-endian signed-16 fx,fy,cx,cy register words; no scaling")
    p.add_argument("--fx", type=int, help="positive signed 16-bit raw focal register value")
    p.add_argument("--fy", type=int, help="positive signed 16-bit raw focal register value")
    p.add_argument("--cx", type=int, help="signed 16-bit raw principal-point register value")
    p.add_argument("--cy", type=int, help="signed 16-bit raw principal-point register value")
    p.add_argument("--enable-observation", action="store_true", help="explicitly enable prediction observation")
    p.add_argument("--disable-observation", action="store_true", help="explicitly disable prediction observation")
    p.add_argument("--status-timeout", type=float, default=4.0, help="A7 reply timeout in seconds (max 30)")
    p.add_argument("--command-timeout", type=float, default=20.0,
                   help="per-command completion timeout (1..900 seconds; writes are never retried)")
    p.add_argument("--upload-timeout", type=float, default=1800.0,
                   help="overall upload budget in seconds (1..3600; default 30 minutes)")
    return p


def main(argv: list[str] | None = None) -> int:
    p = parser()
    args = p.parse_args(argv)
    try:
        if args.list_ports:
            if any((args.port, args.output, args.acceptance, args.session, args.upload_weights,
                    args.set_k, args.enable_observation, args.disable_observation)):
                p.error("--list-ports cannot be combined with board-action arguments")
            return list_ports()
        if not args.port or not args.acceptance or not args.session or not args.output:
            p.error("board actions require explicit --port, --acceptance, --session, and --output")
        if args.status_count not in range(1, 101) or not 0 <= args.interval <= 30:
            p.error("--status-count must be 1..100 and --interval must be 0..30")
        if not 0 < args.status_timeout <= MAX_STATUS_TIMEOUT:
            p.error(f"--status-timeout must be in (0,{MAX_STATUS_TIMEOUT}]")
        if not 1 <= args.command_timeout <= MAX_COMMAND_TIMEOUT:
            p.error(f"--command-timeout must be 1..{MAX_COMMAND_TIMEOUT:g}")
        if not 1 <= args.upload_timeout <= 3600:
            p.error("--upload-timeout must be 1..3600 seconds")
        if args.enable_observation and args.disable_observation:
            p.error("choose only one of --enable-observation and --disable-observation")
        k_values = (args.fx, args.fy, args.cx, args.cy)
        if args.set_k and any(value is None for value in k_values):
            p.error("--set-k requires --fx, --fy, --cx, and --cy")
        if not args.set_k and any(value is not None for value in k_values):
            p.error("--fx/--fy/--cx/--cy require --set-k")

        acceptance, session = validate_program_binding(args.acceptance, args.session)
        evidence = Evidence(args.output, args.acceptance, args.session, acceptance, session)
        link = None
        result: dict[str, Any] = {
            "status": "RUNNING",
            "candidate": EXPECTED_CANDIDATE,
            "startedUtc": utc_now(),
            "port": args.port,
            "baudrate": BAUDRATE,
            "acceptance": evidence.context["acceptance"],
            "acceptanceSha256": evidence.context["acceptanceSha256"],
            "programSession": evidence.context["programSession"],
            "programSessionSha256": evidence.context["programSessionSha256"],
            "programSessionId": evidence.context["programSessionId"],
            "actions": [],
            "diagnosticOnly": True,
            "modelAccuracyQualified": False,
            "automaticR8GeometryUpdate": False,
        }
        try:
            link = BoardLink(args.port, evidence)
            if args.upload_weights:
                uploaded = upload_weights(link, args.command_timeout, args.upload_timeout)
                result["actions"].append({"action": "upload_weights", "completed": True,
                                          "finalTelemetry": uploaded["telemetry"]})
            if args.set_k:
                configured = send_k(link, *k_values, args.command_timeout)
                result["actions"].append({"action": "set_k", "completed": True,
                                          "finalTelemetry": configured["telemetry"]})
            if args.enable_observation or args.disable_observation:
                configured = set_enabled(link, args.enable_observation, args.command_timeout)
                result["actions"].append({"action": "enable_observation" if args.enable_observation else "disable_observation",
                                          "completed": True, "finalTelemetry": configured["telemetry"]})
            for index in range(args.status_count):
                if index:
                    time.sleep(args.interval)
                decoded = link.read_a7(action="status", command=None, timeout=args.status_timeout)
                link.check_errors(decoded)
                result["actions"].append({"action": "status", "ordinal": index,
                                          "telemetry": decoded["telemetry"],
                                          "geometryChecks": decoded["geometryChecks"]})
            result["status"] = "CAPTURE_COMPLETE"
        except Exception as exc:
            result.update(status="FAILED", error=repr(exc))
            raise
        finally:
            if link is not None:
                link.close()
            evidence.close()
            result.update(endedUtc=utc_now(), rawPacketCount=evidence.packet_count,
                          rawPackets="a7_packets.bin", packetIndex="a7_packets.jsonl")
            (args.output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n",
                                                       encoding="utf-8")
            print(json.dumps({key: result[key] for key in ("status", "rawPacketCount", "error") if key in result},
                             ensure_ascii=False), flush=True)
        return 0
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, TimeoutError, AssertionError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
