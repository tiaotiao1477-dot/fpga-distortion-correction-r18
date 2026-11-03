"""Injected, synchronous measurement benchmark worker; never opens a serial port."""
from __future__ import annotations

import math
import time
from collections.abc import Mapping
from threading import Event
from typing import Callable, Iterable

from .latency_protocol import decode_packet, summarize_records
from .measurement_transport import MeasurementCommands, MeasurementTransport, MeasurementTimeout

R16_CAPABILITY_MARKER = 0x000F0007
R15_CAPABILITY_MARKER = 0x000F0003
DEFAULT_ANGLES_DEGREES = (0.0, 15.0, 45.0, 90.0)


class BenchmarkError(RuntimeError):
    pass


class BenchmarkCancelled(BenchmarkError):
    pass


def _cancelled(cancel_event) -> bool:
    if cancel_event is None:
        return False
    if callable(cancel_event):
        return bool(cancel_event())
    is_set = getattr(cancel_event, "is_set", None)
    return bool(is_set()) if callable(is_set) else bool(cancel_event)


def _check_cancel(cancel_event) -> None:
    if _cancelled(cancel_event):
        raise BenchmarkCancelled("latency benchmark cancelled")


def _angle_state(value, *, source: str) -> dict[str, int]:
    """Validate the injected angle service's actual-state/ACK mapping."""
    if not isinstance(value, Mapping):
        raise BenchmarkError(f"{source} must return a mapping with angle_q16 and cfg_version")
    angle = value.get("angle_q16")
    cfg_version = value.get("cfg_version")
    if isinstance(angle, bool) or not isinstance(angle, int) or not -(1 << 31) <= angle < (1 << 31):
        raise BenchmarkError(f"{source} returned an invalid angle_q16")
    if isinstance(cfg_version, bool) or not isinstance(cfg_version, int) or not 0 <= cfg_version <= 0xFFFF:
        raise BenchmarkError(f"{source} returned an invalid cfg_version")
    return {"angle_q16": angle, "cfg_version": cfg_version}


def _require_angle_ack(value, expected_angle: int, *, source: str) -> dict[str, int]:
    ack = _angle_state(value, source=source)
    if ack["angle_q16"] != expected_angle:
        raise BenchmarkError(
            f"{source} acknowledged angle_q16={ack['angle_q16']}, expected {expected_angle}")
    return ack


def run_latency_benchmark(controller, *, capability_reader: Callable[[object], int],
                           angle_reader: Callable[[object], Mapping] | None = None,
                           angle_setter: Callable[[int], Mapping] | None = None,
                           angles: Iterable[float] = DEFAULT_ANGLES_DEGREES,
                           warmup: int = 30, target: int = 300,
                           gamma_identity: bool | None = None,
                           cancel_event: Event | Callable[[], bool] | None = None,
                           timeout_per_angle: float = 30.0, transport_timeout: float = 1.0,
                           raw_packet_sink: Callable[[bytes], None] | None = None,
                           clock: Callable[[], float] = time.monotonic) -> dict:
    """Collect per-angle records using caller-owned UART and injected capability/status services.

    ``capability_reader`` must read the live existing-controller A7 capabilityWord and
    return its integer value. The worker never treats a C5 echo as version evidence.
    ``angle_reader(controller)`` reads the actual injected distortion angle and
    returns ``{"angle_q16": int, "cfg_version": int}``. ``angle_setter(angle_q16)``
    must synchronously apply the physical angle and return the same mapping as an ACK.
    Every A9 record must report that ACK's cfg_version. The worker never substitutes
    the FPGA measurement-angle label for the injected service's actual angle.
    """
    if not callable(capability_reader):
        raise TypeError("capability_reader must read the controller's current capabilityWord")
    if not callable(angle_setter):
        raise TypeError("angle_setter must apply an angle and return an actual-angle/cfg_version ACK")
    if not callable(angle_reader):
        raise TypeError("angle_reader must read the injected service's actual angle state")
    if not 0 <= warmup <= 10000:
        raise ValueError("warmup must be between 0 and 10000")
    if not 300 <= target <= 1000:
        raise ValueError("target must be between 300 and 1000")
    if timeout_per_angle <= 0 or transport_timeout <= 0:
        raise ValueError("timeouts must be positive")
    angle_q16s = []
    for degrees in angles:
        value = float(degrees)
        if not math.isfinite(value):
            raise ValueError("angle must be finite")
        angle_q16 = round(value * 65536)
        if not -(1 << 31) <= angle_q16 < (1 << 31):
            raise ValueError("angle_q16 must fit in signed int32")
        if angle_q16 in angle_q16s:
            raise ValueError("duplicate benchmark angle")
        angle_q16s.append(angle_q16)
    if not angle_q16s:
        raise ValueError("at least one angle is required")

    marker = capability_reader(controller)
    if marker != R16_CAPABILITY_MARKER:
        raise BenchmarkError(f"R16 capability marker required; controller reported {marker!r}")

    transport = MeasurementTransport(controller, timeout=transport_timeout, clock=clock)
    raw_packets: list[bytes] = []
    records: list[dict] = []

    def capture_packet(packet: bytes) -> None:
        raw_packets.append(bytes(packet))
        if raw_packet_sink is not None:
            raw_packet_sink(bytes(packet))
        records.append(decode_packet(packet))

    commands = MeasurementCommands(transport, ack_timeout=transport_timeout, clock=clock,
                                   packet_sink=capture_packet)
    initial = None
    last_record = None
    measurement_owned = False
    gamma_owned = False
    angle_state_owned = False
    latest_angle_ack = None
    original_actual_angle = None
    expected_measure_seq = None
    expected_measure_angle = None
    expected_measure_run = None
    expected_gamma_seq = None
    expected_epoch = None
    primary_error: BaseException | None = None
    result = None
    try:
        _check_cancel(cancel_event)
        initial = commands.transport_record()
        last_record = initial
        expected_epoch = initial["epoch"]
        original_measurement_angle = initial["angle_q16"]
        original_run = initial["run_active"]
        original_identity = initial["gamma_identity"]
        expected_gamma_mode = initial["gamma_mode"]
        expected_gamma_identity = original_identity
        expected_gamma_seq = initial["gamma_cmd_applied_seq"]
        expected_measure_seq = initial["measurement_cmd_applied_seq"]
        expected_measure_angle = original_measurement_angle
        expected_measure_run = original_run
        original_actual_state = _angle_state(
            angle_reader(controller), source="angle_reader")
        original_actual_angle = original_actual_state["angle_q16"]

        if gamma_identity is not None and bool(gamma_identity) != original_identity:
            gamma_record = commands.set_gamma_identity(bool(gamma_identity), previous=last_record)
            gamma_owned = True
            last_record = gamma_record
            expected_gamma_identity = bool(gamma_identity)
            expected_gamma_mode = last_record["gamma_mode"]
            expected_gamma_seq = last_record["gamma_cmd_applied_seq"]

        summaries = {}
        cfg_versions = {}
        for angle_q16 in angle_q16s:
            _check_cancel(cancel_event)
            angle_ack = _require_angle_ack(angle_setter(angle_q16), angle_q16,
                                           source="angle_setter")
            latest_angle_ack = angle_ack
            angle_state_owned = True
            measure_record = commands.set_measurement(angle_q16, True, previous=last_record)
            measurement_owned = True
            last_record = measure_record
            expected_epoch = last_record["epoch"]
            expected_measure_seq = last_record["measurement_cmd_applied_seq"]
            expected_angle = angle_q16
            expected_measure_angle = angle_q16
            expected_measure_run = True
            expected_cfg_version = angle_ack["cfg_version"]
            cfg_versions[angle_q16] = expected_cfg_version
            angle_deadline = clock() + timeout_per_angle

            while True:
                _check_cancel(cancel_event)
                remaining = angle_deadline - clock()
                if remaining <= 0:
                    raise MeasurementTimeout(f"timed out collecting {target} samples at angle_q16={angle_q16}")
                packet = transport.query(timeout=min(transport_timeout, remaining))
                capture_packet(packet)
                last_record = records[-1]
                row = last_record
                if row["epoch"] != expected_epoch:
                    raise BenchmarkError("configuration epoch changed during measurement; refusing mixed samples")
                if row["measurement_cmd_applied_seq"] != expected_measure_seq:
                    raise BenchmarkError("measurement configuration changed during collection")
                if not row["run_active"]:
                    raise BenchmarkError("measurement run stopped during collection")
                if (row["gamma_cmd_applied_seq"] != expected_gamma_seq
                        or row["gamma_mode"] != expected_gamma_mode
                        or row["gamma_identity"] != expected_gamma_identity
                        or row["gamma_busy"]):
                    raise BenchmarkError("Gamma configuration changed or became busy during collection")
                if row["angle_q16"] != expected_angle:
                    raise BenchmarkError("measurement angle changed during collection")
                # An empty queue still returns a CRC-valid status/ownership
                # snapshot. Its event fields (including cfg_version) are zero
                # by RTL contract; it is not a latency sample. Keep all state
                # guards above active while waiting for a complete frame.
                if not row["record_valid"]:
                    continue
                if row["cfg_version"] != expected_cfg_version:
                    raise BenchmarkError(
                        f"record cfg_version changed: got {row['cfg_version']}, expected {expected_cfg_version}")
                angle_records = [record for record in records
                                 if record["epoch"] == expected_epoch and
                                 record["angle_q16"] == angle_q16]
                try:
                    summary = summarize_records(angle_records, warmup=warmup, target=target)
                except ValueError as exc:
                    raise BenchmarkError(f"invalid measurement record window: {exc}") from exc
                if summary["sample_count"] >= target:
                    summaries[angle_q16] = summary
                    break

        result = {
            "schema": "r16-latency-benchmark-v1",
            "capability_marker": marker,
            "epoch": expected_epoch,
            "warmup": warmup,
            "target_per_angle": target,
            "gamma_mode": expected_gamma_mode,
            "gamma_identity": expected_gamma_identity,
            "cfg_versions_by_angle": cfg_versions,
            "per_angle": summaries,
            "raw_packets": raw_packets,
            "raw_packet_count": len(raw_packets),
        }
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        cleanup_errors = []
        if initial is not None and (measurement_owned or gamma_owned or angle_state_owned):
            ownership_record = None
            try:
                ownership_record = commands.transport_record()
                last_record = ownership_record
            except BaseException as exc:
                cleanup_errors.append(BenchmarkError(
                    f"fresh A9 ownership check failed; restoration writes skipped: {exc}"))

            if ownership_record is not None:
                ownership_reasons = []
                if ownership_record["epoch"] != expected_epoch:
                    ownership_reasons.append("epoch changed")
                if ownership_record["measurement_cmd_applied_seq"] != expected_measure_seq:
                    ownership_reasons.append("measurement sequence changed")
                if ownership_record["gamma_cmd_applied_seq"] != expected_gamma_seq:
                    ownership_reasons.append("gamma sequence changed")
                if ownership_record["gamma_busy"]:
                    ownership_reasons.append("gamma is busy")
                if ownership_record["angle_q16"] != expected_measure_angle:
                    ownership_reasons.append("measurement angle changed")
                if ownership_record["run_active"] is not expected_measure_run:
                    ownership_reasons.append("measurement run state changed")
                if ownership_record["gamma_identity"] is not expected_gamma_identity:
                    ownership_reasons.append("gamma identity changed")
                if ownership_record["gamma_mode"] != expected_gamma_mode:
                    ownership_reasons.append("gamma mode changed")

                if ownership_reasons:
                    cleanup_errors.append(BenchmarkError(
                        "fresh A9 no longer proves benchmark ownership; restoration writes skipped: "
                        + ", ".join(ownership_reasons)))
                else:
                    if measurement_owned:
                        try:
                            last_record = commands.set_measurement(
                                original_measurement_angle, original_run, previous=last_record)
                        except BaseException as exc:
                            cleanup_errors.append(exc)
                    if gamma_owned:
                        try:
                            last_record = commands.set_gamma_identity(original_identity, previous=last_record)
                        except BaseException as exc:
                            cleanup_errors.append(exc)
                    if angle_state_owned and latest_angle_ack is not None:
                        try:
                            current_angle_state = _angle_state(
                                angle_reader(controller), source="angle_reader")
                            if current_angle_state != latest_angle_ack:
                                cleanup_errors.append(BenchmarkError(
                                    "actual angle/cfg_version changed since the last setter ACK; "
                                    "angle restoration skipped"))
                            else:
                                _require_angle_ack(angle_setter(original_actual_angle),
                                                   original_actual_angle,
                                                   source="angle_setter restore")
                        except BaseException as exc:
                            cleanup_errors.append(exc)
        if cleanup_errors:
            message = "benchmark cleanup could not restore owned state: " + "; ".join(str(exc) for exc in cleanup_errors)
            if primary_error is not None:
                primary_error.add_note(message)
            else:
                raise BenchmarkError(message) from cleanup_errors[0]
    result["raw_packet_count"] = len(raw_packets)
    return result
