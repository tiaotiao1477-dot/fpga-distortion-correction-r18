"""Pure-Python decoder and latency statistics for the R16 measurement UART protocol."""
from __future__ import annotations

import math
import struct
from collections import defaultdict
from typing import Iterable, Mapping

HEADER = b"\x55\xaa\xe5\x80"
TRAILER = b"\x0d\x0a"
PAYLOAD_SIZE = 128
PACKET_SIZE = 136
TICK_HZ = 125_000_000
MAGIC = 0x36314D4C
COUNTER_MASK = 0xFFFFFFFF
_WRAP_HIGH = 0xF0000000
_WRAP_LOW = 0x0FFFFFFF
FLAG_RECORD_VALID = 1 << 0
FLAG_RUN_ACTIVE = 1 << 1
FLAG_GAMMA_IDENTITY = 1 << 2
FLAG_TRACKER_INVALID = 1 << 3
FLAG_TELEMETRY_OVERFLOW = 1 << 4
FLAG_FIRST_DISPLAY = 1 << 5
FLAG_INPUT_LAST_VALID = 1 << 6


def _crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _u64(words: tuple[int, ...], low_word: int) -> int:
    return words[low_word] | (words[low_word + 1] << 32)


def _s32(value: int) -> int:
    return value - 0x1_0000_0000 if value & 0x80000000 else value


def decode_packet(data: bytes | bytearray | memoryview) -> dict:
    """Validate and decode exactly one 136-byte packet; no stream buffering is done."""
    packet = bytes(data)
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"packet length must be exactly {PACKET_SIZE} bytes")
    if packet[:4] != HEADER:
        raise ValueError("packet header mismatch")
    if packet[-2:] != TRAILER:
        raise ValueError("packet trailer mismatch")
    payload = packet[4:132]
    expected_crc = _crc16_ccitt_false(payload)
    got_crc = packet[132] | (packet[133] << 8)
    if got_crc != expected_crc:
        raise ValueError("packet CRC mismatch")
    words = struct.unpack("<32I", payload)
    if words[0] != MAGIC:
        raise ValueError("record magic mismatch (not a record or timeout packet)")
    version = words[1] >> 16
    if version != 1:
        raise ValueError(f"unsupported protocol version: {version}")
    flags = words[1] & 0xFFFF
    if words[15] != TICK_HZ:
        raise ValueError(f"unexpected tick frequency: {words[15]}")

    t0, t_last, t1, t2 = (_u64(words, idx) for idx in (7, 9, 11, 13))
    if not t0 <= t1 <= t2:
        raise ValueError("event timestamp order must satisfy T0 <= T1 <= T2")
    input_last_valid = bool(flags & FLAG_INPUT_LAST_VALID)
    if input_last_valid and not t0 <= t_last <= t1:
        raise ValueError("event timestamp order must satisfy T0 <= TinputLast <= T1")
    gamma_mode_word = words[27]
    if gamma_mode_word & 0x7FFFFFFF not in (0, 1):
        raise ValueError("unsupported gamma mode; expected auto(0) or identity(1), with optional busy bit")
    record = {
        "magic": words[0], "version": version, "flags": flags,
        "record_valid": bool(flags & FLAG_RECORD_VALID),
        "run_active": bool(flags & FLAG_RUN_ACTIVE),
        "gamma_identity": bool(flags & FLAG_GAMMA_IDENTITY),
        "tracker_invalid": bool(flags & FLAG_TRACKER_INVALID),
        "telemetry_overflow": bool(flags & FLAG_TELEMETRY_OVERFLOW),
        "first_display": bool(flags & FLAG_FIRST_DISPLAY),
        "input_last_valid": input_last_valid,
        "epoch": words[2], "record_seq": words[3],
        "source_frame_id": words[4], "sensor_frame_id": words[5],
        "angle_q16": _s32(words[6]), "angle_degrees": _s32(words[6]) / 65536.0,
        "t0_ticks": t0, "t_input_last_ticks": t_last, "t1_ticks": t1, "t2_ticks": t2,
        "tick_hz": words[15], "source_commit": words[16], "source_drop": words[17],
        "result_commit": words[18], "result_drop": words[19],
        "display_repeat": words[20], "telemetry_overflow_count": words[21],
        "unmatched_count": words[22], "input_observation_errors": words[23],
        "cfg_version": words[24] & 0xFFFF, "reserved": words[24],
        "measurement_cmd_applied_seq": words[25],
        "gamma_cmd_applied_seq": words[26],
        "gamma_mode": gamma_mode_word & 0x7FFFFFFF,
        "gamma_busy": bool(gamma_mode_word & 0x80000000),
        "live_core_ticks": _u64(words, 28), "input_frame_count": words[30],
        "command_error": words[31],
    }
    return record


def _delta32(previous: int, current: int, field: str) -> int:
    if current >= previous:
        return current - previous
    if previous >= _WRAP_HIGH and current <= _WRAP_LOW:
        return (COUNTER_MASK - previous) + 1 + current
    raise ValueError(f"cumulative counter reset detected in {field}")


def _is_primary(record: Mapping) -> bool:
    return (bool(record.get("record_valid")) and bool(record.get("run_active"))
            and not bool(record.get("tracker_invalid"))
            and not bool(record.get("telemetry_overflow")) and bool(record.get("input_last_valid"))
            and bool(record.get("first_display")))


def _stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "avg": None, "min": None, "max": None, "p95": None}
    ordered = sorted(values)
    return {"count": len(values), "avg": sum(values) / len(values), "min": ordered[0],
            "max": ordered[-1], "p95": ordered[math.ceil(0.95 * len(ordered)) - 1]}


def _latencies_ms(record: Mapping) -> dict[str, float]:
    hz = TICK_HZ
    scale = 1000.0 / hz
    return {
        "t1_minus_t0": (record["t1_ticks"] - record["t0_ticks"]) * scale,
        "t1_minus_tinput_last": (record["t1_ticks"] - record["t_input_last_ticks"]) * scale,
        "t2_minus_t0": (record["t2_ticks"] - record["t0_ticks"]) * scale,
        "t2_minus_t1": (record["t2_ticks"] - record["t1_ticks"]) * scale,
    }


def _summarize_subset(records: list[dict]) -> dict:
    metrics = defaultdict(list)
    for record in records:
        for name, value in _latencies_ms(record).items():
            metrics[name].append(value)
    return {name: _stats(metrics[name]) for name in
            ("t1_minus_t0", "t1_minus_tinput_last", "t2_minus_t0", "t2_minus_t1")}


def summarize_records(records: Iterable[Mapping], warmup: int = 30, target: int = 300) -> dict:
    """Summarize unique valid primary records; latency uses FPGA ticks only.

    T0/T2 event spans yield labeled sample rates only. Camera/source/result FPS
    use their cumulative counter differences over the paired liveCoreTicks snapshots.
    Cumulative counters may wrap once through uint32 near the boundary.
    """
    if not isinstance(warmup, int) or warmup < 0:
        raise ValueError("warmup must be a nonnegative integer")
    if not isinstance(target, int) or not 300 <= target <= 1000:
        raise ValueError("target must be between 300 and 1000")
    rows = [dict(record) for record in records]
    if not rows:
        return {"sample_count": 0, "warmup_discarded": 0, "duplicate_source_ids_skipped": 0,
                "filtered_record_count": 0, "telemetry_record_gaps": 0, "incomplete": True,
                "complete_record_window": False, "telemetry_record_sequence_contiguous": False,
                "video_drop_free": None, "valid_input_sample_rate_fps": None,
                "first_display_sample_rate_fps": None, "camera_fps": None, "source_fps": None,
                "result_fps": None, "latency_ms": _summarize_subset([]), "by_angle_q16": {},
                "samples": [], "drop_deltas": {"source": 0, "result": 0}}
    epochs = {row["epoch"] for row in rows}
    if len(epochs) != 1:
        raise ValueError("epoch change detected; refusing to mix measurement epochs")

    filtered = [row for row in rows if _is_primary(row)]
    filtered_count = len(rows) - len(filtered)
    unique, seen_ids, duplicate_count = [], set(), 0
    for row in filtered:
        frame_id = row["source_frame_id"]
        if frame_id in seen_ids:
            duplicate_count += 1
            continue
        seen_ids.add(frame_id)
        unique.append(row)
    if warmup:
        warmup_discarded = min(warmup, len(unique))
        unique = unique[warmup_discarded:]
    else:
        warmup_discarded = 0
    selected = unique[:target]

    # Enforce ordered samples and count missing telemetry sequences; duplicates were removed by source ID.
    gaps = 0
    for previous, current in zip(selected, selected[1:]):
        event_fields = ("t0_ticks", "t_input_last_ticks", "t1_ticks", "t2_ticks")
        if any(current[field] <= previous[field] for field in event_fields):
            raise ValueError("record event order regressed across samples")
        seq_delta = (current["record_seq"] - previous["record_seq"]) & COUNTER_MASK
        if seq_delta == 0:
            raise ValueError("record order has repeated record_seq for distinct source IDs")
        if seq_delta >= 0x80000000:
            raise ValueError("record order regressed across samples")
        gaps += max(0, seq_delta - 1)
        source_delta = (current["source_frame_id"] - previous["source_frame_id"]) & COUNTER_MASK
        if source_delta == 0 or source_delta >= 0x80000000:
            raise ValueError("source frame order regressed across samples")

    count_deltas = {"source_commit": 0, "source_drop": 0, "result_commit": 0,
                    "result_drop": 0, "input_frame_count": 0}
    for previous, current in zip(selected, selected[1:]):
        if current["live_core_ticks"] <= previous["live_core_ticks"]:
            raise ValueError("liveCoreTicks snapshot order/reset detected")
        for name in count_deltas:
            count_deltas[name] += _delta32(previous[name], current[name], name)
    drop_deltas = {"source": count_deltas["source_drop"], "result": count_deltas["result_drop"]}

    by_angle: dict[int, list[dict]] = defaultdict(list)
    samples = []
    for row in selected:
        latencies = _latencies_ms(row)
        by_angle[row["angle_q16"]].append(row)
        samples.append({"source_frame_id": row["source_frame_id"], "sensor_frame_id": row["sensor_frame_id"],
                        "record_seq": row["record_seq"], "angle_q16": row["angle_q16"],
                        "cfg_version": row.get("cfg_version"),
                        "gamma_mode": row["gamma_mode"], "gamma_identity": row["gamma_identity"],
                        "t0_ticks": row["t0_ticks"], "t_input_last_ticks": row["t_input_last_ticks"],
                        "t1_ticks": row["t1_ticks"], "t2_ticks": row["t2_ticks"],
                        "latency_ms": latencies})
    angle_summaries = {angle: _summarize_subset(angle_rows) for angle, angle_rows in sorted(by_angle.items())}
    input_span_ticks = selected[-1]["t0_ticks"] - selected[0]["t0_ticks"] if len(selected) >= 2 else 0
    display_span_ticks = selected[-1]["t2_ticks"] - selected[0]["t2_ticks"] if len(selected) >= 2 else 0
    snapshot_span_ticks = (selected[-1]["live_core_ticks"] - selected[0]["live_core_ticks"]
                           if len(selected) >= 2 else 0)
    continuous = bool(selected) and gaps == 0
    complete = continuous and len(selected) == target
    interval_count = max(0, len(selected) - 1)
    input_sample_rate = interval_count * TICK_HZ / input_span_ticks if input_span_ticks > 0 else None
    display_sample_rate = interval_count * TICK_HZ / display_span_ticks if display_span_ticks > 0 else None
    camera_fps = count_deltas["input_frame_count"] * TICK_HZ / snapshot_span_ticks if snapshot_span_ticks > 0 else None
    source_fps = count_deltas["source_commit"] * TICK_HZ / snapshot_span_ticks if snapshot_span_ticks > 0 else None
    result_fps = count_deltas["result_commit"] * TICK_HZ / snapshot_span_ticks if snapshot_span_ticks > 0 else None
    return {
        "epoch": next(iter(epochs)), "sample_count": len(selected), "warmup_discarded": warmup_discarded,
        "duplicate_source_ids_skipped": duplicate_count, "filtered_record_count": filtered_count,
        "telemetry_record_gaps": gaps, "incomplete": len(selected) < target,
        "complete_record_window": complete, "telemetry_record_sequence_contiguous": continuous,
        "video_drop_free": drop_deltas["source"] == 0 and drop_deltas["result"] == 0,
        "valid_input_sample_rate_fps": input_sample_rate,
        "first_display_sample_rate_fps": display_sample_rate,
        "camera_fps": camera_fps, "source_fps": source_fps, "result_fps": result_fps,
        "sample_span_ticks_t0": input_span_ticks, "sample_span_ticks_t2": display_span_ticks,
        "counter_snapshot_span_ticks_live_core": snapshot_span_ticks,
        "latency_ms": _summarize_subset(selected), "by_angle_q16": angle_summaries,
        "drop_deltas": drop_deltas, "count_deltas": count_deltas, "samples": samples,
    }
