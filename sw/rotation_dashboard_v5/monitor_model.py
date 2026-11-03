"""Pure decoders and freshness bookkeeping for V4 algorithm observations."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import time

GAMMA_HEADER = b"\x55\xaa\xe4\x14"
GAMMA_MAGIC = 0x474D0001
MAX_PIXELS = 2073600


def decode_gamma(packet: bytes) -> dict:
    if not isinstance(packet, bytes) or len(packet) != 27:
        raise ValueError("A8 response must be exactly 27 bytes")
    if packet[:4] != GAMMA_HEADER:
        raise ValueError("A8 header/status mismatch")
    payload = packet[4:24]
    check = 0
    for byte in payload:
        check ^= byte
    if packet[24] != check or packet[25:27] != b"\r\n":
        raise ValueError("A8 checksum or CRLF mismatch")
    bits = int.from_bytes(payload, "little")
    if bits & 0xffffffff != GAMMA_MAGIC:
        raise ValueError("A8 magic/version mismatch (unsupported firmware or timeout)")
    obs = bits >> 32
    total = obs & ((1 << 40) - 1)
    count = (obs >> 40) & 0xffffffff
    frame = (obs >> 72) & 0xffffffff
    config = (obs >> 104) & 0xffff
    applied, next_bank = (obs >> 120) & 7, (obs >> 123) & 7
    enabled, valid = bool((obs >> 126) & 1), bool((obs >> 127) & 1)
    if applied > 5 or next_bank > 5:
        raise ValueError("A8 Gamma bank is outside 0..5")
    if count > MAX_PIXELS or total > count * 255:
        raise ValueError("A8 luminance statistics exceed valid pixel bounds")
    if valid and count == 0:
        raise ValueError("A8 marks empty luminance statistics valid")
    return {"enabled": enabled, "applied_bank": applied, "next_bank": next_bank,
            "valid": valid, "valid_count": count, "sum": total,
            "mean_luminance": total / count if valid and count else None,
            "frame_id": frame, "config_version": config}


def decode_cnn(packet: bytes, host_path: Path) -> dict:
    spec = importlib.util.spec_from_file_location("v4_nn_host", host_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load frozen nn_host decoder: {host_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    decoded = module.decode_prediction_packet(packet)
    telemetry = decoded["telemetry"]
    flags = telemetry["flags"]
    count = int(telemetry.get("predictionCount") or 0)
    reasons = decoded.get("geometryChecks", {}).get("reasons", [])
    ready = bool(flags.get("loaded") and flags.get("k_valid") and count and
                 not flags.get("nn_error") and not flags.get("protocol_error") and
                 not flags.get("fault_latched") and not flags.get("saturated") and not reasons)
    fault_text = []
    if flags.get("nn_error"): fault_text.append(f"NN错误={flags['nn_error']}")
    if flags.get("protocol_error"): fault_text.append("协议错误")
    if flags.get("fault_latched"): fault_text.append("故障锁存")
    if flags.get("saturated"): fault_text.append("输出饱和")
    return {"loaded_bytes": telemetry.get("loadedBytes"),
            "decoded": decoded,
            "capability_word": int(telemetry.get("reservedWord31", 0)),
            "crc32": telemetry.get("computedCrc32"),
            "loaded": flags.get("loaded"), "enabled": flags.get("enabled"),
            "k_ready": flags.get("k_valid"), "busy": flags.get("nn_busy"),
            "error": '、'.join(fault_text) if fault_text else '无', "nn_error": flags.get("nn_error"),
            "protocol_error": flags.get("protocol_error"),
            "fault_latched": flags.get("fault_latched"), "saturated": flags.get("saturated"),
            "prediction_valid": ready, "prediction_reasons": list(reasons),
            "prediction_count": count,
            "cycles": telemetry.get("predictionCycles"),
            "nn_cycles": telemetry.get("nnCycles"),
            "predicted_angle_deg": decoded.get("direction", {}).get("angleDegrees") if ready else None,
            "corners": [[point["x"], point["y"]] for point in decoded.get("corners640x480", [])] if ready else []}


class RateTracker:
    def __init__(self):
        self.last_count = self.last_time = self.last_identity = None

    def rate(self, count: int, now: float | None = None, identity=None, ready=True) -> float | None:
        now = time.monotonic() if now is None else now
        rate = None
        count = int(count) & 0xffffffff
        if (ready and self.last_count is not None and self.last_time is not None
                and identity == self.last_identity):
            elapsed = now - self.last_time
            if count >= self.last_count:
                delta = count - self.last_count
            elif self.last_count > 0xf0000000 and count < 0x0fffffff:
                delta = (count - self.last_count) & 0xffffffff
            else:
                delta = None
            if 0 < elapsed <= 3 and delta is not None:
                rate = delta / elapsed
        self.last_count = count if ready else None
        self.last_time = now if ready else None
        self.last_identity = identity if ready else None
        return rate


class VideoProgressTracker:
    """Distinguish live input/output commits from an HDMI replay of old data."""
    def __init__(self):
        self.last = None
        self.last_result_change = None

    def observe(self, status, now=None):
        now = time.monotonic() if now is None else float(now)
        if not isinstance(status, dict) or 'resultFrameCommitCount' not in status:
            return {'state': 'unknown', 'detail': '尚未取得视频处理计数'}
        point = (int(status.get('cameraFrameCount', 0)) & 0xffffffff,
                 int(status['resultFrameCommitCount']) & 0xffffffff)
        previous = self.last
        if previous is None:
            self.last, self.last_result_change = point, now
            return {'state': 'checking', 'detail': '正在确认新画面'}
        camera_delta = (point[0] - previous[0]) & 0xffffffff
        result_delta = (point[1] - previous[1]) & 0xffffffff
        self.last = point
        if camera_delta >= 0x80000000 or result_delta >= 0x80000000:
            self.last_result_change = now
            return {'state': 'checking', 'detail': '板卡计数已重置，正在确认'}
        if result_delta:
            self.last_result_change = now
            return {'state': 'running', 'detail': '视频处理正常', 'resultDelta': result_delta}
        if now - self.last_result_change >= 3:
            detail = ('摄像头有输入，但板卡处理已停止；请复位板卡' if camera_delta else
                      '摄像头和结果帧均未更新，请检查板卡运行状态')
            return {'state': 'stalled', 'detail': detail}
        return {'state': 'checking', 'detail': '等待下一处理帧'}
