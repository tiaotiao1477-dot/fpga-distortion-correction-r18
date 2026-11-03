"""Synchronous A9 transport over a caller-owned serial controller.

This module never opens/configures/closes a serial port. The caller must pass
its already-owned serial-like object; a finite read timeout is required.
"""
from __future__ import annotations

import time
from typing import Callable

from .latency_protocol import HEADER, PACKET_SIZE, decode_packet

LEGACY_LENGTHS={b'\x55\xaa\xd1\xf4':251,b'\x55\xaa\xe3\x80':135,b'\x55\xaa\xe4\x14':27}
LEGACY_HEADERS = tuple(LEGACY_LENGTHS)+(b"\x55\xaa\xa5\x80", b"\x55\xaa\xa7\x80", b"\x55\xaa\xa8\x80")
MAX_LEGACY_FRAME_BYTES = 4096


class MeasurementTransportError(RuntimeError):
    """Base error for errors reading or writing the measurement UART protocol."""


class MeasurementTimeout(MeasurementTransportError, TimeoutError):
    pass


class MeasurementFrameError(MeasurementTransportError, ValueError):
    pass


class MeasurementTransport:
    def __init__(self, controller, timeout: float = 1.0, *, clock: Callable[[], float] = time.monotonic):
        if not callable(getattr(controller, "write", None)) or not callable(getattr(controller, "read", None)):
            raise TypeError("controller must be an existing serial-like object with write/read")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.controller = controller
        self.timeout = float(timeout)
        self._clock = clock
        self._buffer = bytearray()

    def query(self, *, timeout: float | None = None) -> bytes:
        """Write one A9 request and return one CRC-validated 136-byte A9 response."""
        duration = self.timeout if timeout is None else float(timeout)
        if duration <= 0:
            raise ValueError("timeout must be positive")
        written = self.controller.write(b"\xa9")
        if written not in (None, 1):
            raise MeasurementTransportError(f"A9 request write was incomplete: {written}/1 bytes")
        deadline = self._clock() + duration
        return self._read_packet(deadline)

    def _read_packet(self, deadline: float) -> bytes:
        while True:
            packet = self._extract_packet()
            if packet is not None:
                try:
                    decode_packet(packet)
                except ValueError as exc:
                    raise MeasurementFrameError(str(exc)) from exc
                return packet
            if self._clock() >= deadline:
                raise MeasurementTimeout("timed out waiting for a complete A9 measurement response")
            chunk = self.controller.read(1)
            if chunk:
                if not isinstance(chunk, (bytes, bytearray, memoryview)):
                    raise MeasurementTransportError("serial read must return bytes")
                self._buffer.extend(chunk)
            else:
                # Yield for nonblocking serial-like implementations; their owner retains timeout control.
                time.sleep(min(0.001, max(0.0, deadline - self._clock())))

    def _extract_packet(self) -> bytes | None:
        while True:
            a9_index = self._buffer.find(HEADER)
            legacy_candidates = [(self._buffer.find(header), header) for header in LEGACY_HEADERS]
            legacy_candidates = [(idx, header) for idx, header in legacy_candidates if idx >= 0]
            legacy_index, legacy_header = min(legacy_candidates, default=(-1, b""), key=lambda row: row[0])

            if legacy_index >= 0 and (a9_index < 0 or legacy_index < a9_index):
                if legacy_header in LEGACY_LENGTHS:
                    length=LEGACY_LENGTHS[legacy_header]
                    if len(self._buffer)-legacy_index<length:
                        del self._buffer[:legacy_index]
                        return None
                    del self._buffer[:legacy_index+length]
                    continue
                # Legacy streams terminate with CRLF; discard the whole frame so payload
                # bytes that resemble a new A9 header cannot be mistaken for a record.
                terminator = self._buffer.find(b"\x0d\x0a", legacy_index + len(legacy_header))
                if terminator < 0:
                    if len(self._buffer) - legacy_index > MAX_LEGACY_FRAME_BYTES:
                        raise MeasurementFrameError("legacy A5/A7/A8 frame exceeded framing limit")
                    del self._buffer[:legacy_index]
                    return None
                del self._buffer[:terminator + 2]
                continue

            if a9_index >= 0:
                if a9_index:
                    del self._buffer[:a9_index]
                if len(self._buffer) < PACKET_SIZE:
                    return None
                packet = bytes(self._buffer[:PACKET_SIZE])
                del self._buffer[:PACKET_SIZE]
                return packet

            # Keep the longest suffix that could start a split header.
            keep = 0
            for header in (HEADER,)+LEGACY_HEADERS:
                for size in range(1,len(header)):
                    if self._buffer.endswith(header[:size]):keep=max(keep,size)
            if len(self._buffer) > keep:
                if keep:
                    del self._buffer[:-keep]
                else:
                    self._buffer.clear()
            return None


def _encode_command(op: int, seq: int, address: int, data: bytes) -> bytes:
    """Use the existing offline nn_host encoder; this wrapper opens no device."""
    from pathlib import Path
    import importlib.util

    repo = Path(__file__).resolve().parents[2]
    helper = repo / "wanchengceshi" / "algorithm_v04_1080p_upgrade" / "algorithm1080_nn_r1" / "tools" / "nn_host.py"
    spec = importlib.util.spec_from_file_location("nn_host_r16_transport", helper)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import the shared nn_host command encoder: {helper}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.encode_command(op, seq, address, data)


class MeasurementCommands:
    """Fresh-sequence op10/op11 commands with feedback-confirmed application."""
    OP_MEASUREMENT = 0x10
    OP_GAMMA_OVERRIDE = 0x11

    def __init__(self, transport: MeasurementTransport, *, ack_timeout: float = 2.0,
                 clock: Callable[[], float] = time.monotonic, packet_sink: Callable[[bytes], None] | None = None):
        self.transport = transport
        self.ack_timeout = float(ack_timeout)
        if self.ack_timeout <= 0:
            raise ValueError("ack_timeout must be positive")
        self._clock = clock
        self._next_seq: int | None = None
        self.packet_sink = packet_sink

    def _fresh_seq(self, record: dict) -> int:
        if self._next_seq is None:
            self._next_seq = (max(record["measurement_cmd_applied_seq"],
                                  record["gamma_cmd_applied_seq"]) + 1) & 0xFFFFFFFF
        seq = self._next_seq
        self._next_seq = (seq + 1) & 0xFFFFFFFF
        return seq

    def _send_and_wait(self, op: int, seq: int, address: int, data: bytes, applied_field: str,
                       feedback: Callable[[dict], bool] | None = None) -> dict:
        command = _encode_command(op, seq, address, data)
        written = self.transport.controller.write(command)
        if written not in (None, len(command)):
            raise MeasurementTransportError(f"command write was incomplete: {written}/{len(command)} bytes")
        deadline = self._clock() + self.ack_timeout
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise MeasurementTimeout(f"timed out waiting for applied sequence {seq}")
            record = self.transport.query(timeout=min(self.transport.timeout, remaining))
            if self.packet_sink is not None:
                self.packet_sink(record)
            from .latency_protocol import decode_packet
            decoded = decode_packet(record)
            applied = decoded[applied_field]
            if applied == seq and (feedback is None or feedback(decoded)):
                return decoded
            if applied != seq and ((applied - seq) & 0xFFFFFFFF) < 0x80000000:
                raise MeasurementTransportError(f"applied sequence advanced past command {seq}: {applied}")

    def set_measurement(self, angle_q16: int, run: bool, *, previous: dict | None = None) -> dict:
        if not isinstance(angle_q16, int) or not -(1 << 31) <= angle_q16 < (1 << 31):
            raise ValueError("angle_q16 must fit in signed int32")
        if previous is None:
            previous = self.transport_record()
        seq = self._fresh_seq(previous)
        data = bytearray(32)
        data[0] = int(bool(run))
        return self._send_and_wait(self.OP_MEASUREMENT, seq, angle_q16 & 0xFFFFFFFF, bytes(data),
                                   "measurement_cmd_applied_seq",
                                   feedback=lambda row: row["angle_q16"] == angle_q16
                                   and row["run_active"] is bool(run))

    def set_gamma_identity(self, identity: bool, *, previous: dict | None = None) -> dict:
        if previous is None:
            previous = self.transport_record()
        seq = self._fresh_seq(previous)
        data = bytearray(32)
        data[0] = int(bool(identity))
        return self._send_and_wait(
            self.OP_GAMMA_OVERRIDE, seq, 0, bytes(data), "gamma_cmd_applied_seq",
            feedback=lambda row: row["gamma_identity"] is bool(identity) and not row["gamma_busy"],
        )

    def transport_record(self) -> dict:
        from .latency_protocol import decode_packet
        packet = self.transport.query()
        if self.packet_sink is not None:
            self.packet_sink(packet)
        return decode_packet(packet)
