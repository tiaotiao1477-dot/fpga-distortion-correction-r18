"""One bounded, serialized A7/A8 UART observation exchange."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import time
import sys

from monitor_model import decode_gamma, decode_cnn

A7_HEADER = b"\x55\xaa\xe3\x80"
A7_SIZE = 135


def read_a5_state(port: str, tools_dir: Path, timeout: float = 1.0) -> dict:
    """Read one fresh native A5 status using the R14-selected console worker."""
    import importlib.util
    import serial
    decoder_path = tools_dir / 'decode_algorithm_packet.py'
    raw_path = tools_dir / 'raw1080_uart.py'
    spec = importlib.util.spec_from_file_location('_v4_r14_diag_decoder', decoder_path)
    decoder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(decoder)
    old_decoder = sys.modules.get('decode_algorithm_packet')
    sys.modules['decode_algorithm_packet'] = decoder
    try:
        raw_spec = importlib.util.spec_from_file_location('_v4_r14_raw_uart', raw_path)
        raw_module = importlib.util.module_from_spec(raw_spec)
        raw_spec.loader.exec_module(raw_module)
    finally:
        if old_decoder is None:
            sys.modules.pop('decode_algorithm_packet', None)
        else:
            sys.modules['decode_algorithm_packet'] = old_decoder
    uart = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.5)
    uart.port, uart.dtr, uart.rts = port, False, False
    try:
        uart.open()
        packet = read_packet(uart, b'\xa5', b'\x55\xaa\xd1\xf4', 251, timeout)
        decoded = raw_module.decode_raw1080(packet)
    finally:
        if uart.is_open:
            uart.close()
    config = decoded.get('algorithmConfig')
    if not isinstance(config, dict) or config.get('fresh') is not True:
        raise ValueError('Fresh A5 native algorithm diagnostics are unavailable')
    if config.get('frameError') is not False:
        raise ValueError('Fresh A5 diagnostic reports a frame error')
    return decoded


def read_packet(uart, request: bytes, header: bytes, size: int, timeout: float = .6) -> bytes:
    uart.reset_input_buffer()
    uart.write(request)
    deadline = time.monotonic() + timeout
    data = bytearray()
    while time.monotonic() < deadline:
        data.extend(uart.read(64))
        offset = data.find(header)
        if offset >= 0 and len(data) >= offset + size:
            return bytes(data[offset:offset + size])
    raise TimeoutError(f"UART observation timeout waiting for {header.hex()}")


def read_observations(port: str, candidate_tools: Path, *, include_a8: bool = True,
                      include_a5: bool = False) -> dict:
    """Open/close serial inside the ControlWorker only; never retries writes."""
    import serial
    a7 = candidate_tools / "nn_host.py"
    uart = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.2)
    uart.port, uart.dtr, uart.rts = port, False, False
    result = {"cnn": None, "gamma": None, "a7_error": None, "a8_error": None}
    try:
        uart.open()
        try:
            packet = read_packet(uart, b"\xa7", A7_HEADER, A7_SIZE)
            result["cnn"] = decode_cnn(packet, a7)
            result["cnn"]["raw_a7_hex"] = packet.hex()
            result["cnn"]["raw_packet_hex"] = packet.hex()
        except Exception as exc:
            result["a7_error"] = str(exc)
        if include_a8:
            try:
                result["gamma"] = decode_gamma(read_packet(uart, b"\xa8", b"\x55\xaa\xe4\x14", 27))
            except Exception as exc:
                result["a8_error"] = str(exc)
    finally:
        if uart.is_open:
            uart.close()
    if include_a5:
        try:
            from interactive_control import _live_read_a5
            result['video'] = _live_read_a5(port)
        except Exception as exc:
            result['a5_error'] = str(exc)
    return result
