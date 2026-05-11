#!/usr/bin/env python3
"""
ble_hci_test.py — BLE HCI test suite over a raw H4 serial/PTY port

Talks directly in HCI H4 packets (type + fixed header + payload) — no
Spinel/MUX framing. Point it at the BLE PTY exposed by spinel_bridge.py
(e.g. /dev/pts/N) or any other H4-speaking serial port.

Runs a fixed suite of HCI/LE commands once per iteration and prints a
PASS/FAIL line per test, looping for a configurable number of iterations.

Usage:
    python3 ble_hci_test.py --port /dev/pts/4 --iterations 5
    python3 ble_hci_test.py --port /dev/ttyACM0 --baud 115200 -n 1 --timeout 3.0
"""

import argparse
import os
import struct
import sys
import time

import serial

# ---------------------------------------------------------------------------
# H4 packet type indicators and HCI event codes
# ---------------------------------------------------------------------------
H4_CMD   = 0x01
H4_ACL   = 0x02
H4_SCO   = 0x03
H4_EVENT = 0x04

HCI_EVT_COMMAND_COMPLETE = 0x0E
HCI_EVT_LE_META          = 0x3E
LE_META_ADV_REPORT       = 0x02

_H4_HEADER_LEN = {
    H4_CMD:   3,   # opcode(2) + param_total_length(1)
    H4_ACL:   4,   # handle+flags(2) + data_total_length(2)
    H4_SCO:   3,   # handle+flags(2) + data_total_length(1)
    H4_EVENT: 2,   # event_code(1) + param_total_length(1)
}

_H4_LEN_OFFSET = {
    H4_CMD:   2,
    H4_ACL:   2,   # 2-byte LE field
    H4_SCO:   2,
    H4_EVENT: 1,
}


def opcode(ogf: int, ocf: int) -> int:
    return (ogf << 10) | ocf


def hci_command(ogf: int, ocf: int, params: bytes = b'') -> bytes:
    return bytes([H4_CMD]) + struct.pack('<H', opcode(ogf, ocf)) + bytes([len(params)]) + params


# ---------------------------------------------------------------------------
# Raw H4 I/O helpers
# ---------------------------------------------------------------------------
def read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
    """Read exactly n bytes or raise TimeoutError before the deadline."""
    buf = bytearray()
    while len(buf) < n:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Timeout reading {n} bytes (got {len(buf)})")
        ser.timeout = min(0.1, remaining)
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def read_h4_packet(ser: serial.Serial, timeout: float) -> bytes:
    """Read one complete H4 packet (type + fixed header + payload)."""
    deadline = time.monotonic() + timeout
    pkt_type = read_exact(ser, 1, deadline)[0]
    if pkt_type not in _H4_HEADER_LEN:
        raise ValueError(f"Unknown H4 packet type: {pkt_type:#04x}")

    header = read_exact(ser, _H4_HEADER_LEN[pkt_type], deadline)
    len_off = _H4_LEN_OFFSET[pkt_type]
    if pkt_type == H4_ACL:
        payload_len = header[len_off] | (header[len_off + 1] << 8)
    else:
        payload_len = header[len_off]

    payload = read_exact(ser, payload_len, deadline) if payload_len else b''
    return bytes([pkt_type]) + header + payload


def send_command(ser: serial.Serial, ogf: int, ocf: int, params: bytes = b''):
    ser.write(hci_command(ogf, ocf, params))


def wait_command_complete(ser: serial.Serial, ogf: int, ocf: int, timeout: float):
    """
    Read H4 packets until the Command Complete event for (ogf, ocf)
    arrives. Any other event (LE Meta advertising reports, etc.) is
    skipped. Returns (status, return_params).
    """
    want_opcode = opcode(ogf, ocf)
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Timeout waiting for command complete (opcode={want_opcode:#06x})")
        pkt = read_h4_packet(ser, remaining)
        if pkt[0] != H4_EVENT or pkt[1] != HCI_EVT_COMMAND_COMPLETE:
            continue
        # [type][evt=0x0E][len][num_hci_cmd_pkts][opcode_lo][opcode_hi][status][params...]
        got_opcode = struct.unpack('<H', pkt[4:6])[0]
        if got_opcode != want_opcode:
            continue
        status = pkt[6]
        return status, pkt[7:]


def _require_status_ok(status: int):
    if status != 0x00:
        raise AssertionError(f"status=0x{status:02x}")


# ---------------------------------------------------------------------------
# Individual test cases
#
# Each test sends one (or a short sequence of) HCI command(s) and returns a
# short info string on success, or raises on failure. OGF/OCF values are
# from Bluetooth Core Spec Vol 4 Part E.
# ---------------------------------------------------------------------------
def test_reset(ser, timeout):
    send_command(ser, 0x03, 0x0003)                       # Reset
    status, _ = wait_command_complete(ser, 0x03, 0x0003, timeout)
    _require_status_ok(status)
    return ""


def test_read_local_version(ser, timeout):
    send_command(ser, 0x04, 0x0001)                        # Read Local Version Information
    status, params = wait_command_complete(ser, 0x04, 0x0001, timeout)
    _require_status_ok(status)
    if len(params) < 8:
        raise ValueError(f"response too short ({len(params)} bytes)")
    hci_ver    = params[0]
    hci_rev    = struct.unpack('<H', params[1:3])[0]
    manuf      = struct.unpack('<H', params[4:6])[0]
    lmp_subver = struct.unpack('<H', params[6:8])[0]
    return f"HCI ver=0x{hci_ver:02x} rev=0x{hci_rev:04x} manuf=0x{manuf:04x} lmp_subver=0x{lmp_subver:04x}"


def test_read_bd_addr(ser, timeout):
    send_command(ser, 0x04, 0x0009)                        # Read BD_ADDR
    status, params = wait_command_complete(ser, 0x04, 0x0009, timeout)
    _require_status_ok(status)
    if len(params) < 6:
        raise ValueError(f"response too short ({len(params)} bytes)")
    return ':'.join(f'{b:02X}' for b in reversed(params[:6]))


def test_le_read_buffer_size(ser, timeout):
    send_command(ser, 0x08, 0x0002)                        # LE Read Buffer Size
    status, params = wait_command_complete(ser, 0x08, 0x0002, timeout)
    _require_status_ok(status)
    if len(params) < 3:
        raise ValueError(f"response too short ({len(params)} bytes)")
    pkt_len  = struct.unpack('<H', params[0:2])[0]
    num_pkts = params[2]
    return f"{num_pkts} x {pkt_len}B ACL buffers"


def test_le_read_local_features(ser, timeout):
    send_command(ser, 0x08, 0x0003)                        # LE Read Local Supported Features
    status, params = wait_command_complete(ser, 0x08, 0x0003, timeout)
    _require_status_ok(status)
    if len(params) < 8:
        raise ValueError(f"response too short ({len(params)} bytes)")
    mask = int.from_bytes(params[:8], 'little')
    return f"features=0x{mask:016x}"


def test_le_rand(ser, timeout):
    send_command(ser, 0x08, 0x0018)                        # LE Rand
    status, params = wait_command_complete(ser, 0x08, 0x0018, timeout)
    _require_status_ok(status)
    if len(params) < 8:
        raise ValueError(f"response too short ({len(params)} bytes)")
    return f"random={params[:8].hex()}"


def test_le_read_supported_states(ser, timeout):
    send_command(ser, 0x08, 0x001C)                        # LE Read Supported States
    status, params = wait_command_complete(ser, 0x08, 0x001C, timeout)
    _require_status_ok(status)
    if len(params) < 8:
        raise ValueError(f"response too short ({len(params)} bytes)")
    mask = int.from_bytes(params[:8], 'little')
    return f"states=0x{mask:016x}"


def test_le_set_random_address(ser, timeout):
    addr = bytearray(os.urandom(6))
    addr[5] = (addr[5] & 0x3F) | 0xC0    # mark as a static random address
    send_command(ser, 0x08, 0x0005, bytes(addr))            # LE Set Random Address
    status, _ = wait_command_complete(ser, 0x08, 0x0005, timeout)
    _require_status_ok(status)
    return ':'.join(f'{b:02X}' for b in reversed(addr))


def test_le_set_advertising_parameters(ser, timeout):
    params = struct.pack(
        '<HHBBB6sBB',
        0x00A0, 0x00A0,      # Advertising_Interval_Min/Max (~100 ms)
        0x00,                # Advertising_Type = ADV_IND
        0x01,                # Own_Address_Type = random
        0x00,                # Peer_Address_Type = public
        b'\x00' * 6,          # Peer_Address (unused for ADV_IND)
        0x07,                # Advertising_Channel_Map = all 3 channels
        0x00)                # Advertising_Filter_Policy
    send_command(ser, 0x08, 0x0006, params)                 # LE Set Advertising Parameters
    status, _ = wait_command_complete(ser, 0x08, 0x0006, timeout)
    _require_status_ok(status)
    return ""


def _build_ad_structure(local_name: bytes) -> bytes:
    """
    Build a 31-byte, length-prefixed AD payload (Flags + optional Complete
    Local Name), used for both Advertising Data and Scan Response Data.
    """
    flags = bytes([0x02, 0x01, 0x06])   # len=2, type=Flags, LE General Discoverable + no BR/EDR
    name_ad = (bytes([len(local_name) + 1, 0x09]) + local_name) if local_name else b''
    data = (flags + name_ad)[:31]
    return bytes([len(data)]) + data + bytes(31 - len(data))


def test_le_set_advertising_data(ser, timeout):
    send_command(ser, 0x08, 0x0008, _build_ad_structure(b'TI-BLE-Test'))  # LE Set Advertising Data
    status, _ = wait_command_complete(ser, 0x08, 0x0008, timeout)
    _require_status_ok(status)
    return ""


def test_le_set_scan_response_data(ser, timeout):
    send_command(ser, 0x08, 0x0009, _build_ad_structure(b''))  # LE Set Scan Response Data
    status, _ = wait_command_complete(ser, 0x08, 0x0009, timeout)
    _require_status_ok(status)
    return ""


def test_le_set_advertise_enable(ser, timeout):
    send_command(ser, 0x08, 0x000A, bytes([0x01]))          # LE Set Advertise Enable
    status, _ = wait_command_complete(ser, 0x08, 0x000A, timeout)
    _require_status_ok(status)
    return ""


def test_le_set_scan_parameters(ser, timeout):
    params = struct.pack(
        '<BHHBB',
        0x01,      # Scan_Type = active
        0x0060,    # Scan_Interval  (96 * 0.625ms = 60ms)
        0x0018,    # Scan_Window    (24 * 0.625ms = 15ms, 25% duty cycle)
        0x00,      # Own_Address_Type = public
        0x00)      # Scan_Filtering_Policy
    send_command(ser, 0x08, 0x000B, params)                 # LE Set Scan Parameters
    status, _ = wait_command_complete(ser, 0x08, 0x000B, timeout)
    _require_status_ok(status)
    return ""


def test_le_scan_for_devices(ser, timeout, scan_duration=10.0):
    # A 100%-duty-cycle scan floods the shared combined-serial UART with
    # advertising-report events — enough nearby BLE traffic can saturate
    # the link and delay concurrent OpenThread spinel replies past
    # otbr-agent's fixed 2s RCP response timeout. The realistic duty cycle
    # set above (rather than dedup filtering) is what keeps this test from
    # starving OT traffic.
    try:
        send_command(ser, 0x08, 0x000C, bytes([0x01, 0x00]))  # LE Set Scan Enable(on, filter_dup=off)
        status, _ = wait_command_complete(ser, 0x08, 0x000C, timeout)
        _require_status_ok(status)
    except Exception as e:
        raise type(e)(f"scan enable: {e}") from None

    reports = 0
    deadline = time.monotonic() + scan_duration
    while time.monotonic() < deadline:
        try:
            pkt = read_h4_packet(ser, deadline - time.monotonic())
        except TimeoutError:
            break
        if (pkt[0] == H4_EVENT and pkt[1] == HCI_EVT_LE_META
                and len(pkt) > 3 and pkt[3] == LE_META_ADV_REPORT):
            reports += 1

    try:
        send_command(ser, 0x08, 0x000C, bytes([0x00, 0x00]))  # LE Set Scan Enable(off)
        status, _ = wait_command_complete(ser, 0x08, 0x000C, timeout)
        _require_status_ok(status)
    except Exception as e:
        raise type(e)(f"scan disable: {e}") from None

    return f"{reports} advertising report(s) seen in {scan_duration:.0f}s"


def test_reset_cleanup(ser, timeout):
    return test_reset(ser, timeout)


# ---------------------------------------------------------------------------
# Test suite (order matters: leaves the controller reset at the end)
# ---------------------------------------------------------------------------
TESTS = [
    ("HCI Reset",                          test_reset),
    ("HCI Read Local Version",              test_read_local_version),
    ("HCI Read BD_ADDR",                    test_read_bd_addr),
    ("LE Read Buffer Size",                 test_le_read_buffer_size),
    ("LE Read Local Supported Features",    test_le_read_local_features),
    ("LE Rand (random number)",             test_le_rand),
    ("LE Read Supported States",            test_le_read_supported_states),
    ("LE Set Random Address",               test_le_set_random_address),
    ("LE Set Advertising Parameters",       test_le_set_advertising_parameters),
    ("LE Set Advertising Data",             test_le_set_advertising_data),
    ("LE Set Scan Response Data",           test_le_set_scan_response_data),
    ("LE Set Advertise Enable",             test_le_set_advertise_enable),
    ("LE Set Scan Parameters",              test_le_set_scan_parameters),
    ("LE Scan for Devices (10 s)",          test_le_scan_for_devices),
    ("HCI Reset (cleanup)",                 test_reset_cleanup),
]


def run_iteration(ser: serial.Serial, timeout: float, iteration: int, total: int) -> bool:
    suffix = f"  [iteration {iteration}/{total}]" if total > 1 else ""
    print(f"Running HCI tests (timeout={timeout:.1f}s each) ...{suffix}")
    print('-' * 70)

    passed = 0
    for name, fn in TESTS:
        try:
            info = fn(ser, timeout)
            print(f"  PASS  {name}" + (f"  — {info}" if info else ""))
            passed += 1
        except Exception as e:
            print(f"  FAIL  {name}  — {e}")

    print('-' * 70)
    print(f"  {passed}/{len(TESTS)} passed")
    return passed == len(TESTS)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="BLE HCI test suite over a raw H4 serial/PTY port",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--port', required=True,
                     help='Serial/PTY device speaking raw HCI H4, e.g. /dev/pts/4 or /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200,
                     help='Baud rate (ignored when --port is a PTY)')
    ap.add_argument('--iterations', '-n', type=int, default=1,
                     help='Number of times to repeat the full test suite')
    ap.add_argument('--timeout', type=float, default=3.0,
                     help='Per-command response timeout in seconds')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: could not open {args.port}: {e}")
        return 1

    time.sleep(0.2)
    ser.reset_input_buffer()

    all_passed = True
    try:
        for i in range(1, args.iterations + 1):
            if not run_iteration(ser, args.timeout, i, args.iterations):
                all_passed = False
            if i < args.iterations:
                print()
    finally:
        ser.close()

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
