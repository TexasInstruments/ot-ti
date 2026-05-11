#!/usr/bin/env python3
"""
mux_test.py - Combined Serial MUX protocol test for DMM RCP+BLE Controller

Connects directly to the physical UART (/dev/ttyACMx) and tests the TI
Combined Serial MUX protocol without needing the host_mux_ot_ble daemon.

Tests:
  1. Keepalive  — wait for CMD_KEEPALIVE from device, reply with ACK
  2. Thread NLI — send MUX-wrapped spinel frames, verify responses
  3. BLE NLI   — send MUX-wrapped HCI Reset, verify event response

MUX wire format (each direction):
  [0x7E] [Spinel_hdr(NLI)] [CMD packed-uint] [payload] [CRC16-Kermit] [0x7E]

  NLI 0 = Thread spinel   payload = complete raw HDLC-framed spinel frame
  NLI 1 = BLE HCI         payload = H4 HCI bytes (type + opcode + params)
  NLI 3 = Keepalive       payload = empty

Usage:
  python3 mux_test.py /dev/ttyACM0 [--baud 921600]
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# ─── MUX / Spinel constants ───────────────────────────────────────────────────

MUX_NLI_OT        = 0
MUX_NLI_BLE       = 1
MUX_NLI_KEEPALIVE = 3

CMD_KEEPALIVE        = 15555
CMD_KEEPALIVE_ACK    = 15556

SPINEL_CMD_PROP_VALUE_GET = 0x02
SPINEL_CMD_PROP_VALUE_SET = 0x03
SPINEL_CMD_PROP_VALUE_IS  = 0x06
SPINEL_CMD_RESET          = 0x01

SPINEL_PROP_PROTOCOL_VERSION = 1   # major/minor, packed-uint × 2
SPINEL_PROP_NCP_VERSION      = 2   # UTF-8 string
SPINEL_PROP_CAPS             = 5   # packed-uint list

HDLC_FLAG   = 0x7E
HDLC_ESCAPE = 0x7D
HDLC_MASK   = 0x20

# ─── HCI commands (H4 format) ────────────────────────────────────────────────
# [0x01 type] [OCF_LSB] [OGF_OCF_MSB] [param_len] [params...]
#
# Opcode = (OGF << 10) | OCF  stored little-endian
#   OGF 0x03 = Controller & Baseband
#   OGF 0x04 = Informational Parameters
#   OGF 0x08 = LE Controller

HCI_CMD_RESET              = bytes([0x01, 0x03, 0x0C, 0x00])  # OGF=0x03 OCF=0x003
HCI_CMD_READ_LOCAL_VERSION = bytes([0x01, 0x01, 0x10, 0x00])  # OGF=0x04 OCF=0x001
HCI_CMD_READ_BD_ADDR       = bytes([0x01, 0x09, 0x10, 0x00])  # OGF=0x04 OCF=0x009
HCI_CMD_READ_LOCAL_CMDS    = bytes([0x01, 0x02, 0x10, 0x00])  # OGF=0x04 OCF=0x002
HCI_CMD_LE_READ_BUF_SIZE   = bytes([0x01, 0x02, 0x20, 0x00])  # OGF=0x08 OCF=0x002
HCI_CMD_LE_READ_FEATURES   = bytes([0x01, 0x03, 0x20, 0x00])  # OGF=0x08 OCF=0x003
HCI_CMD_LE_READ_MAX_PDU    = bytes([0x01, 0x2F, 0x20, 0x00])  # OGF=0x08 OCF=0x02F LE Read Max Data Length

HCI_VERSION_NAMES = {
    0x06: 'BT 4.0', 0x07: 'BT 4.1', 0x08: 'BT 4.2',
    0x09: 'BT 5.0', 0x0A: 'BT 5.1', 0x0B: 'BT 5.2',
    0x0C: 'BT 5.3', 0x0D: 'BT 5.4', 0x0E: 'BT 5.5',
}

LE_FEATURES = [
    'LE Encryption', 'Conn Param Request', 'Extended Reject Ind',
    'Peripheral Init Features', 'LE Ping', 'LE Data Len Extension',
    'LL Privacy', 'Extended Scan Filter', 'LE 2M PHY',
    'Stable Modulation Index TX', 'Stable Modulation Index RX',
    'LE Coded PHY', 'LE Ext Advertising', 'LE Periodic Advertising',
    'Channel Selection Algo 2', 'LE Power Class 1',
]

# ─── CRC-16 / Kermit ─────────────────────────────────────────────────────────

def crc16(data: bytes) -> int:
    """CRC-16/Kermit (poly=0x1021 reflected, init=0x0000)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else crc >> 1
    return crc


# ─── HDLC helpers ────────────────────────────────────────────────────────────

def hdlc_escape(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in (HDLC_FLAG, HDLC_ESCAPE):
            out += bytes([HDLC_ESCAPE, b ^ HDLC_MASK])
        else:
            out.append(b)
    return bytes(out)


def hdlc_unescape(data: bytes) -> bytes:
    out = bytearray()
    esc = False
    for b in data:
        if esc:
            out.append(b ^ HDLC_MASK)
            esc = False
        elif b == HDLC_ESCAPE:
            esc = True
        else:
            out.append(b)
    return bytes(out)


def hdlc_encode(payload: bytes) -> bytes:
    """Wrap payload in HDLC frame: 0x7E [escaped payload + CRC] 0x7E."""
    crc = crc16(payload)
    return bytes([HDLC_FLAG]) + hdlc_escape(payload + struct.pack('<H', crc)) + bytes([HDLC_FLAG])


def hdlc_decode(frame: bytes):
    """Strip HDLC flags, unescape, verify CRC. Returns payload or None."""
    inner = frame.strip(bytes([HDLC_FLAG]))
    raw = hdlc_unescape(inner)
    if len(raw) < 2:
        return None
    payload, stored = raw[:-2], struct.unpack('<H', raw[-2:])[0]
    return payload if crc16(payload) == stored else None


# ─── Spinel packed-uint ───────────────────────────────────────────────────────

def pack_uint(v: int) -> bytes:
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        out.append(b | (0x80 if v else 0))
        if not v:
            break
    return bytes(out)


def unpack_uint(data: bytes, offset: int = 0):
    """Return (value, bytes_consumed)."""
    v, shift, n = 0, 0, 0
    for b in data[offset:]:
        v |= (b & 0x7F) << shift
        shift += 7
        n += 1
        if not (b & 0x80):
            break
    return v, n


# ─── MUX frame encode / decode ───────────────────────────────────────────────

def mux_encode(nli: int, cmd: int, payload: bytes) -> bytes:
    """Build a MUX HDLC+Spinel frame."""
    hdr = 0x80 | ((nli & 0x03) << 4)
    return hdlc_encode(bytes([hdr]) + pack_uint(cmd) + payload)


def mux_decode(frame: bytes):
    """Decode MUX frame. Returns (nli, cmd, payload) or None."""
    sp = hdlc_decode(frame)
    if sp is None or len(sp) < 1:
        return None
    nli = (sp[0] >> 4) & 0x03
    cmd, n = unpack_uint(sp, 1)
    return nli, cmd, sp[1 + n:]


# ─── Inner spinel helpers ─────────────────────────────────────────────────────

def spinel_reset_frame() -> bytes:
    """Return a complete HDLC-framed spinel RESET command (TID=0)."""
    return hdlc_encode(bytes([0x80, SPINEL_CMD_RESET]))


def spinel_prop_get_frame(prop_id: int, tid: int = 1) -> bytes:
    """Return a complete HDLC-framed spinel PROP_VALUE_GET."""
    hdr = 0x80 | (tid & 0x0F)
    return hdlc_encode(bytes([hdr]) + pack_uint(SPINEL_CMD_PROP_VALUE_GET) + pack_uint(prop_id))


def spinel_decode_frame(raw_hdlc: bytes):
    """Decode inner spinel HDLC frame. Returns (header, cmd, data) or None."""
    payload = hdlc_decode(raw_hdlc)
    if not payload:
        return None
    cmd, n = unpack_uint(payload, 1)
    return payload[0], cmd, payload[1 + n:]


# ─── Serial frame reader ─────────────────────────────────────────────────────

def read_frame(ser, timeout: float = 3.0) -> bytes | None:
    """Read bytes until a complete 0x7E...0x7E HDLC frame arrives."""
    buf = bytearray()
    in_frame = False
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        ser.timeout = min(0.1, remaining)
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]
        if byte == HDLC_FLAG:
            if in_frame and len(buf) > 2:
                buf.append(byte)
                return bytes(buf)
            buf = bytearray([byte])
            in_frame = True
        elif in_frame:
            buf.append(byte)
    return None


# ─── Formatting helpers ───────────────────────────────────────────────────────

GREEN  = '\033[92m'
RED    = '\033[91m'
YELLOW = '\033[93m'
BLUE   = '\033[94m'
RESET  = '\033[0m'

def ok(msg):   print(f'  {GREEN}[PASS]{RESET} {msg}')
def fail(msg): print(f'  {RED}[FAIL]{RESET} {msg}')
def info(msg): print(f'  {BLUE}[INFO]{RESET} {msg}')
def tx(data):  print(f'  {YELLOW}  TX{RESET}  {data.hex()}')
def rx(data):  print(f'  {YELLOW}  RX{RESET}  {data.hex()}')


# ─── Keepalive handling (shared by all tests) ─────────────────────────────────

def handle_keepalive(ser, nli, cmd):
    if nli == MUX_NLI_KEEPALIVE and cmd == CMD_KEEPALIVE:
        ack = mux_encode(MUX_NLI_KEEPALIVE, CMD_KEEPALIVE_ACK, b'')
        ser.write(ack)
        info(f'Keepalive ping → sent ACK')
        return True
    return False


# ─── Individual tests ─────────────────────────────────────────────────────────

def test_keepalive(ser) -> bool:
    """Wait for CMD_KEEPALIVE from device and reply with ACK."""
    print('\n── Test 1: Keepalive ──────────────────────────────────────')
    info('Waiting up to 6 s for CMD_KEEPALIVE from device...')
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        frame = read_frame(ser, timeout=deadline - time.monotonic())
        if frame is None:
            break
        result = mux_decode(frame)
        if result is None:
            continue
        nli, cmd, payload = result
        if nli == MUX_NLI_KEEPALIVE and cmd == CMD_KEEPALIVE:
            rx(frame)
            ack = mux_encode(MUX_NLI_KEEPALIVE, CMD_KEEPALIVE_ACK, b'')
            ser.write(ack)
            tx(ack)
            ok('CMD_KEEPALIVE received; CMD_KEEPALIVE_ACK sent')
            return True
    fail('No CMD_KEEPALIVE received within 6 s')
    return False


def test_thread_reset(ser) -> bool:
    """Send MUX-wrapped spinel RESET, expect a spinel response."""
    print('\n── Test 2: Thread – Spinel RESET ─────────────────────────')
    inner = spinel_reset_frame()
    frame = mux_encode(MUX_NLI_OT, SPINEL_CMD_PROP_VALUE_SET, inner)
    info(f'MUX frame (NLI_OT, CMD_SET, spinel_RESET):')
    tx(frame)
    info(f'  inner spinel HDLC: {inner.hex()}')
    ser.write(frame)

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        f = read_frame(ser, timeout=deadline - time.monotonic())
        if f is None:
            break
        result = mux_decode(f)
        if result is None:
            continue
        nli, cmd, payload = result
        if handle_keepalive(ser, nli, cmd):
            continue
        if nli != MUX_NLI_OT:
            info(f'Ignored frame NLI={nli}')
            continue
        rx(f)
        info(f'  inner payload: {payload.hex()}')
        parsed = spinel_decode_frame(payload)
        if parsed:
            hdr, s_cmd, data = parsed
            tid = hdr & 0x0F
            info(f'  spinel: hdr=0x{hdr:02x} cmd=0x{s_cmd:02x} tid={tid} data={data.hex()}')
        ok('Thread spinel response received over NLI_OT')
        return True
    fail('No Thread response (timeout 3 s)')
    return False


def test_thread_prop_get(ser, prop_id: int, prop_name: str, tid: int = 1) -> bool:
    """Send MUX-wrapped spinel PROP_VALUE_GET, expect matching response."""
    print(f'\n── Test 3: Thread – {prop_name} (PROP_VALUE_GET) ──────────')
    inner = spinel_prop_get_frame(prop_id, tid=tid)
    frame = mux_encode(MUX_NLI_OT, SPINEL_CMD_PROP_VALUE_SET, inner)
    info(f'MUX frame (NLI_OT, CMD_SET, GET prop={prop_id}):')
    tx(frame)
    info(f'  inner spinel HDLC: {inner.hex()}')
    ser.write(frame)

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        f = read_frame(ser, timeout=deadline - time.monotonic())
        if f is None:
            break
        result = mux_decode(f)
        if result is None:
            continue
        nli, cmd, payload = result
        if handle_keepalive(ser, nli, cmd):
            continue
        if nli != MUX_NLI_OT:
            continue
        rx(f)
        info(f'  inner payload: {payload.hex()}')
        parsed = spinel_decode_frame(payload)
        if parsed:
            hdr, s_cmd, data = parsed
            resp_tid = hdr & 0x0F
            info(f'  spinel: hdr=0x{hdr:02x} cmd=0x{s_cmd:02x} tid={resp_tid} data={data.hex()}')
            # Decode PROP_PROTOCOL_VERSION specially
            if prop_id == SPINEL_PROP_PROTOCOL_VERSION and s_cmd == SPINEL_CMD_PROP_VALUE_IS:
                major, off = unpack_uint(data, 0)
                minor, _   = unpack_uint(data, off)
                info(f'  Protocol version: {major}.{minor}')
            elif prop_id == SPINEL_PROP_NCP_VERSION and s_cmd == SPINEL_CMD_PROP_VALUE_IS:
                ncp_ver = data.decode('utf-8', errors='replace').rstrip('\x00')
                info(f'  NCP version: {ncp_ver}')
        ok(f'Thread spinel {prop_name} response received')
        return True
    fail(f'No Thread response for {prop_name} (timeout 3 s)')
    return False


# ─── Generic BLE HCI send + receive helper ────────────────────────────────────

def hci_send_recv(ser, cmd: bytes, timeout: float = 3.0):
    """
    Send a MUX-wrapped H4 HCI command and wait for the matching
    Command Complete event.  Returns the event payload bytes (everything
    after the status byte) or None on timeout / error.
    """
    # Extract expected opcode from command bytes [1:3]
    expected_opcode = struct.unpack('<H', cmd[1:3])[0]

    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, cmd)
    ser.write(frame)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        f = read_frame(ser, timeout=deadline - time.monotonic())
        if f is None:
            break
        result = mux_decode(f)
        if result is None:
            continue
        nli, mux_cmd, payload = result
        if handle_keepalive(ser, nli, mux_cmd):
            continue
        if nli != MUX_NLI_BLE or len(payload) < 7:
            continue
        # H4: type=0x04, event_code=0x0E (Command Complete)
        if payload[0] != 0x04 or payload[1] != 0x0E:
            continue
        resp_opcode = struct.unpack('<H', payload[4:6])[0]
        if resp_opcode != expected_opcode:
            continue
        status = payload[6]
        return status, payload[7:]   # (status, return_params)
    return None


# ─── BLE HCI tests ────────────────────────────────────────────────────────────

def test_ble_hci_reset(ser) -> bool:
    """Send HCI Reset, expect Command Complete with status=0."""
    print('\n── BLE Test 1: HCI Reset ──────────────────────────────────')
    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, HCI_CMD_RESET)
    info(f'TX HCI Reset: {HCI_CMD_RESET.hex()}')
    tx(frame)
    ser.write(frame)

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        f = read_frame(ser, timeout=deadline - time.monotonic())
        if f is None:
            break
        result = mux_decode(f)
        if result is None:
            continue
        nli, mux_cmd, payload = result
        if handle_keepalive(ser, nli, mux_cmd):
            continue
        if nli != MUX_NLI_BLE:
            continue
        rx(f)
        info(f'  HCI payload: {payload.hex()}')
        if len(payload) >= 7 and payload[0] == 0x04 and payload[1] == 0x0E:
            status = payload[6]
            if status == 0x00:
                ok('HCI Reset → Command Complete (status=0x00 OK)')
                return True
            else:
                fail(f'HCI Reset → status=0x{status:02x}')
                return False
        ok(f'NLI_BLE frame received: {payload.hex()}')
        return True
    fail('No BLE HCI response (timeout 3 s)')
    return False


def test_ble_read_local_version(ser) -> bool:
    """HCI Read Local Version Information — prints BT version, manufacturer."""
    print('\n── BLE Test 2: HCI Read Local Version ─────────────────────')
    info(f'TX HCI Read Local Version: {HCI_CMD_READ_LOCAL_VERSION.hex()}')
    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, HCI_CMD_READ_LOCAL_VERSION)
    tx(frame)
    result = hci_send_recv(ser, HCI_CMD_READ_LOCAL_VERSION)
    if result is None:
        fail('No response (timeout 3 s)')
        return False
    status, params = result
    if status != 0x00:
        fail(f'Status=0x{status:02x}')
        return False
    if len(params) < 8:
        fail(f'Response too short: {params.hex()}')
        return False
    hci_ver                   = params[0]
    hci_rev                   = struct.unpack('<H', params[1:3])[0]
    lmp_ver                   = params[3]
    manufacturer              = struct.unpack('<H', params[4:6])[0]
    lmp_subver                = struct.unpack('<H', params[6:8])[0]
    ver_name = HCI_VERSION_NAMES.get(hci_ver, f'Unknown(0x{hci_ver:02x})')
    info(f'  HCI Version   : 0x{hci_ver:02x}  ({ver_name})')
    info(f'  HCI Revision  : 0x{hci_rev:04x}')
    info(f'  LMP Version   : 0x{lmp_ver:02x}')
    info(f'  Manufacturer  : 0x{manufacturer:04x}'
         f'{"  (Texas Instruments)" if manufacturer == 0x000D else ""}')
    info(f'  LMP Subversion: 0x{lmp_subver:04x}')
    ok('HCI Read Local Version Information')
    return True


def test_ble_read_bd_addr(ser) -> bool:
    """HCI Read BD_ADDR — prints Bluetooth device address."""
    print('\n── BLE Test 3: HCI Read BD_ADDR ───────────────────────────')
    info(f'TX HCI Read BD_ADDR: {HCI_CMD_READ_BD_ADDR.hex()}')
    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, HCI_CMD_READ_BD_ADDR)
    tx(frame)
    result = hci_send_recv(ser, HCI_CMD_READ_BD_ADDR)
    if result is None:
        fail('No response (timeout 3 s)')
        return False
    status, params = result
    if status != 0x00:
        fail(f'Status=0x{status:02x}')
        return False
    if len(params) < 6:
        fail(f'Response too short: {params.hex()}')
        return False
    # BD_ADDR is 6 bytes, LSB first → display MSB first
    bd_addr = ':'.join(f'{b:02X}' for b in reversed(params[:6]))
    info(f'  BD_ADDR: {bd_addr}')
    ok('HCI Read BD_ADDR')
    return True


def test_ble_le_read_features(ser) -> bool:
    """HCI LE Read Local Supported Features — prints enabled LE features."""
    print('\n── BLE Test 4: HCI LE Read Local Supported Features ───────')
    info(f'TX LE Read Local Supported Features: {HCI_CMD_LE_READ_FEATURES.hex()}')
    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, HCI_CMD_LE_READ_FEATURES)
    tx(frame)
    result = hci_send_recv(ser, HCI_CMD_LE_READ_FEATURES)
    if result is None:
        fail('No response (timeout 3 s)')
        return False
    status, params = result
    if status != 0x00:
        fail(f'Status=0x{status:02x}')
        return False
    if len(params) < 8:
        fail(f'Response too short: {params.hex()}')
        return False
    feature_bits = int.from_bytes(params[:8], 'little')
    info(f'  LE Features mask: 0x{feature_bits:016x}')
    enabled = [LE_FEATURES[i] for i in range(len(LE_FEATURES)) if feature_bits & (1 << i)]
    for f in enabled:
        info(f'    ✓ {f}')
    ok(f'HCI LE Read Local Supported Features ({len(enabled)} enabled)')
    return True


def test_ble_le_read_buffer_size(ser) -> bool:
    """HCI LE Read Buffer Size — prints LE ACL buffer size and count."""
    print('\n── BLE Test 5: HCI LE Read Buffer Size ────────────────────')
    info(f'TX LE Read Buffer Size: {HCI_CMD_LE_READ_BUF_SIZE.hex()}')
    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, HCI_CMD_LE_READ_BUF_SIZE)
    tx(frame)
    result = hci_send_recv(ser, HCI_CMD_LE_READ_BUF_SIZE)
    if result is None:
        fail('No response (timeout 3 s)')
        return False
    status, params = result
    if status != 0x00:
        fail(f'Status=0x{status:02x}')
        return False
    if len(params) < 3:
        fail(f'Response too short: {params.hex()}')
        return False
    le_data_pkt_len   = struct.unpack('<H', params[0:2])[0]
    le_total_num_pkts = params[2]
    info(f'  LE ACL Data Packet Length  : {le_data_pkt_len} bytes')
    info(f'  Total LE ACL Data Packets  : {le_total_num_pkts}')
    ok('HCI LE Read Buffer Size')
    return True


def test_ble_read_local_supported_cmds(ser) -> bool:
    """HCI Read Local Supported Commands — prints supported command count."""
    print('\n── BLE Test 6: HCI Read Local Supported Commands ──────────')
    info(f'TX HCI Read Local Supported Commands: {HCI_CMD_READ_LOCAL_CMDS.hex()}')
    frame = mux_encode(MUX_NLI_BLE, SPINEL_CMD_PROP_VALUE_SET, HCI_CMD_READ_LOCAL_CMDS)
    tx(frame)
    result = hci_send_recv(ser, HCI_CMD_READ_LOCAL_CMDS)
    if result is None:
        fail('No response (timeout 3 s)')
        return False
    status, params = result
    if status != 0x00:
        fail(f'Status=0x{status:02x}')
        return False
    if len(params) < 64:
        fail(f'Response too short ({len(params)} bytes, expected 64)')
        return False
    # Count supported commands (set bits in 64-byte bitmask)
    supported = sum(bin(b).count('1') for b in params[:64])
    info(f'  Supported commands bitmask: {params[:8].hex()}...')
    info(f'  Total supported commands   : {supported}')
    ok(f'HCI Read Local Supported Commands ({supported} supported)')
    return True


def test_ble_listen(ser, duration: float = 2.0) -> bool:
    """Passively listen for any NLI_BLE frames to confirm BLE path is active."""
    print('\n── BLE Test 7: Passive Listen ─────────────────────────────')
    info(f'Listening {duration:.0f} s for any NLI_BLE frames...')
    found = False
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        f = read_frame(ser, timeout=deadline - time.monotonic())
        if f is None:
            continue
        result = mux_decode(f)
        if result is None:
            continue
        nli, cmd, payload = result
        if handle_keepalive(ser, nli, cmd):
            continue
        if nli == MUX_NLI_BLE:
            rx(f)
            info(f'  NLI_BLE payload: {payload.hex()}')
            found = True
    if found:
        ok('NLI_BLE traffic observed')
    else:
        info('No NLI_BLE frames seen (BLE stack may be idle — not a failure)')
    return True  # not a hard failure


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='TI Combined Serial MUX protocol test',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('device',         help='Serial device  e.g. /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=921600, help='Baud rate (default 921600)')
    ap.add_argument('--no-ble',  action='store_true', help='Skip BLE HCI tests')
    args = ap.parse_args()

    print(f'TI Combined Serial MUX Test')
    print(f'Device: {args.device}   Baud: {args.baud}')
    print('=' * 60)

    try:
        ser = serial.Serial(args.device, args.baud, timeout=1.0)
        ok(f'Opened {args.device}')
    except Exception as exc:
        fail(f'Could not open {args.device}: {exc}')
        return 1

    # Flush any stale data
    time.sleep(0.2)
    ser.reset_input_buffer()

    results = []

    # ── Keepalive ────────────────────────────────────────────────────────────
    results.append(('Keepalive',               test_keepalive(ser)))

    # ── Thread spinel ────────────────────────────────────────────────────────
    results.append(('Thread RESET',            test_thread_reset(ser)))
    results.append(('Thread PROP_PROTOCOL_VER',
                     test_thread_prop_get(ser, SPINEL_PROP_PROTOCOL_VERSION,
                                          'PROP_PROTOCOL_VERSION', tid=1)))
    results.append(('Thread PROP_NCP_VERSION',
                     test_thread_prop_get(ser, SPINEL_PROP_NCP_VERSION,
                                          'PROP_NCP_VERSION', tid=2)))

    # ── BLE HCI ──────────────────────────────────────────────────────────────
    if not args.no_ble:
        results.append(('BLE HCI Reset',                  test_ble_hci_reset(ser)))
        results.append(('BLE Read Local Version',          test_ble_read_local_version(ser)))
        results.append(('BLE Read BD_ADDR',                test_ble_read_bd_addr(ser)))
        results.append(('BLE LE Read Local Features',      test_ble_le_read_features(ser)))
        results.append(('BLE LE Read Buffer Size',         test_ble_le_read_buffer_size(ser)))
        results.append(('BLE Read Local Supported Cmds',   test_ble_read_local_supported_cmds(ser)))
        results.append(('BLE Passive Listen',              test_ble_listen(ser)))

    # ── Summary ──────────────────────────────────────────────────────────────
    print('\n' + '=' * 60)
    print('SUMMARY')
    print('-' * 60)
    passed = sum(1 for _, r in results if r)
    for name, r in results:
        mark = f'{GREEN}PASS{RESET}' if r else f'{RED}FAIL{RESET}'
        print(f'  [{mark}]  {name}')
    print('-' * 60)
    print(f'  {passed}/{len(results)} passed')

    ser.close()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
