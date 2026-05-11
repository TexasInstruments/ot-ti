#!/usr/bin/env python3
"""
spinel_bridge.py — Bidirectional Spinel/MUX <-> multi-stack PTY bridge

Bridges BLE HCI, Zigbee, and Thread (OpenThread) stacks between a physical
serial port running the TI Spinel/MUX protocol and per-stack virtual PTYs.

NLI assignments:
    NLI=0 → Thread (OpenThread)   PTY mode: HDLC-framed Spinel (pass-through;
                                             the device's otNcpHdlc layer
                                             already HDLC-frames its content,
                                             so no re-wrapping is done here)
    NLI=1 → BLE HCI                PTY mode: raw HCI H4 — the complete H4
                                             packet (type + fixed header +
                                             variable payload) is carried
                                             unmodified as the Spinel/MUX
                                             payload.
    NLI=2 → Zigbee                PTY mode: HDLC-framed Spinel. Disabled by
                                             default: not every firmware
                                             build wires up MUX_NLI_ZB (e.g.
                                             the rcp_ble_controller example
                                             only registers NLI_OT and
                                             NLI_BLE) — enable explicitly
                                             with --zb once your firmware
                                             actually implements it.
    NLI=3 → Keepalive (ACK'd inline, not forwarded)

Usage:
    source ../.venv/bin/activate
    python spinel_bridge.py [--port /dev/ttyACM0] [--baud 115200]
                            [--no-ble] [--zb] [--no-thread]
                            [--verbose] [--error]
"""

import argparse
import logging
import os
import select
import struct
import sys
import threading
import time
import tty

import serial


# ---------------------------------------------------------------------------
# MUX / Spinel protocol constants  (from mux_common.h)
# ---------------------------------------------------------------------------
HDLC_FLAG        = 0x7E
HDLC_ESCAPE      = 0x7D
HDLC_ESCAPE_MASK = 0x20

SPINEL_HEADER_FLAG = 0x80
SPINEL_NLI_SHIFT   = 4

MUX_NLI_THREAD    = 0   # Thread / OpenThread
MUX_NLI_BLE       = 1   # BLE HCI
MUX_NLI_ZB        = 2   # Zigbee
MUX_NLI_KEEPALIVE = 3

CMD_KEEPALIVE     = 15555   # embedded -> host ping
CMD_KEEPALIVE_ACK = 15556   # host -> embedded ack

SPINEL_CMD_PROP_VALUE_SET = 0x03   # host  -> device  (TX)
SPINEL_CMD_PROP_VALUE_IS  = 0x06   # device -> host   (RX)

# Maximum size of a single frame (MUX/HDLC frame, Spinel-mode PTY HDLC
# frame, or H4 HCI packet) that this bridge will buffer. Anything exceeding
# this without completing is treated as corrupt/desynced and discarded so
# the reader can resync on the next flag/type byte, mirroring the
# firmware's own frame-size cap (MAX_FRAME_SIZE in mux_common.h).
MAX_FRAME_SIZE = 1024

# H4 HCI packet type indicators (BLE PTY mode)
H4_CMD   = 0x01
H4_ACL   = 0x02
H4_SCO   = 0x03
H4_EVENT = 0x04


# ---------------------------------------------------------------------------
# CRC-16 / Kermit  (poly=0x8408, init=0x0000, reflected I/O, no final XOR)
# ---------------------------------------------------------------------------
def crc16_kermit(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 0x0001) else (crc >> 1)
    return crc


# ---------------------------------------------------------------------------
# Spinel packed-uint encode / decode
# ---------------------------------------------------------------------------
def pack_uint(value: int) -> bytes:
    result = bytearray()
    while True:
        b = value & 0x7F
        value >>= 7
        if value:
            b |= 0x80
        result.append(b)
        if not value:
            break
    return bytes(result)


def unpack_uint(data: bytes, offset: int):
    """
    Returns (value, bytes_consumed).

    Raises ValueError on a truncated packed-uint (ran out of data before a
    terminating byte) or one exceeding 5 bytes / 35 bits, mirroring the
    firmware's MuxSpinel_decodeUint() bounds (MUX_ERR_NO_SPACE /
    MUX_ERR_OVERFLOW) instead of raising an uncaught IndexError or accepting
    an unbounded value.
    """
    result, shift, consumed = 0, 0, 0
    while True:
        if offset + consumed >= len(data):
            raise ValueError("Truncated packed-uint")
        if shift >= 35:
            raise ValueError("Packed-uint exceeds 32 bits")
        b = data[offset + consumed]
        consumed += 1
        result |= (b & 0x7F) << shift
        shift  += 7
        if not (b & 0x80):
            break
    return result, consumed


# ---------------------------------------------------------------------------
# HDLC helpers
# ---------------------------------------------------------------------------
def hdlc_escape_bytes(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in (HDLC_FLAG, HDLC_ESCAPE):
            out.append(HDLC_ESCAPE)
            out.append(b ^ HDLC_ESCAPE_MASK)
        else:
            out.append(b)
    return bytes(out)


def hdlc_unescape_bytes(data: bytes) -> bytes:
    out, escape = bytearray(), False
    for b in data:
        if b == HDLC_ESCAPE:
            escape = True
        elif escape:
            out.append(b ^ HDLC_ESCAPE_MASK)
            escape = False
        else:
            out.append(b)
    if escape:
        raise ValueError("Trailing escape byte with no following byte")
    return bytes(out)


# ---------------------------------------------------------------------------
# MUX frame encode / decode
# ---------------------------------------------------------------------------
def mux_encode(nli: int, cmd: int, payload: bytes) -> bytes:
    """Build a complete MUX wire frame."""
    hdr          = SPINEL_HEADER_FLAG | ((nli & 0x03) << SPINEL_NLI_SHIFT)
    spinel_frame = bytes([hdr]) + pack_uint(cmd) + payload
    crc          = crc16_kermit(spinel_frame)
    raw          = spinel_frame + struct.pack('<H', crc)
    return bytes([HDLC_FLAG]) + hdlc_escape_bytes(raw) + bytes([HDLC_FLAG])


def mux_decode(frame: bytes):
    """
    Decode a complete MUX frame (including outer 0x7E flags).
    Returns (nli, cmd, payload) or raises ValueError.
    """
    if len(frame) < 4 or frame[0] != HDLC_FLAG or frame[-1] != HDLC_FLAG:
        raise ValueError("Missing or incomplete HDLC flag bytes")

    raw = hdlc_unescape_bytes(frame[1:-1])

    if len(raw) < 4:
        raise ValueError(f"Frame too short after unescape ({len(raw)} bytes)")

    crc_received = struct.unpack_from('<H', raw, len(raw) - 2)[0]
    spinel_frame = raw[:-2]
    crc_computed = crc16_kermit(spinel_frame)
    if crc_computed != crc_received:
        raise ValueError(
            f"CRC mismatch: received {crc_received:#06x}, computed {crc_computed:#06x}")

    hdr = spinel_frame[0]
    if (hdr & 0xC0) != SPINEL_HEADER_FLAG:
        raise ValueError(f"Invalid Spinel header flags: {hdr:#04x}")

    nli           = (hdr >> SPINEL_NLI_SHIFT) & 0x03
    cmd, consumed = unpack_uint(spinel_frame, 1)
    payload       = spinel_frame[1 + consumed:]
    return nli, cmd, payload


# ---------------------------------------------------------------------------
# Serial helper: read one complete HDLC frame from serial.Serial
#
# Blocks until a full frame is received or stop_event is set — never
# discards partially-received bytes due to an internal timeout of its own
# (there isn't one). ser itself has a short port-level read timeout, so
# stop_event is checked promptly whenever the link is idle. If a frame
# grows past MAX_FRAME_SIZE without a closing flag, it is discarded and the
# reader resyncs by resuming its search for the next opening flag, rather
# than growing the buffer without bound.
# ---------------------------------------------------------------------------
def read_mux_frame(ser: serial.Serial, stop_event: threading.Event) -> bytes:
    log = logging.getLogger('spinel_bridge')
    buf, in_frame = bytearray(), False

    while not stop_event.is_set():
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]

        if byte == HDLC_FLAG:
            if in_frame and len(buf) > 1:
                buf.append(byte)
                return bytes(buf)
            buf      = bytearray([byte])
            in_frame = True
        elif in_frame:
            if len(buf) >= MAX_FRAME_SIZE:
                log.warning(
                    "Frame exceeded MAX_FRAME_SIZE (%d bytes) without a closing "
                    "flag, discarding and resyncing", MAX_FRAME_SIZE)
                buf, in_frame = bytearray(), False
                continue
            buf.append(byte)

    raise EOFError("bridge stopped")


# ---------------------------------------------------------------------------
# PTY helper: read one complete HDLC frame from a raw file descriptor
# Used for Spinel-mode (Zigbee / Thread) PTY -> physical direction.
# Uses select() so the stop_event is checked every 0.5 s. Same
# MAX_FRAME_SIZE resync behavior as read_mux_frame() above.
# ---------------------------------------------------------------------------
def read_hdlc_frame_fd(fd: int, stop_event: threading.Event) -> bytes:
    log = logging.getLogger('spinel_bridge')
    buf, in_frame = bytearray(), False

    while not stop_event.is_set():
        ready, _, _ = select.select([fd], [], [], 0.5)
        if not ready:
            continue

        b = os.read(fd, 1)
        if not b:
            raise EOFError("PTY master fd closed")
        byte = b[0]

        if byte == HDLC_FLAG:
            if in_frame and len(buf) > 1:
                buf.append(byte)
                return bytes(buf)
            buf      = bytearray([byte])
            in_frame = True
        elif in_frame:
            if len(buf) >= MAX_FRAME_SIZE:
                log.warning(
                    "PTY HDLC frame exceeded MAX_FRAME_SIZE (%d bytes) without a "
                    "closing flag, discarding and resyncing", MAX_FRAME_SIZE)
                buf, in_frame = bytearray(), False
                continue
            buf.append(byte)

    raise EOFError("bridge stopped")


# ---------------------------------------------------------------------------
# H4 HCI packet reader from a file descriptor (BLE PTY mode)
#
# Uses select() so stop_event is checked regularly (interruptible on
# shutdown, unlike a plain blocking os.read()). An unrecognized packet-type
# byte or an implausible declared length is treated as a corrupt/desynced
# stream: discarded, and the reader resumes scanning for the next type
# byte, rather than raising and killing the calling thread.
# ---------------------------------------------------------------------------
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


def read_h4_packet_fd(fd: int, stop_event: threading.Event) -> bytes:
    log = logging.getLogger('spinel_bridge')

    def read_byte() -> int:
        while not stop_event.is_set():
            ready, _, _ = select.select([fd], [], [], 0.5)
            if not ready:
                continue
            b = os.read(fd, 1)
            if not b:
                raise EOFError("PTY master fd closed")
            return b[0]
        raise EOFError("bridge stopped")

    def read_n(n: int) -> bytes:
        out = bytearray()
        while len(out) < n:
            out.append(read_byte())
        return bytes(out)

    while True:
        pkt_type = read_byte()
        if pkt_type not in _H4_HEADER_LEN:
            log.warning("H4 unknown packet type %#04x, discarding and resyncing", pkt_type)
            continue   # unknown type byte — resync, keep scanning

        header  = read_n(_H4_HEADER_LEN[pkt_type])
        len_off = _H4_LEN_OFFSET[pkt_type]
        if pkt_type == H4_ACL:
            payload_len = header[len_off] | (header[len_off + 1] << 8)
        else:
            payload_len = header[len_off]

        if payload_len > MAX_FRAME_SIZE:
            log.warning(
                "H4 packet type %#04x declared implausible length %d (> %d), "
                "discarding and resyncing", pkt_type, payload_len, MAX_FRAME_SIZE)
            continue   # implausible length for a desynced stream — resync

        payload = read_n(payload_len)
        return bytes([pkt_type]) + header + payload


# ---------------------------------------------------------------------------
# Per-stack PTY state
# ---------------------------------------------------------------------------
class StackPty:
    def __init__(self, nli: int, name: str, mode: str):
        self.nli        = nli
        self.name       = name    # 'ble', 'zb', 'thread'
        self.mode       = mode    # 'hci' (BLE) or 'spinel' (Zigbee/Thread)
        self.master_fd  = -1
        self.slave_fd   = -1
        self.slave_path = ''


# ---------------------------------------------------------------------------
# SpinelBridge
# ---------------------------------------------------------------------------
class SpinelBridge:
    def __init__(self, port: str, baud: int, stacks: list, verbose: bool = False,
                 error: bool = False):
        self.port    = port
        self.baud    = baud
        self.stacks  = stacks                              # list[StackPty]
        self.nli_map = {s.nli: s for s in stacks}         # NLI -> StackPty
        self._stop   = threading.Event()
        # Guards every ser.write() call: the phys-rx thread (keepalive ACKs)
        # and every pty-rx thread (one per enabled stack) write to the same
        # serial.Serial object concurrently. pyserial's write() is not
        # guaranteed atomic across threads for multi-byte writes, so without
        # this lock two threads writing at once could interleave bytes on
        # the physical UART TX line and corrupt both frames.
        self._ser_lock = threading.Lock()

        # --verbose (DEBUG) > --error / default (INFO, which already includes
        # WARNING and ERROR). --error is an explicit, self-documenting alias
        # for the default level — it exists so a caller can request
        # "info+warning+error" without relying on omitting every other flag.
        level = logging.DEBUG if verbose else logging.INFO
        logging.basicConfig(
            format='%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%H:%M:%S',
            level=level,
            stream=sys.stdout,   # default is stderr, which `tee`/redirection misses
        )
        self.log = logging.getLogger('spinel_bridge')

    # ------------------------------------------------------------------
    def _setup_ptys(self):
        """Create one PTY master/slave pair per enabled stack."""
        for stack in self.stacks:
            master_fd, slave_fd = os.openpty()
            tty.setraw(master_fd)
            stack.master_fd  = master_fd
            stack.slave_fd   = slave_fd          # held open to prevent EIO
            stack.slave_path = os.ttyname(slave_fd)

            # openpty() creates the slave with 0620 (crw--w----).
            # Make it 0666 so that hciattach / btattach / otbr-agent can
            # open it regardless of which user or sudo context they run under.
            os.chmod(stack.slave_path, 0o666)

            self.log.info("[%s] PTY: %s  (NLI=%d, mode=%s)",
                          stack.name, stack.slave_path,
                          stack.nli, stack.mode)
            # Print clearly to stdout so the caller can capture the path
            print(f"{stack.name.upper()} PTY: {stack.slave_path}", flush=True)

    # ------------------------------------------------------------------
    def _physical_rx_loop(self, ser: serial.Serial):
        """
        Thread: physical port -> PTY(s).
        Decodes each Spinel/MUX frame and dispatches the payload to the
        matching stack's PTY master fd.  Keepalive frames are ACK'd inline.
        """
        self.log.info("phys-rx loop started")
        while not self._stop.is_set():
            try:
                raw = read_mux_frame(ser, self._stop)
            except EOFError:
                break
            except Exception as e:
                if not self._stop.is_set():
                    self.log.error("phys-rx read error: %s", e)
                break

            try:
                nli, cmd, payload = mux_decode(raw)
            except ValueError as e:
                self.log.warning("Decode error, skipping frame: %s", e)
                continue

            if nli == MUX_NLI_KEEPALIVE:
                if cmd == CMD_KEEPALIVE:
                    ack = mux_encode(MUX_NLI_KEEPALIVE, CMD_KEEPALIVE_ACK, b'')
                    with self._ser_lock:
                        ser.write(ack)
                    self.log.debug("[KA] keepalive -> ACK sent")
                continue

            stack = self.nli_map.get(nli)
            if stack is None:
                self.log.warning("[SKIP] NLI=%d not configured, discarding %d bytes",
                                  nli, len(payload))
                continue

            self.log.debug("<< PHY [%s] NLI=%d cmd=%d  %d bytes: %s",
                           stack.name, nli, cmd, len(payload), ' '.join('%02x' % b for b in payload))
            # For both modes, payload already carries whatever framing the
            # device-side protocol needs (a complete H4 HCI packet for BLE,
            # a complete HDLC-Spinel frame for Zigbee/Thread) — write as-is.
            try:
                written = os.write(stack.master_fd, payload)
                if written != len(payload):
                    self.log.error(
                        "[%s] PTY short write: wrote %d of %d bytes, frame truncated",
                        stack.name, written, len(payload))
            except OSError as e:
                self.log.error("[%s] PTY write error: %s", stack.name, e)

        self.log.info("phys-rx loop stopped")

    # ------------------------------------------------------------------
    def _pty_rx_loop(self, ser: serial.Serial, stack: StackPty):
        """
        Thread: PTY -> physical port (one thread per stack).

        hci mode    — reads complete H4 HCI packets (BLE), wraps in Spinel/MUX.
        spinel mode — reads HDLC frames from PTY, wraps in MUX unmodified.
        """
        self.log.info("pty-rx [%s] loop started", stack.name)
        while not self._stop.is_set():
            try:
                if stack.mode == 'hci':
                    packet = read_h4_packet_fd(stack.master_fd, self._stop)
                else:
                    # Pass the complete HDLC frame (7e...7e) as the MUX payload.
                    # The device's otNcpHdlcReceive expects full HDLC frames;
                    # stripping the framing here breaks spinel decoding on-device.
                    packet = read_hdlc_frame_fd(stack.master_fd, self._stop)
            except (EOFError, OSError) as e:
                if not self._stop.is_set():
                    self.log.error("pty-rx [%s] read error: %s", stack.name, e)
                break

            self.log.debug(">> PTY [%s] %d bytes: %s",
                           stack.name, len(packet), ' '.join('%02x' % b for b in packet))
            frame = mux_encode(stack.nli, SPINEL_CMD_PROP_VALUE_SET, packet)
            try:
                with self._ser_lock:
                    ser.write(frame)
            except serial.SerialException as e:
                self.log.error("[%s] physical write error: %s", stack.name, e)

        self.log.info("pty-rx [%s] loop stopped", stack.name)

    # ------------------------------------------------------------------
    def run(self):
        self._setup_ptys()
        self.log.info("Opening %s at %d baud", self.port, self.baud)

        with serial.Serial(self.port, self.baud, timeout=0.1) as ser:
            t_phys = threading.Thread(
                target=self._physical_rx_loop, args=(ser,),
                daemon=True, name="phys-rx")
            pty_threads = [
                threading.Thread(
                    target=self._pty_rx_loop, args=(ser, stack),
                    daemon=True, name=f"pty-rx-{stack.name}")
                for stack in self.stacks
            ]

            t_phys.start()
            for t in pty_threads:
                t.start()

            stack_info = "  ".join(f"{s.name} = {s.slave_path}" for s in self.stacks)
            self.log.info("Bridge running.  %s  Press Ctrl-C to stop.", stack_info)

            try:
                while t_phys.is_alive() and all(t.is_alive() for t in pty_threads):
                    time.sleep(0.5)
            except KeyboardInterrupt:
                self.log.info("Shutting down ...")
            finally:
                self._stop.set()
                t_phys.join(timeout=3)
                for t in pty_threads:
                    t.join(timeout=3)
                for stack in self.stacks:
                    for fd in (stack.slave_fd, stack.master_fd):
                        try:
                            os.close(fd)
                        except OSError:
                            pass
                self.log.info("Bridge stopped.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Spinel/MUX <-> multi-stack PTY bridge (BLE / Zigbee / Thread)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--port',       default='/dev/ttyACM0', help='Physical serial port')
    parser.add_argument('--baud',       default=115200, type=int, help='Baud rate')
    parser.add_argument('--no-ble',    action='store_true', help='Disable BLE HCI PTY (NLI=1)')
    parser.add_argument('--zb',        action='store_true',
                         help='Enable Zigbee PTY (NLI=2). Off by default: not every '
                              'firmware build wires up MUX_NLI_ZB (e.g. rcp_ble_controller '
                              'only implements NLI_OT and NLI_BLE) — traffic sent here '
                              'would otherwise be silently dropped by the device.')
    parser.add_argument('--no-thread', action='store_true', help='Disable Thread PTY (NLI=0)')
    parser.add_argument('--verbose',    action='store_true', help='Enable debug logging')
    parser.add_argument('--error',      action='store_true',
                         help='Print info, warning, and error log messages (this is the '
                              'default level; --error requests it explicitly). Overridden '
                              'by --verbose, which also enables debug logging.')
    args = parser.parse_args()

    stacks = []
    if not args.no_ble:
        stacks.append(StackPty(nli=MUX_NLI_BLE,    name='ble',    mode='hci'))
    if args.zb:
        stacks.append(StackPty(nli=MUX_NLI_ZB,     name='zb',     mode='spinel'))
    if not args.no_thread:
        stacks.append(StackPty(nli=MUX_NLI_THREAD, name='thread', mode='spinel'))

    if not stacks:
        parser.error("At least one stack must be enabled.")

    SpinelBridge(
        port    = args.port,
        baud    = args.baud,
        stacks  = stacks,
        verbose = args.verbose,
        error   = args.error,
    ).run()


if __name__ == '__main__':
    main()
