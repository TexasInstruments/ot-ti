# spinel_bridge — Design Document

## Purpose

`spinel_bridge.py` is a bidirectional protocol bridge that sits between:

- A **physical serial port** (`/dev/ttyACM0`) running the TI Spinel/MUX transport
- **Per-stack virtual PTYs** that expose each stack's native interface to host consumers

This allows BLE, Zigbee, and Thread host stacks to communicate with the embedded
device independently, without any knowledge of the Spinel/MUX multiplexing layer.

---

## System Architecture

```
BLE consumer          /tmp/ble_hci  ──┐
(HCI H4 format)                       │
                                       ├── spinel_bridge.py ── /dev/ttyACM0 ── Embedded
Zigbee consumer       /tmp/zb    ──┤                           (Spinel/MUX)    device
(HDLC Spinel format)               │
                                   │
Thread consumer       /tmp/thread ─┘
(HDLC Spinel format)
```

The bridge runs:
- **1 × `phys-rx` thread** — reads Spinel/MUX frames from the serial port, dispatches each frame to the correct stack's PTY master fd
- **N × `pty-rx-{name}` threads** — one per enabled stack; reads from the stack's PTY and writes Spinel/MUX frames to the serial port

---

## NLI Assignments

| NLI | Stack | PTY mode | Notes |
|---|---|---|---|
| 0 | Thread (OpenThread) | HDLC-framed Spinel | Standard NCP UART interface |
| 1 | BLE HCI | Raw HCI H4 | Standard HCI UART H4 transport |
| 2 | Zigbee | HDLC-framed Spinel | Standard NCP UART interface |
| 3 | Keepalive | — | ACK'd inline; never forwarded to a PTY |

---

## Protocol Stack

### Physical side — Spinel/MUX over HDLC

```
[ 0x7E ][ escaped( spinel_hdr | packed_cmd | stack_payload | crc_lo | crc_hi ) ][ 0x7E ]
```

| Layer | Details |
|---|---|
| HDLC framing | `0x7E` start/end flags; `0x7D` escape byte with XOR `0x20` mask |
| CRC | CRC-16/Kermit (poly `0x8408`, init `0x0000`, reflected I/O, no final XOR) |
| Spinel header | 1 byte: `bits[7:6]=0b10` (flag), `bits[5:4]=NLI`, `bits[3:0]=0` |
| Command | Packed-uint (varint): `PROP_VALUE_SET=0x03` (host→device), `PROP_VALUE_IS=0x06` (device→host) |
| Stack payload | Raw HCI H4 bytes (BLE) or raw Spinel content without HDLC (Zigbee/Thread) |

### PTY side — HCI mode (BLE)

```
[ 1-byte type ] [ fixed header ] [ variable payload ]
```

| Type | Indicator | Fixed header | Payload length field |
|---|---|---|---|
| HCI Command | `0x01` | opcode(2) + param_len(1) | `header[2]` |
| HCI ACL Data | `0x02` | handle(2) + data_len(2)  | `header[2..3]` LE |
| HCI SCO Data | `0x03` | handle(2) + data_len(1)  | `header[2]` |
| HCI Event    | `0x04` | event_code(1) + param_len(1) | `header[1]` |

### PTY side — Spinel mode (Zigbee / Thread)

```
[ 0x7E ][ escaped( spinel_content | crc_lo | crc_hi ) ][ 0x7E ]
```

Identical to the standalone HDLC-Spinel NCP UART interface. This is what consumers
like `otbr-agent` (Thread) and `zigbeed` (Zigbee) expect on a serial PTY.

The bridge re-adds HDLC framing when forwarding physical→PTY, and strips it when
forwarding PTY→physical (where the MUX provides its own HDLC layer).

---

## Module Structure

```
spinel_bridge/
├── spinel_bridge.py     # Bridge implementation
└── DESIGN.md            # This document
```

### Key symbols

| Symbol | Purpose |
|---|---|
| `crc16_kermit()` | CRC-16/Kermit over a byte sequence |
| `pack_uint()` / `unpack_uint()` | Spinel packed-uint (varint) encode/decode |
| `hdlc_escape_bytes()` / `hdlc_unescape_bytes()` | HDLC byte stuffing |
| `mux_encode()` / `mux_decode()` | Build / parse a Spinel/MUX wire frame |
| `read_mux_frame()` | Read one complete HDLC frame from `serial.Serial` |
| `read_hdlc_frame_fd()` | Read one complete HDLC frame from a raw fd (PTY, Spinel mode) |
| `spinel_to_hdlc()` | Re-wrap raw Spinel content in standalone HDLC for NCP consumers |
| `read_h4_packet()` | Read one complete H4 HCI packet from a raw fd (PTY, HCI mode) |
| `StackPty` | Per-stack state: NLI, name, PTY link path, mode, master/slave fds |
| `SpinelBridge` | Main bridge class: PTY setup, I/O threads, lifecycle |
| `main()` | Argument parsing and entry point |

---

## Keepalive Handling

Keepalive frames on NLI=3 (`CMD_KEEPALIVE = 15555`) are ACK'd inline in the
`phys-rx` thread with `CMD_KEEPALIVE_ACK = 15556` and never forwarded to any PTY.

---

## Virtual PTY Details

`SpinelBridge._setup_ptys()` calls `os.openpty()` once per enabled stack:

- The **master fd** is held by the bridge for I/O.
- The **slave fd** is kept open by the bridge to prevent `EIO` on the master when
  no consumer has connected yet (Linux returns `EIO` if no holder on the slave side).
- The **slave path** (e.g., `/dev/pts/4`) is exposed via a stable symlink.
- Master fds are put into raw mode (`tty.setraw`) so bytes pass unmodified.
- All symlinks are removed on clean shutdown.

---

## Threading Model

```
main thread
  └── SpinelBridge.run()
        ├── _setup_ptys()          one PTY per enabled stack
        ├── serial.Serial open
        ├── Thread(phys-rx)        _physical_rx_loop(ser)
        ├── Thread(pty-rx-ble)     _pty_rx_loop(ser, ble_stack)
        ├── Thread(pty-rx-zb)      _pty_rx_loop(ser, zb_stack)   [if enabled]
        ├── Thread(pty-rx-thread)  _pty_rx_loop(ser, thread_stack) [if enabled]
        └── monitors all threads; Ctrl-C sets _stop event
```

`read_hdlc_frame_fd` uses `select()` with a 0.5 s timeout so Spinel-mode PTY
threads check `_stop` regularly and exit promptly on shutdown.

---

## Dependencies

| Package | Version | Source |
|---|---|---|
| `pyserial` | ≥ 3.5 | `requirements.txt` (already present) |

No new dependencies required. Use the existing virtual environment:

```bash
source ../.venv/bin/activate
```

---

## Usage

```bash
source .venv/bin/activate

# BLE only (backward-compatible default)
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0

# BLE + Thread
python spinel_bridge/spinel_bridge.py \
    --ble-pty /tmp/ble_hci \
    --thread-pty /tmp/thread \
    --verbose

# All three stacks
python spinel_bridge/spinel_bridge.py \
    --ble-pty /tmp/ble_hci \
    --zb-pty  /tmp/zb \
    --thread-pty /tmp/thread \
    --verbose
```

### CLI Options

| Option | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyACM0` | Physical serial port (Spinel/MUX side) |
| `--baud` | `115200` | Serial baud rate |
| `--ble-pty PATH` | `/tmp/ble_hci` | BLE HCI PTY symlink (NLI=1, H4 mode); omit to disable |
| `--zb-pty PATH` | disabled | Zigbee PTY symlink (NLI=2, Spinel mode) |
| `--thread-pty PATH` | disabled | Thread PTY symlink (NLI=0, Spinel mode) |
| `--verbose` | off | Enable DEBUG-level logging |

---

## Data Flow Examples

### Host sends HCI Reset (BLE)

```
PTY consumer writes:  01 03 0C 00
  pty-rx-ble reads complete H4 Command packet (4 bytes)
  mux_encode(NLI=1, cmd=0x03, payload=01 03 0C 00)
  Writes Spinel/MUX frame to /dev/ttyACM0
```

### Device sends HCI Command Complete (BLE)

```
/dev/ttyACM0 receives Spinel frame NLI=1
  phys-rx decodes, extracts payload: 04 0E 04 01 03 0C 00
  mode=hci → os.write(ble.master_fd, payload)
  PTY consumer reads: 04 0E 04 01 03 0C 00
```

### Device sends Thread NCP Spinel frame

```
/dev/ttyACM0 receives Spinel frame NLI=0
  phys-rx decodes, extracts Spinel content (no HDLC)
  mode=spinel → spinel_to_hdlc(content) → re-add HDLC framing + CRC
  os.write(thread.master_fd, hdlc_frame)
  otbr-agent reads standard HDLC-Spinel NCP frame from /tmp/thread
```

### Host Thread stack sends Spinel frame

```
otbr-agent writes HDLC-framed Spinel to /tmp/thread
  pty-rx-thread reads one complete HDLC frame via read_hdlc_frame_fd()
  Unescapes, strips CRC → raw Spinel content
  mux_encode(NLI=0, cmd=0x03, spinel_content)
  Writes Spinel/MUX frame to /dev/ttyACM0
```
