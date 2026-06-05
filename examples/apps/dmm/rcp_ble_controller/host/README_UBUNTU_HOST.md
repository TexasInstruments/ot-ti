# DMM RCP+BLE Controller — Ubuntu PC Host Setup Guide

Step-by-step instructions for setting up and running the host-side tools on an
Ubuntu PC to work with a TI cc27xx device running the DMM RCP+BLE Controller
firmware.  Covers the Combined Serial MUX bridge, BLE HCI validation, Thread
OTBR startup, and Thread network operations via `ot-ctl`.

**Validated on: Ubuntu 22.04 LTS | Python 3.11**

---

## Table of Contents

- [Quick Start](#quick-start)
- [Overview](#overview)
- [Directory Layout](#directory-layout)
- [Prerequisites](#prerequisites)
- [Step 1 — Flash and connect the device](#step-1--flash-and-connect-the-device)
- [Step 2 — Start the MUX bridge](#step-2--start-the-mux-bridge)
- [Step 3 — Attach BlueZ HCI adapter](#step-3--attach-bluez-hci-adapter)
- [Step 4 — Start OTBR agent](#step-4--start-otbr-agent)
- [Step 5 — Verify Thread network with ot-ctl](#step-5--verify-thread-network-with-ot-ctl)
- [Step 6 — Verify BLE connectivity](#step-6--verify-ble-connectivity)
- [Serial MUX Validation Scripts](#serial-mux-validation-scripts)
- [Step 7 — Detach the BLE PTY](#step-7--detach-the-ble-pty)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

Four terminals are needed. Replace `/dev/pts/9` and `/dev/pts/28` with the
actual PTY paths printed by the bridge.

```bash
# ── Terminal 1: MUX bridge (keep running throughout) ──────────────────
cd host/
bash setup.sh
source .venv/bin/activate
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
# Note: BLE PTY = /dev/pts/9 (NLI=1)  Thread PTY = /dev/pts/28 (NLI=0)

# ── Terminal 2: BlueZ HCI adapter ─────────────────────────────────────
sudo hciattach /dev/pts/9 any 921600
hciconfig    # verify: hci0  UP RUNNING

# ── Terminal 3: OTBR agent (stop systemd service first) ───────────────
sudo systemctl stop otbr-agent
sudo systemctl disable otbr-agent
sudo otbr-agent -I wpan0 -B eno1 \
    "spinel+hdlc+uart:///dev/pts/28" \
    trel://eno1 --verbose

# ── Terminal 4: Thread network via ot-ctl ─────────────────────────────
sudo ot-ctl dataset init new
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
# wait ~30 s, then:
sudo ot-ctl state      # Expected: leader

# ── Terminal 2 (continued): BLE scan ──────────────────────────────────
sudo hciconfig                  # note BD Address: XX:XX:XX:XX:XX:XX
bluetoothctl
> select XX:XX:XX:XX:XX:XX
> scan le
```

---

## Overview

The device runs a single-UART Combined Serial MUX that multiplexes:

| NLI | Protocol | Consumer |
|-----|----------|----------|
| 0 | Thread Spinel | `otbr-agent` |
| 1 | BLE HCI (H4) | BlueZ / `hciattach` |
| 2 | Zigbee Spinel | not used in this test suite (PTY created, no consumer) |
| 3 | Keepalive | bridge internal |

The `spinel_bridge.py` daemon demultiplexes the physical UART into per-stack
PTYs. All tools connect to those PTYs, not to `/dev/ttyACM0` directly.

---

## Directory Layout

```
host/
├── spinel_bridge/
│   ├── spinel_bridge.py      # MUX bridge daemon (Thread + BLE + Zigbee PTYs)
│   └── DESIGN.md             # Bridge design document
├── setup.sh                  # Python venv setup
├── requirements.txt          # Venv dependencies
├── objects/                  # Offline packages for BeagleBone Black
│   ├── pyserial-3.5-py2.py3-none-any.whl
│   └── python3-serial_3.5-1_all.deb
├── README_UBUNTU_HOST.md     # This file
└── README_BBB_HOST.md        # BeagleBone Black setup guide
```

---

## Prerequisites

### System packages

```bash
sudo apt install bluez python3-venv
```

`hciconfig` and `hciattach` are included in the `bluez` package.

### Python virtual environment

```bash
cd host/
bash setup.sh
source .venv/bin/activate
```

### OTBR agent (`otbr-agent`)

Build and install `otbr-agent` from source following the official OpenThread guides:

1. Get the OTBR source code:
   <https://openthread.io/guides/border-router/build-native#get-otbr-code>

2. Build and install OTBR:
   <https://openthread.io/guides/border-router/build-native#build-and-install-otbr>

After installation the build script registers a systemd service that starts
`otbr-agent` automatically. This must be stopped before running it manually
from the command line (see [Step 4](#step-4--start-otbr-agent)).

---

## Step 1 — Flash and connect the device

Flash `ot-rcp-ble-controller.out` to the cc27xx device using UniFlash.
Connect the board via USB (XDS110 cable). Identify the application UART:

```bash
ls /dev/ttyACM*
# Typically /dev/ttyACM0 (application) and /dev/ttyACM1 (XDS110 debug)
```

Confirm which interface is the application UART:

```bash
udevadm info /dev/ttyACM0 | grep ID_USB_INTERFACE_NUM
# Interface 00 = Application UART  (use this for --port)
# Interface 03 = XDS110 Auxiliary UART
```

---

## Step 2 — Start the MUX bridge

The bridge owns `/dev/ttyACM0` and creates one PTY per stack.
Keep this running in a dedicated terminal.

```bash
cd host/
source .venv/bin/activate
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
```

Sample output:

```
10:19:40 [INFO] [ble]    PTY: /dev/pts/9   (NLI=1, mode=hci)
10:19:40 [INFO] [zb]     PTY: /dev/pts/26  (NLI=2, mode=spinel)
10:19:40 [INFO] [thread] PTY: /dev/pts/28  (NLI=0, mode=spinel)
10:19:40 [INFO] Bridge running.  ble=/dev/pts/9  zb=/dev/pts/26  thread=/dev/pts/28
```

Note the PTY paths — use them in all subsequent steps.

To disable stacks you don't need:

```bash
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600 --no-zb
```

To run with verbose debug logging:

```bash
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600 --verbose
```

---

## Step 3 — Attach BlueZ HCI adapter

Use the BLE PTY path printed by the bridge (e.g. `/dev/pts/9`):

```bash
sudo hciattach /dev/pts/9 any 921600
# Output: Device setup complete
```

Verify the adapter is up:

```bash
hciconfig
# Expected: hci0  Type: Primary  Bus: UART  ...  UP RUNNING
```

> **Note on symlinks:** Do not create symlinks under `/tmp` for the PTY and then
> pass them to `sudo hciattach` — Linux's `fs.protected_symlinks` blocks
> privileged processes from following symlinks in sticky directories.
> Always use the direct `/dev/pts/N` path with `sudo hciattach`.

---

## Step 4 — Start OTBR agent

The OTBR build script installs a systemd service that starts `otbr-agent` at
boot and owns the radio interface. Stop and disable it before running
`otbr-agent` manually, otherwise the two instances will conflict:

```bash
sudo systemctl stop otbr-agent
sudo systemctl disable otbr-agent
```

Verify the service is no longer running:

```bash
systemctl is-active otbr-agent
# Expected: inactive
```

Now start `otbr-agent` manually using the Thread PTY path printed by the bridge
(e.g. `/dev/pts/28`). Replace `eno1` with your Ethernet interface name:

```bash
sudo otbr-agent -I wpan0 -B eno1 \
    "spinel+hdlc+uart:///dev/pts/28" \
    trel://eno1 --verbose
```

To re-enable the service after testing:

```bash
sudo systemctl enable otbr-agent
sudo systemctl start otbr-agent
```

---

## Step 5 — Verify Thread network with ot-ctl

With `otbr-agent` running, use `ot-ctl` to interact with the Thread stack.
`ot-ctl` requires `sudo`.

### 5a. Check current state

```bash
sudo ot-ctl state
# Expected: disabled  (no Thread dataset configured yet)
```

### 5b. Form a new Thread network

```bash
sudo ot-ctl dataset init new
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
```

Wait approximately 30 seconds for leader election, then verify:

```bash
sudo ot-ctl state      # Expected: leader
sudo ot-ctl ipaddr     # List assigned IPv6 addresses
sudo ot-ctl extaddr    # Show EUI-64 extended address
sudo ot-ctl channel    # Show active channel
sudo ot-ctl panid      # Show PAN ID
```

### 5c. Useful `ot-ctl` commands

| Command | Description |
|---------|-------------|
| `sudo ot-ctl state` | Thread role: `disabled` / `detached` / `leader` / `router` / `child` |
| `sudo ot-ctl ipaddr` | List all IPv6 addresses |
| `sudo ot-ctl extaddr` | Extended (EUI-64) address |
| `sudo ot-ctl channel` | Active 802.15.4 channel |
| `sudo ot-ctl panid` | PAN ID |
| `sudo ot-ctl networkname` | Thread network name |
| `sudo ot-ctl neighbor table` | Neighbor/child devices |
| `sudo ot-ctl thread stop` | Stop the Thread stack |
| `sudo ot-ctl ifconfig down` | Bring Thread interface down |

**Passwordless sudo for `ot-ctl`** (optional):

```bash
echo "$USER ALL=(ALL) NOPASSWD: /usr/bin/ot-ctl" | sudo tee /etc/sudoers.d/ot-ctl
sudo chmod 440 /etc/sudoers.d/ot-ctl
```

---

## Step 6 — Verify BLE connectivity

With `hciattach` running (Step 3), verify the BLE adapter and scan for devices.

### 6a. Find BD address of HCI adapter

```bash
sudo hciconfig
# hci0:   Type: Primary  Bus: UART
#         BD Address: XX:XX:XX:XX:XX:XX  ACL MTU: 255:5  SCO MTU: 0:0
#         UP RUNNING
```

### 6b. Scan for BLE devices

```bash
bluetoothctl
> select XX:XX:XX:XX:XX:XX
> scan le
```

### 6c. Connect to a BLE peripheral

```bash
bluetoothctl
> connect XX:XX:XX:XX:XX:XX
```

---

## Serial MUX Validation Scripts

The following Python scripts provide low-level validation of the MUX bridge
paths for both Thread and BLE, independently of `otbr-agent` and BlueZ.
Run them after the bridge is up (Step 2) and before starting `otbr-agent`.

Recommended validation order:

```
spinel_bridge.py  →  ble_hci_test.py  →  bluez_hci_validate.py  →  otbr-agent
```

> These scripts operate directly on the PTYs. Stop `hciattach` before running
> `ble_hci_test.py` on the BLE PTY to avoid port conflicts.

### BLE HCI Validator (`ble_hci_test.py`)

Sends H4 HCI commands (`HCI Reset`, `Read Local Version`, `Read BD_ADDR`)
directly to the BLE PTY printed by the bridge (e.g. `/dev/pts/9`):

```bash
cd host/
source .venv/bin/activate
python ble_hci_test.py /dev/pts/9 --baud 921600
```

Expected output:

```
BLE HCI Connection Validator
  Port    : /dev/pts/9
────────────────────────────────────────────────────
  [✓] HCI Reset
  [✓] Read Local Version — ... manufacturer=0x000D (Texas Instruments) ...
  [✓] Read BD_ADDR — XX:XX:XX:XX:XX:XX
────────────────────────────────────────────────────
Result: 3/3 passed  — ALL PASS
```

### Comprehensive BlueZ Validation (`bluez_hci_validate.py`)

Runs 15 tests via the BlueZ D-Bus API. Requires BlueZ to be managing `hci0`
(Step 3 must be complete before running this script).

> **Must use system Python** (`/usr/bin/python3`) for D-Bus access — the
> venv's Python 3.11 cannot build `dbus-python`.

```bash
# All 15 tests, no peripheral needed
sudo /usr/bin/python3 bluez_hci_validate.py

# All 15 tests including connection tests (Tests 10–13)
sudo /usr/bin/python3 bluez_hci_validate.py --target AA:BB:CC:DD:EE:FF

# Tune scan / connection timeouts
sudo /usr/bin/python3 bluez_hci_validate.py --scan-timeout 15 --conn-timeout 20

# Skip connection tests explicitly
sudo /usr/bin/python3 bluez_hci_validate.py --no-conn
```

---

## Step 7 — Detach the BLE PTY

When done testing or before reattaching to a different PTY:

### 7a. Bring the interface down

```bash
sudo hciconfig hci0 down
```

### 7b. Kill the `hciattach` process

```bash
sudo pkill hciattach
```

Verify it is gone:

```bash
hciconfig        # should show nothing (or only other adapters)
pgrep hciattach  # should return empty
```

### 7c. Full detach one-liner

```bash
sudo hciconfig hci0 down && sudo pkill hciattach
```

### 7d. Reattach to a new PTY

```bash
BLE_PTY=/dev/pts/12   # replace with actual path from bridge output

sudo hciconfig hci0 down 2>/dev/null; sudo pkill hciattach 2>/dev/null
sleep 1
sudo hciattach $BLE_PTY any 921600
```

> Always bring the interface down before killing `hciattach` to allow BlueZ to
> cleanly unregister the adapter. If `hci0` still appears as a stale entry after
> an ungraceful kill: `sudo systemctl restart bluetooth`

---

## Troubleshooting

### `hciattach: Can't open serial port: Permission denied`

Use the direct PTY path, not a `/tmp` symlink:

```bash
sudo hciattach /dev/pts/9 any 921600    # correct
sudo hciattach /tmp/ble_hci any 921600  # fails — symlink in sticky /tmp
```

### `otbr-agent` exits silently after printing the Radio URL line

**Symptom:**

```
otbr-agent[455830]: [NOTE]-AGENT---: Radio URL: spinel+hdlc+uart:///dev/pts/28
# <-- exits here with no error
```

**Cause:** The Thread PTY path passed to `otbr-agent` does not exist or is stale.
This happens when `spinel_bridge.py` was restarted and allocated a different
`/dev/pts/N` number.

**Fix:**

1. Confirm the bridge is running and note the Thread PTY it printed.
2. Verify the PTY exists: `ls -la /dev/pts/`
3. If the number is missing, restart the bridge and use the new Thread PTY:

```bash
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
# Note the new THREAD PTY path, e.g. /dev/pts/31

sudo otbr-agent -I wpan0 -B eno1 \
    "spinel+hdlc+uart:///dev/pts/31" \
    trel://eno1 --verbose
```

### `otbr-agent` exits immediately after startup (HDLC framing error)

Ensure you are running the latest `spinel_bridge.py`, which passes complete
HDLC frames as the MUX payload.

### `ot-ctl: connect session failed: Permission denied`

`ot-ctl` must be run with `sudo`. Add a `NOPASSWD` rule to avoid repeated
password prompts (see Step 5c).

### BLE HCI no response when connecting directly to `/dev/ttyACM0`

The device speaks the Combined Serial MUX protocol — raw H4 frames sent
directly to the UART are not understood. Always go through `spinel_bridge.py`
and use the printed PTY paths.

### Keepalive watchdog: bridge exits after 15 s

The device sends `CMD_KEEPALIVE` every 5 s. If the bridge misses 3 consecutive
keepalives it exits. Verify the device firmware is running and the baud rate
matches (921600).

### `ModuleNotFoundError: No module named 'serial'`

```bash
source .venv/bin/activate
pip install pyserial
```
