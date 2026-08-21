# DMM NCP+BLE Controller — Ubuntu PC Host Setup Guide

Step-by-step instructions for setting up and running the host-side tools on an Ubuntu PC to work with a TI cc27xx device running the **DMM NCP+BLE Controller** firmware. Covers the Combined Serial MUX bridge, BLE HCI validation, Thread OTBR startup, and Thread network operations via `ot-ctl`.

**Validated on: Ubuntu 22.04 LTS | Python 3.11**

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Overview](#overview)
3. [Directory Layout](#directory-layout)
4. [Prerequisites](#prerequisites)
   - [System packages](#system-packages)
   - [Python virtual environment](#python-virtual-environment)
   - [OTBR agent](#otbr-agent)
5. [Step 1 — Flash and connect the device](#step-1--flash-and-connect-the-device)
6. [Step 2 — Start the MUX bridge](#step-2--start-the-mux-bridge)
7. [Step 3 — Attach BlueZ HCI adapter](#step-3--attach-bluez-hci-adapter)
8. [Step 4 — Start OTBR agent](#step-4--start-otbr-agent)
9. [Step 5 — Verify Thread network with ot-ctl](#step-5--verify-thread-network-with-ot-ctl)
10. [Step 6 — Run BLE HCI tests](#step-6--run-ble-hci-tests)
11. [Step 7 — Detach the BLE PTY](#step-7--detach-the-ble-pty)
12. [Troubleshooting](#troubleshooting)

---

## Quick Start

Four terminals are needed. Replace `/dev/pts/9` and `/dev/pts/28` with the actual PTY paths printed by the bridge.

**Terminal 1: MUX bridge (keep running throughout)**

```bash
bash setup.sh
source .venv/bin/activate
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
# Note: BLE PTY = /dev/pts/9 (NLI=1)  Thread PTY = /dev/pts/28 (NLI=0)
```

**Terminal 2: BlueZ HCI adapter**

```bash
sudo hciattach /dev/pts/9 any 921600
hciconfig    # verify: hci0  UP RUNNING
```

**Terminal 3: OTBR agent (stop systemd service first)**

```bash
sudo systemctl stop otbr-agent
sudo systemctl disable otbr-agent
sudo otbr-agent -I wpan0 -B eno1 \
    "spinel+hdlc+uart:///dev/pts/28" \
    trel://eno1 --verbose
```

**Terminal 4: Thread network via ot-ctl**

```bash
sudo ot-ctl dataset init new
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
# wait ~30 s, then:
sudo ot-ctl state      # Expected: leader
```

**Terminal 2 (continued): BLE HCI tests**

```bash
source .venv/bin/activate
python ble_hci_test.py /dev/pts/9 --baud 921600
```

---

## Overview

The device runs a single-UART **Combined Serial MUX** that multiplexes:

| NLI | Protocol       | Consumer                                      |
|-----|----------------|-----------------------------------------------|
| 0   | Thread spinel  | `otbr-agent`                                  |
| 1   | BLE HCI (H4)   | BlueZ / `hciattach`                           |
| 2   | Zigbee spinel  | not used in this test suite (PTY created, no consumer) |
| 3   | Keepalive      | daemon internal                               |

The `spinel_bridge.py` daemon demultiplexes the physical UART into per-stack PTYs. All tools connect to those PTYs, not to `/dev/ttyACM0` directly.

---

## Directory Layout

```
ble_hci_host_test_tool/
├── spinel_bridge/
│   └── spinel_bridge.py      # MUX bridge daemon (Thread + BLE + Zigbee PTYs)
├── ble_hci_test.py            # Basic BLE HCI connectivity validator (H4 direct)
├── bluez_hci_validate.py      # Comprehensive BlueZ D-Bus validation (15 tests)
├── setup.sh                   # Python venv setup
├── requirements.txt           # Venv dependencies
└── README.md
```

---

## Prerequisites

### System packages

```bash
sudo apt install bluez python3-dbus python3-gi
```

(`hciconfig` and `hciattach` are included in the `bluez` package.)

### Python virtual environment

```bash
bash setup.sh
source .venv/bin/activate
```

### OTBR agent (otbr-agent)

Build and install `otbr-agent` from source following the official OpenThread guides:

1. Get the OTBR source code:
   https://openthread.io/guides/border-router/build-native#get-otbr-code

2. Build and install OTBR:
   https://openthread.io/guides/border-router/build-native#build-and-install-otbr

After installation the build script registers a `systemd` service that starts `otbr-agent` automatically. This must be stopped before running it manually from the command line (see Step 4).

---

## Step 1 — Flash and connect the device

Flash `ot-ncp-ble-controller.out` to the cc27xx device using UniFlash. Connect the board via USB (XDS110 cable). Identify the application UART:

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

The bridge owns `/dev/ttyACM0` and creates one PTY per stack. **Keep this running in a dedicated terminal.**

```bash
source .venv/bin/activate
python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
```

Sample output:

```
10:19:40 [INFO] [ble]    PTY: /dev/pts/9   (NLI=1, mode=hci)
BLE PTY: /dev/pts/9
10:19:40 [INFO] [zb]     PTY: /dev/pts/26  (NLI=2, mode=spinel)
ZB PTY: /dev/pts/26
10:19:40 [INFO] [thread] PTY: /dev/pts/28  (NLI=0, mode=spinel)
THREAD PTY: /dev/pts/28
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

Use the **BLE PTY path** printed by the bridge (e.g. `/dev/pts/9`):

```bash
sudo hciattach /dev/pts/9 any 921600
# Output: Device setup complete
```

Verify the adapter is up:

```bash
hciconfig
# Expected: hci0  UP RUNNING
```

> **Note on symlinks:** Do not create symlinks under `/tmp` for the PTY and then pass them to `sudo hciattach` — Linux's `fs.protected_symlinks` blocks privileged processes from following symlinks in sticky directories. Always use the direct `/dev/pts/N` path with `sudo hciattach`.

---

## Step 4 — Start OTBR agent

The OTBR build script installs a `systemd` service that starts `otbr-agent` at boot and owns the radio interface. Stop and disable it **before** running `otbr-agent` manually, otherwise the two instances will conflict:

```bash
sudo systemctl stop otbr-agent
sudo systemctl disable otbr-agent
```

Verify the service is no longer running:

```bash
systemctl is-active otbr-agent
# Expected: inactive
```

Now start `otbr-agent` manually using the **Thread PTY path** printed by the bridge (e.g. `/dev/pts/28`):

```bash
sudo otbr-agent -I wpan0 -B eno1 \
    "spinel+hdlc+uart:///dev/pts/28" \
    trel://eno1 --verbose
```

> To re-enable the service after testing:
> ```bash
> sudo systemctl enable otbr-agent
> sudo systemctl start otbr-agent
> ```

---

## Step 5 — Verify Thread network with ot-ctl

With `otbr-agent` running, use `ot-ctl` to interact with the Thread stack from the command line. `ot-ctl` communicates with `otbr-agent` through a Unix socket and requires `sudo`.

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

### 5c. Useful ot-ctl commands

| Command | Description |
|---------|-------------|
| `sudo ot-ctl state` | Thread role: disabled / detached / leader / router / child |
| `sudo ot-ctl ipaddr` | List all IPv6 addresses |
| `sudo ot-ctl extaddr` | Extended (EUI-64) address |
| `sudo ot-ctl channel` | Active 802.15.4 channel |
| `sudo ot-ctl panid` | PAN ID |
| `sudo ot-ctl networkname` | Thread network name |
| `sudo ot-ctl neighbor table` | Neighbor/child devices |
| `sudo ot-ctl thread stop` | Stop the Thread stack |
| `sudo ot-ctl ifconfig down` | Bring Thread interface down |

> **Passwordless sudo for ot-ctl:** To avoid entering a password on every call, add a `NOPASSWD` rule:
> ```bash
> echo "$USER ALL=(ALL) NOPASSWD: /usr/bin/ot-ctl" | sudo tee /etc/sudoers.d/ot-ctl
> sudo chmod 440 /etc/sudoers.d/ot-ctl
> ```

---

## Step 6 — Run BLE HCI tests

### 6a. Basic connectivity test (ble_hci_test.py)

Sends H4 HCI commands directly to the **BLE PTY** (e.g. `/dev/pts/9`) printed by the bridge in Step 2:

```bash
source .venv/bin/activate
python ble_hci_test.py /dev/pts/9 --baud 921600
```

Tests run: HCI Reset, Read Local Version, Read BD_ADDR.

### 6b. Comprehensive BlueZ validation (bluez_hci_validate.py)

Runs 15 tests via BlueZ D-Bus API. Requires BlueZ to be managing `hci0` (Step 3 must be complete).

> **Must use system Python** (`/usr/bin/python3`) for D-Bus access — the venv's custom Python 3.11 cannot build `dbus-python`.

All 15 tests, no peripheral needed:

```bash
sudo /usr/bin/python3 bluez_hci_validate.py
```

All 15 tests including connection tests (Tests 10–13):

```bash
sudo /usr/bin/python3 bluez_hci_validate.py --target AA:BB:CC:DD:EE:FF
```

Tune scan / connection timeouts:

```bash
sudo /usr/bin/python3 bluez_hci_validate.py --scan-timeout 15 --conn-timeout 20
```

Skip connection tests explicitly:

```bash
sudo /usr/bin/python3 bluez_hci_validate.py --no-conn
```

---

## Step 7 — Detach the BLE PTY

When done testing or before reattaching to a different PTY:

### 7a. Bring the interface down

```bash
sudo hciconfig hci0 down
```

### 7b. Kill the hciattach process

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
# Check new PTY path from bridge output
BLE_PTY=/dev/pts/12   # replace with actual path

sudo hciconfig hci0 down 2>/dev/null; sudo pkill hciattach 2>/dev/null
sleep 1
sudo hciattach $BLE_PTY any 921600
```

Notes:

- Always bring the interface **down before killing** `hciattach` to allow BlueZ to cleanly unregister the adapter.
- If `bluetoothd` is running it will automatically remove `hci0` once `hciattach` exits.
- If `hci0` still appears as a stale entry after an ungraceful kill:
  ```bash
  sudo systemctl restart bluetooth
  ```

---

## Troubleshooting

### hciattach: Can't open serial port: Permission denied

Use the direct PTY path, not a `/tmp` symlink:

```bash
sudo hciattach /dev/pts/9 any 921600    # correct
sudo hciattach /tmp/ble_hci any 921600  # fails — symlink in sticky /tmp
```

### otbr-agent exits silently after printing the Radio URL line

**Symptom:** `otbr-agent` starts, prints the Radio URL, then exits without an error message:

```
otbr-agent[455830]: [NOTE]-AGENT---: Running 0.3.0-a3af469
otbr-agent[455830]: [NOTE]-AGENT---: Thread version: 1.4.0
otbr-agent[455830]: [NOTE]-AGENT---: Thread interface: wpan0
otbr-agent[455830]: [NOTE]-AGENT---: Radio URL: spinel+hdlc+uart:///dev/pts/28
# <-- exits here
```

**Cause:** The Thread PTY path passed to `otbr-agent` does not exist or is stale. This happens when:

- The `spinel_bridge.py` was restarted and allocated a different `/dev/pts/N` number.
- The bridge is not running at all.
- A wrong PTY path was copied (e.g. the BLE PTY was used instead of the Thread PTY).

**Debug steps:**

1. Confirm the bridge is running and note the **THREAD** PTY path it printed:

   ```bash
   # In the bridge terminal you should see a line like:
   THREAD PTY: /dev/pts/28
   ```

2. Verify the PTY exists on the filesystem:

   ```bash
   ls -la /dev/pts/
   # The number printed by the bridge must appear in this list
   ```

3. If the number is missing, the bridge has stopped or restarted. Re-run the bridge and use the newly printed Thread PTY path:

   ```bash
   python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
   # Note the new THREAD PTY path, e.g. /dev/pts/31

   sudo otbr-agent -I wpan0 -B eno1 \
       "spinel+hdlc+uart:///dev/pts/31" \
       trel://eno1 --verbose
   ```

### otbr-agent exits immediately after startup (HDLC framing error)

Ensure you are running the latest `spinel_bridge.py` which passes complete HDLC frames as the MUX payload (earlier versions stripped HDLC before wrapping).

### ot-ctl: connect session failed: Permission denied

`ot-ctl` must be run with `sudo` to access the `otbr-agent` Unix socket. Add a NOPASSWD rule to avoid repeated password prompts:

```bash
echo "$USER ALL=(ALL) NOPASSWD: /usr/bin/ot-ctl" | sudo tee /etc/sudoers.d/ot-ctl
sudo chmod 440 /etc/sudoers.d/ot-ctl
```

### dbus-python fails to install in venv

```
error: ld returned 1 exit status — undefined reference to `round'
```

Use system Python for D-Bus scripts:

```bash
sudo /usr/bin/python3 bluez_hci_validate.py
```

### BLE HCI no response when connecting directly to /dev/ttyACM0

The device speaks the Combined Serial MUX protocol — raw H4 frames sent directly to the UART are not understood. Always go through `spinel_bridge.py` and use the printed PTY paths.

### Keepalive watchdog: bridge exits after 15 s

The device sends `CMD_KEEPALIVE` every 5 s. If the bridge misses 3 consecutive keepalives it exits. Ensure the device firmware is running and the baud rate matches (921600).
