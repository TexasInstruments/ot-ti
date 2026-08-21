# BeagleBone Black (BBB) Setup — DMM NCP+BLE Controller Host

Setup guide for running the TI cc27xx **DMM NCP+BLE Controller** host-side tools on a
BeagleBone Black, including OTBR agent installation and the Combined Serial MUX bridge.

**Validated on: Debian 10 (Buster) | Python 3.7**

---

## Architecture

| | NCP + BLE |
|---|---|
| Thread stack location | **Device** (CC27xx) |
| Spinel role | Network Co-Processor |
| Bridge (`spinel_bridge.py`) | Demuxes Thread spinel + BLE HCI over one UART |
| BLE path | `hciattach` / BlueZ via bridge PTY |
| `otbr-agent` command | Same as RCP — uses Thread PTY from bridge |
| `CMD_RESET` behaviour | Pseudo-reset: Thread restarts on device, BLE unaffected |

---

## Hardware / Network Interfaces

| Interface | Address         | Notes                                         |
|-----------|-----------------|-----------------------------------------------|
| `eth0`    | 10.24.53.x/21   | Primary LAN — use for all installation and OTBR |
| `usb0`    | 192.168.7.2     | USB gadget — use **only** to find `eth0` IP, then disable |
| `usb1`    | 192.168.6.2     | USB gadget — disable before installing OTBR   |

> **Internet access required** for OTBR installation.
> Connect the BBB Ethernet port (`eth0`) to the internet before starting.
> The DMM bridge and BLE tests do **not** require internet on the BBB.

---

## Pre-Installation: Find eth0 IP via USB

On a fresh BBB, use the USB gadget interface to get initial shell access and
discover the `eth0` IP, then switch to Ethernet for all subsequent steps.

**From development PC — SSH over USB:**

```bash
ssh debian@192.168.7.2
# Default password: temppwd
```

**On BBB — find the Ethernet IP:**

```bash
ip addr show eth0
# Note the inet address, e.g. 10.24.53.150
```

**Reconnect via Ethernet** for all remaining steps:

```bash
# From development PC
ssh debian@<eth0-ip>
```

**Bring down USB interfaces** to prevent port conflicts during installation:

```bash
sudo ip link set usb0 down
sudo ip link set usb1 down
```

---

## 0. Setup TI Network Proxy (if BBB is on TI Network)

If the BBB is connected to the TI corporate network, configure proxy settings
before running any package install or git commands.

Refer to the internal TI guide:
https://confluence.itg.ti.com/display/J7TDA4xSW/How+to+setup+TI+network+proxy

---

## 1. Build and Flash NCP+BLE Firmware

On the development PC:

```bash
cd ot-ti
./script/bootstrap          # first time only
./script/build LP_EM_CC2755P10 -DOT_APP_NCP_BLE_CONTROLLER=1 -DOT_FTD=1
```

Flash `build/bin/ot-ncp-ble-controller.out` to the device using UniFlash or CCS.

---

## 2. Install OTBR Agent

Reference: https://openthread.io/guides/border-router/build-native

### 2a. Clone ot-br-posix

```bash
git clone https://github.com/openthread/ot-br-posix.git
cd ot-br-posix
```

### 2b. Bootstrap (install build dependencies)

```bash
./script/bootstrap
```

### 2c. Build and install

Use `INFRA_IF_NAME=eth0` to bind OTBR to the Ethernet interface:

```bash
INFRA_IF_NAME=eth0 ./script/setup
```

The `setup` script builds and installs `otbr-agent`, registers it as a
`systemd` service, and installs `bind9` as the DNS server. The service is
enabled and will start on reboot.

To start the service immediately without rebooting:

```bash
sudo systemctl start otbr-agent
sudo systemctl status otbr-agent
```

---

## 3. Fix bind9 Conflict with dnsmasq

The `setup` script installs `bind9`. On BBB, the pre-installed `dnsmasq`
holds port 53 on all interfaces, causing bind9 to fail.

### Symptom

```
bind9 named[xxx]: binding TCP socket: address in use
bind9 named[xxx]: unable to listen on any configured interfaces
```

### Identify the conflict

```bash
sudo ss -tlnp 'sport = :53'
# Expected: dnsmasq (pid XXXX) holding 0.0.0.0:53
```

### Fix — stop and disable dnsmasq

```bash
sudo systemctl stop dnsmasq
sudo systemctl disable dnsmasq

# Complete the interrupted bind9 installation
sudo dpkg --configure bind9
```

> If dnsmasq is needed for DHCP on USB interfaces, keep it but disable its
> DNS listener: set `port=0` in `/etc/dnsmasq.conf`, then
> `sudo systemctl restart dnsmasq` before configuring bind9.

### Restrict bind9 to eth0 only

Edit `/etc/bind/named.conf.options`, inside the `options { }` block:

```
options {
    listen-on { <eth0-ip>; 127.0.0.1; };
    listen-on-v6 { none; };
    // ... rest of options
};
```

```bash
sudo systemctl restart bind9
sudo systemctl status bind9   # should show active (running)
```

---

## 4. Transfer Host Tools to BBB

The DMM host tools (MUX bridge, BLE test scripts, offline packages) are in:

```
examples/apps/dmm/ncp_ble_controller/host/
```

From the development PC, inside the `ncp_ble_controller/` directory:

```bash
scp -r host/ debian@<BBB-IP>:~
```

Verify on BBB:

```bash
ls ~/host/objects/
# Expected: pyserial-3.5-py2.py3-none-any.whl  python3-serial_3.5-1_all.deb
```

Host directory layout:

```
host/
├── spinel_bridge/
│   └── spinel_bridge.py          # Combined Serial MUX bridge daemon
├── setup.sh                      # Ubuntu PC venv setup (not for BBB)
├── requirements.txt              # Ubuntu PC pip requirements (not for BBB)
└── objects/                      # Offline packages for BBB
    ├── pyserial-3.5-py2.py3-none-any.whl
    └── python3-serial_3.5-1_all.deb
```

---

## 5. Install pyserial (offline — no internet needed)

**Method A — Python wheel (recommended):**

```bash
cd ~/host
pip3 install --no-index objects/pyserial-3.5-py2.py3-none-any.whl
```

**Method B — Debian package:**

```bash
cd ~/host
sudo dpkg -i objects/python3-serial_3.5-1_all.deb
```

Verify:

```bash
python3 -c "import serial; print(serial.__version__)"
# Expected: 3.5
```

---

## 6. Serial Port Permissions

```bash
sudo usermod -aG dialout $USER
# Log out and back in, then verify:
groups | grep dialout
```

---

## 7. Identify the Serial Port

| Connection type           | Typical port on BBB              |
|---------------------------|----------------------------------|
| USB / XDS110 (CDC-ACM)    | `/dev/ttyACM0` (application UART) |
| USB / XDS110 (CDC-ACM)    | `/dev/ttyACM1` (XDS110 debug UART) |
| USB-to-UART adapter       | `/dev/ttyUSB0`                   |
| BBB UART1 header pins     | `/dev/ttyO1`                     |
| BBB UART2 header pins     | `/dev/ttyO2`                     |

```bash
# Check which device appeared after plugging in
dmesg | tail -20
# Look for: cdc_acm 1-1:1.0: ttyACM0: USB ACM device

# Confirm which interface is the application UART
udevadm info /dev/ttyACM0 | grep ID_USB_INTERFACE_NUM
# Interface 00 = Application UART (use this)
# Interface 03 = XDS110 Auxiliary UART
```

---

## 8. Start the MUX Bridge

The `spinel_bridge.py` daemon demultiplexes the single UART into per-stack PTYs:

| NLI | Protocol       | Consumer on BBB                    |
|-----|----------------|------------------------------------|
| 0   | Thread spinel  | `otbr-agent`                       |
| 1   | BLE HCI (H4)   | `hciattach` / BlueZ                |
| 2   | Zigbee spinel  | not used (PTY created, no consumer) |
| 3   | Keepalive      | bridge internal                    |

**BBB — Terminal 1 (keep running):**

```bash
cd ~/host
python3 spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
```

Sample output:

```
18:00:38 [INFO] [ble]    PTY: /dev/pts/1   (NLI=1, mode=hci)
18:00:38 [INFO] [thread] PTY: /dev/pts/3   (NLI=0, mode=spinel)
18:00:38 [INFO] Bridge running.  ble=/dev/pts/1  zb=/dev/pts/2  thread=/dev/pts/3
```

> Note the **Thread PTY** (e.g. `/dev/pts/3`) and **BLE PTY** (e.g. `/dev/pts/1`).
> The PTY number changes every time the bridge restarts — always read it from the output.

Optional flags:

```bash
# Verbose debug logging
python3 spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600 --verbose
```

To survive SSH disconnects, run inside `screen` or `tmux`:

```bash
screen -S bridge
python3 spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
# Detach: Ctrl-A then D  |  Reattach: screen -r bridge
```

---

## 9. Verify BLE HCI Path

Before starting `otbr-agent`, verify the BLE path through the bridge using
the BLE PTY (e.g. `/dev/pts/1`) printed by the bridge.

**BBB — Terminal 2:**

```bash
cd ~/host
python3 ble_hci_test.py /dev/pts/1   # use BLE PTY path from bridge output
```

Expected output:

```
BLE HCI Connection Validator
  Port    : /dev/pts/1
────────────────────────────────────────────────────
  [✓] HCI Reset
  [✓] Read Local Version — ... manufacturer=0x000D (Texas Instruments) ...
  [✓] Read BD_ADDR — XX:XX:XX:XX:XX:XX
────────────────────────────────────────────────────
Result: 3/3 passed  — ALL PASS
```

---

## 10. Start otbr-agent (Manual — using MUX bridge PTY)

The `setup` script registers a systemd service that starts `otbr-agent` at boot
and owns the radio interface. Stop it before running manually:

```bash
sudo systemctl stop otbr-agent
sudo systemctl disable otbr-agent
```

Start `otbr-agent` using the **Thread PTY** from the bridge (e.g. `/dev/pts/3`):

```bash
sudo otbr-agent -I wpan0 -B eth0 \
    "spinel+hdlc+uart:///dev/pts/3" \
    trel://eth0 --verbose
```

> Replace `/dev/pts/3` with the actual Thread PTY path printed by the bridge.

`otbr-agent` auto-detects NCP vs RCP from `SPINEL_PROP_CAPS` at connect time.

To re-enable the service after testing:

```bash
sudo systemctl enable otbr-agent
sudo systemctl start otbr-agent
```

---

## 11. CMD_RESET Behaviour (NCP-Specific)

When `otbr-agent` first connects it sends `SPINEL_CMD_RESET`. In NCP mode, this
triggers `otPlatReset()` on the device. Without the DMM fix this would cause a
full hardware reset, killing BLE advertising.

**The NCP+BLE firmware** (`src/misc.c`, guarded by `USE_DMM`) implements a
pseudo-reset instead:

1. Sets a flag read by `otSysPseudoResetWasRequested()`
2. Wakes the OT main loop via `platformAlarmSignal()`
3. Main loop exits → `otInstanceFinalize()` → clears NCP HDLC state
4. `goto pseudo_reset` → `otSysInit()` → `otInstanceInitSingle()` → `otAppNcpInit()` — Thread and NCP restart cleanly
5. BLE FreeRTOS task is never touched

From `otbr-agent`'s perspective: it sends RESET → device does a controlled
Thread-stack restart → NCP is ready → `otbr-agent` continues normal init.

---

## 12. Verify Thread Network

```bash
sudo ot-ctl state           # disabled  (Thread not yet started)
sudo ot-ctl dataset init new
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
# Wait ~30 s
sudo ot-ctl state           # leader
sudo ot-ctl ipaddr
```

In NCP mode the Thread stack runs **on the device**. `ot-ctl` commands are
forwarded as spinel NCP property sets/gets; the device executes them and
reports results back.

---

## Troubleshooting

### bind9 fails to start

```bash
sudo ss -tlnp 'sport = :53'
# Stop whichever process is listed, then:
sudo dpkg --configure bind9
```

### otbr-agent exits after printing the Radio URL (no error)

The Thread PTY path is stale. The bridge was restarted and allocated a new
`/dev/pts/N`. Check the bridge terminal for the new path, then restart
`otbr-agent` with the updated path.

```bash
ls -la /dev/pts/    # confirm the PTY exists
```

### Port 53 conflict after reboot

```bash
systemctl is-enabled dnsmasq    # should be disabled
```

### ModuleNotFoundError: No module named 'serial'

```bash
pip3 install --no-index ~/host/objects/pyserial-3.5-py2.py3-none-any.whl
```

### Permission denied: /dev/ttyACM0

```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### BLE HCI no response when connecting directly to /dev/ttyACM0

The device speaks the Combined Serial MUX protocol — raw H4 frames sent
directly to the UART will not work. Always go through `spinel_bridge.py`
and use the printed PTY path.

### Keepalive watchdog: bridge exits after 15 s

The bridge missed 3 consecutive `CMD_KEEPALIVE` messages (sent every 5 s).
Verify the device firmware is running and the baud rate is 921600.

### SSH session dropped — bridge stopped

Run the bridge in `screen` or `tmux` (see Section 8).

### BLE advertising stops when otbr-agent connects

Cause: firmware built without `USE_DMM` pseudo-reset fix. Rebuild with:

```bash
./script/build LP_EM_CC2755P10 -DOT_APP_NCP_BLE_CONTROLLER=1 -DOT_FTD=1
```

### `ot-ctl` commands time out

Cause: Thread PTY path stale. Restart the bridge and restart `otbr-agent`
with the new PTY from the bridge output.
