# OpenThread RCP-BLE-Controller Example

## Introduction

This example demonstrates running both the Thread (RCP) and BLE Controller stacks
simultaneously on a single TI CC27XX device connected to a Linux host.

Following this guide you will be able to:

- Create a Thread network via OTBR + RCP-BLE-Controller setup
- Join a node to the Thread network
- Send messages between two Thread network devices
- Scan for active BLE peripheral devices via BlueZ
- Advertise the device as a BLE peripheral (visible on phone)
- Connect and disconnect BLE devices

## Architecture

The RCP-BLE-Controller uses the Combined Serial MUX to multiplex Thread Spinel
and BLE HCI over a single UART port. The `spinel_bridge.py` daemon on the host
demultiplexes the single UART into per-stack virtual PTYs, one for `otbr-agent`
and one for BlueZ (`hciattach`). No FTDI cable is required.

```
┌────────────────────────────────────────────────────────────────────────┐
│ Linux Host                                                             │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ Application (e.g. Matter)                                        │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                                                        │
│  ┌────────────┐ Thread PTY ┌──────────────────┐ BLE PTY ┌───────┐      │
│  │    OTBR    │◄──────────►│ spinel_bridge.py │◄───────►│ BlueZ │      │
│  │ otbr-agent │  (NLI=0)   │ Serial MUX demux │ (NLI=1) │       │      │
│  └────────────┘            └────────┬─────────┘         └───────┘      │
│                                     │                                  │
└─────────────────────────────────────┼──────────────────────────────────┘
                                      │  Single UART, 921600 baud
                                      │  HDLC + Spinel NLI
┌─────────────────────────────────────┼──────────────────────────────────┐
│ LP_EM_CC2755P20 - DMM firmware: RCP + BLE Controller                   │
│                                     ▼                                  │
│  ┌──────────────────────────┐   ┌──────────────────────────┐           │
│  │ 802.15.4 radio driver    │   │ BLE Controller           │           │
│  │ (Spinel, NLI=0)          │   │ (HCI, NLI=1)             │           │
│  └────────────┬─────────────┘   └─────────────┬────────────┘           │
│               └───────────────┬───────────────┘                        │
│              ┌────────────────┴───────────────┐                        │
│              │  DMM policy scheduler (radio)  │                        │
│              └────────────────┬───────────────┘                        │
│ DMM / CC27xx                                                           │
└────────────────────────────────────────────────────────────────────────┘
                         BLE  ◄──────┴──────►  Thread
                         (BLE Device)          (Thread Device)
```

### Wire Protocol

Each protocol is wrapped in an HDLC frame with a Spinel NLI header:

```
[0x7E] [Spinel header (NLI)] [payload] [CRC16] [0x7E]

NLI 0 = Thread Spinel frames  → otbr-agent
NLI 1 = BLE HCI packets       → BlueZ / hciattach
NLI 3 = Keepalive heartbeat   → bridge internal
```

**Key difference from NCP:** In RCP mode the Thread stack runs on the host
(`otbr-agent`) and the device is only a radio co-processor. The device runs the
IEEE 802.15.4 radio driver and the BLE controller, and the DMM policy scheduler
arbitrates the shared radio between them.

## Software Prerequisites

- [OpenThread Border Router (ot-br-posix)](https://github.com/openthread/ot-br-posix)
- [UniFlash](https://www.ti.com/tool/UNIFLASH) or CCS for flashing
- Linux host (Ubuntu 22.04 recommended)
- [BlueZ](https://github.com/bluez/bluez) (`sudo apt install bluez`)
- Python 3.7+ with `pyserial` (`pip3 install pyserial`)

## Hardware Prerequisites

### RCP-BLE-Controller Device

- [LP_EM_CC2755P20](https://www.ti.com/tool/LP-EM-CC2755P20)
- [LP_EM_CC2755P10](https://www.ti.com/tool/LP-EM-CC2755P10)
- LP_EM_CC2755R10_BG
- [LP_EM_CC2745R10_Q1](https://www.ti.com/tool/LP-EM-CC2745R10-Q1)

### Linux Host (Border Router / BLE Host)

This example has been verified on:

- Generic Linux Ubuntu 22.04 host
- [Raspberry Pi 4](https://www.raspberrypi.com/)
- [BeagleBone Black](https://www.beagleboard.org/boards/beaglebone-black)
  (Debian 10 Buster, offline — see [../host/README_BBB_HOST.md](../host/README_BBB_HOST.md))

The host must have a Linux image with native BlueZ and HCI support.

### Secondary Thread Device (for network testing)

Any CC27XX or CC13XX board running the `cli-ftd` or `cli-mtd` example.

## Building the Firmware

From the root of the `ot-ti` repository. On the first build, run the bootstrap
script:

```bash
./script/bootstrap
./script/build LP_EM_CC2755P20 -DOT_APP_RCP_BLE_CONTROLLER=ON
```

Other supported boards: `LP_EM_CC2755P10`, `LP_EM_CC2755R10_BG`,
`LP_EM_CC2745R10_Q1`. The UART baud rate for both BLE and Thread is **921600**.

Output binary: `build/bin/ot-rcp-ble-controller.out`

Flash to the device using CCS or UniFlash.

> **Note:** Flags must be passed with a `-D` prefix and a value. The build
> script reuses `build/` and CMake caches flags between runs. When switching
> between RCP+BLE, NCP+BLE and the plain apps, run `rm -rf build` first or set
> `OT_CMAKE_BUILD_DIR=<dir>` to keep one build directory per mode.

## Example Usage

### Pre-work: Disable Existing Bluetooth Adapters

If the host has an internal adapter, bring it down so the TI device is the only
BLE adapter (or select it explicitly in `bluetoothctl`):

```bash
sudo hciconfig
sudo hciconfig hci0 down
```

### Step 1: Identify the Serial Port

Plug in the LaunchPad and identify the application UART:

```bash
ls /dev/ttyACM*
```

Confirm which port is the application UART (interface 00):

```bash
udevadm info /dev/ttyACM0 | grep ID_USB_INTERFACE_NUM
# Interface 00 = Application UART ← use this one
# Interface 03 = XDS110 auxiliary UART
```

### Step 2: Start the MUX Bridge

```bash
cd examples/apps/dmm/host
bash setup.sh                      # first time only: creates .venv
source .venv/bin/activate
python3 spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
```

Sample output:

```
10:19:40 [INFO] [ble]    PTY: /dev/pts/9   (NLI=1, mode=hci)
10:19:40 [INFO] [thread] PTY: /dev/pts/28  (NLI=0, mode=spinel)
10:19:40 [INFO] Bridge running.  ble=/dev/pts/9  thread=/dev/pts/28
```

Note the **Thread PTY** and **BLE PTY** paths. Keep this terminal running.

Useful `spinel_bridge.py` options:

| Option | Description |
|--------|-------------|
| `--port PORT` | Serial port (e.g. `/dev/ttyACM0`) |
| `--baud BAUD` | Baud rate (default: 921600) |
| `--no-ble` | Disable BLE PTY (NLI=1) |
| `--no-thread` | Disable Thread PTY (NLI=0) |
| `--verbose` | Debug logging |

Detailed host guides (shared with NCP+BLE):

| Platform | Guide |
|----------|-------|
| BeagleBone Black, Debian 10 (offline) | [../host/README_BBB_HOST.md](../host/README_BBB_HOST.md) |
| Bridge internals | [../host/spinel_bridge/DESIGN.md](../host/spinel_bridge/DESIGN.md) |

### Step 3: Start OTBR Agent

Follow the [OpenThread Border Router build guide](https://openthread.io/guides/border-router/build)
(Task 1, agent setup only), then:

```bash
sudo systemctl stop otbr-agent   # stop the systemd service if running
sudo otbr-agent -I wpan0 -B eth0 \
    "spinel+hdlc+uart:///dev/pts/28" \
    trel://eth0 --verbose
```

Replace `/dev/pts/28` with your Thread PTY and `eth0` with your LAN interface.

### Step 4: Form a Thread Network

```bash
sudo ot-ctl dataset init new
sudo ot-ctl dataset panid 0xface
sudo ot-ctl dataset channel 11
sudo ot-ctl dataset networkkey 00112233445566778899aabbccddeeff
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
```

Wait a few seconds and verify:

```bash
sudo ot-ctl state
# Expected: leader
```

### Step 5: Join a Second Device

On a secondary device running `cli-ftd`, use the same network parameters:

```
networkkey 00112233445566778899aabbccddeeff
panid 0xface
channel 11
ifconfig up
thread start
```

After a few seconds, `state` should report `child` (or `router`).

### Step 6: Ping Between Devices

Get the peer's RLOC on the second device:

```
ipaddr rloc
```

Ping from the host:

```bash
sudo ot-ctl ping <peer-rloc-address> 100 10 1 64 5
```

### Step 7: Attach BLE (hciattach)

```bash
sudo hciattach /dev/pts/9 any 921600
```

Replace `/dev/pts/9` with your BLE PTY. Verify:

```bash
hciconfig
```

You should see a new adapter with `Bus: UART` and `UP RUNNING`.

### Step 8: BLE Scan (HCI commands only)

> **Scan with raw HCI commands, not `bluetoothctl scan on` / `scan le`.**
> BlueZ picks its own scan parameters, and for discovery it typically uses a
> window equal to the interval (100% duty cycle). The scan then takes the radio
> away from Thread, and `otbr-agent` can fail with **radio TX timeouts** and
> reset or crash. Always set the scan parameters yourself and keep
> **scan window ≤ 75% of scan interval**.

Find the TI adapter index (the examples use `hci1`):

```bash
hciconfig
sudo hciconfig hci1 up
```

Set a 75% duty cycle (interval 80 ms, window 60 ms) and start scanning with
duplicates reported:

```bash
sudo hcitool -i hci1 cmd 0x08 0x000C 00 00                  # scan off
sudo hcitool -i hci1 cmd 0x08 0x000B 01 80 00 60 00 00 00   # 80 ms / 60 ms = 75%
sudo hcitool -i hci1 cmd 0x08 0x000C 01 00                  # scan on
```

View the advertising reports in a second terminal:

```bash
sudo btmon
```

Stop the scan:

```bash
sudo hcitool -i hci1 cmd 0x08 0x000C 00 00
```

The full command reference (other duty cycles, duplicate filtering, connect,
disconnect) is in
[BLE Scan Duty Cycle and Raw HCI Commands](#ble-scan-duty-cycle-and-raw-hci-commands).

**If you use `bluetoothctl` anyway:** it only works safely if the scan window
stays at or below 75% of the scan interval. Run `sudo btmon` first and check the
`LE Set Scan Parameters` command BlueZ sends (`Interval` and `Window`). If the
window is more than 75% of the interval, stop the scan and use the HCI commands
above instead.

### Step 9: BLE Advertise (with device name)

Make the device visible to phones and other BLE scanners with a custom name:

```
[bluetooth]# power on
[bluetooth]# menu advertise
[bluetooth]# name TI-CC27XX-DMM
[bluetooth]# back
[bluetooth]# advertise on
```

Open a BLE scanner app on your phone (e.g. nRF Connect, LightBlue) and filter by
name **TI-CC27XX-DMM**. Stop advertising:

```
[bluetooth]# advertise off
```

### Step 10: Connect to a BLE Peripheral

Connect with the HCI `LE Create Connection` command, using the peer address from
the scan (reversed byte order; address type `01` random or `00` public). Example
for `7C:14:8B:25:54:5C` (random):

```bash
sudo hcitool -i hci1 cmd 0x08 0x000D \
    60 00  30 00  00  01  5C 54 25 8B 14 7C  00 \
    18 00  28 00  00 00  C8 00  00 00  00 00
```

Disconnect using the connection handle from the connection-complete event
(for example `0x0040`):

```bash
sudo hcitool -i hci1 cmd 0x01 0x0006 40 00 13
```

See the field-by-field breakdown in the HCI reference below.

### Step 11: Verify Concurrent Operation

With Thread and BLE both running:

| Check | Command |
|-------|---------|
| Thread state | `sudo ot-ctl state` → `leader` or `router` |
| Thread ping | `sudo ot-ctl ping <address> 100 10 1 64 5` |
| BLE scan | HCI scan commands (Step 8) → advertising reports in `btmon` |
| BLE advertise | Sniffer sees "TI-CC27XX-DMM" |
| HCI traffic | `sudo btmon` → shows events |

### Cleanup

```bash
sudo killall hciattach        # stop BLE
# Ctrl+C in the otbr-agent terminal
# Ctrl+C in the bridge terminal
```

## BLE Scan Duty Cycle and Raw HCI Commands

### Keep the scan duty cycle at or below 75%

The BLE controller and the Thread radio share one radio through the DMM policy
scheduler. A continuous scan (window = interval, 100% duty cycle) leaves no
radio time for Thread, which causes Thread packet loss, missed polls and parent
detach.

**Always keep `scan window / scan interval` at 75% or lower.**

HCI scan parameters are in units of 0.625 ms:

| Interval | Window | Duty cycle | Use |
|----------|--------|-----------|-----|
| `0x0080` (80 ms) | `0x0020` (20 ms) | 25% | Light BLE load, best Thread throughput |
| `0x0080` (80 ms) | `0x0040` (40 ms) | 50% | Balanced |
| `0x0080` (80 ms) | `0x0060` (60 ms) | **75%** | **Maximum allowed** |
| `0x0080` (80 ms) | `0x0080` (80 ms) | 100% | **Do not use** |

> `bluetoothctl scan on` / `scan le` lets BlueZ choose the parameters, which can
> be a 100% duty cycle and cause `otbr-agent` radio TX timeouts. Set the
> parameters yourself with `hcitool` as shown below.

### Finding your HCI index

`hciattach` creates a new adapter. Use `hciconfig` to find its index (the
examples below use `hci1`; replace it with yours):

```bash
hciconfig
sudo hciconfig hci1 up
```

All commands below use `sudo hcitool -i hci1 cmd 0x08 <OCF> <bytes>`.
OGF `0x08` is LE Controller. Multi-byte fields are little-endian.

### Set scan parameters (75% duty cycle) and start scanning

Scanning must be disabled before changing parameters.

```bash
# 1. Make sure scan is off
sudo hcitool -i hci1 cmd 0x08 0x000C 00 00

# 2. LE Set Scan Parameters (0x000B)
#    type=01 (active)  interval=0x0080  window=0x0060  own addr=00 (public)  filter=00
sudo hcitool -i hci1 cmd 0x08 0x000B 01 80 00 60 00 00 00

# 3. LE Set Scan Enable (0x000C): enable=01  filter-duplicates=00 (report duplicates)
sudo hcitool -i hci1 cmd 0x08 0x000C 01 00
```

| Byte(s) | Field | Value above |
|---------|-------|-------------|
| `01` | Scan type (`00` passive, `01` active) | active |
| `80 00` | Scan interval (LE16, ×0.625 ms) | 0x0080 = 80 ms |
| `60 00` | Scan window (LE16, ×0.625 ms) | 0x0060 = 60 ms (75%) |
| `00` | Own address type (`00` public, `01` random) | public |
| `00` | Filter policy (`00` accept all) | accept all |

A success response is `HCI Command: ... plen ...` followed by an event ending in
`00` (status = success).

To see advertising reports while scanning:

```bash
sudo btmon                       # in another terminal
# or
sudo hcitool -i hci1 lescan --duplicates   # simple alternative (uses BlueZ default parameters)
```

### Stop scanning

```bash
sudo hcitool -i hci1 cmd 0x08 0x000C 00 00
```

### Change the duty cycle and scan again

Stop the scan, change the window, then start again. Example: 25% then 50%.

```bash
sudo hcitool -i hci1 cmd 0x08 0x000C 00 00                     # stop
sudo hcitool -i hci1 cmd 0x08 0x000B 01 80 00 20 00 00 00      # 25%: window 0x0020
sudo hcitool -i hci1 cmd 0x08 0x000C 01 00                     # start

sudo hcitool -i hci1 cmd 0x08 0x000C 00 00                     # stop
sudo hcitool -i hci1 cmd 0x08 0x000B 01 80 00 40 00 00 00      # 50%: window 0x0040
sudo hcitool -i hci1 cmd 0x08 0x000C 01 00                     # start
```

### Duplicate filtering

The second byte of `LE Set Scan Enable` controls duplicates:

| Command | Behaviour |
|---------|-----------|
| `... 0x000C 01 00` | Report every advertisement (duplicates **on**) |
| `... 0x000C 01 01` | Report each device once (duplicates filtered) |

### Connect to a MAC address

`LE Create Connection` (`0x000D`) takes 25 bytes. The peer address is sent
**least-significant byte first**, so `7C:14:8B:25:54:5C` becomes
`5C 54 25 8B 14 7C`.

```bash
# Peer 7C:14:8B:25:54:5C, Random address type
sudo hcitool -i hci1 cmd 0x08 0x000D \
    60 00  30 00  00  01  5C 54 25 8B 14 7C  00 \
    18 00  28 00  00 00  C8 00  00 00  00 00
```

| Bytes | Field | Value |
|-------|-------|-------|
| `60 00` | Scan interval | 0x0060 = 60 ms |
| `30 00` | Scan window | 0x0030 = 30 ms (50%, within the 75% limit) |
| `00` | Initiator filter policy | use peer address below |
| `01` | **Peer address type** | `00` public, `01` random |
| `5C 54 25 8B 14 7C` | Peer address (reversed) | 7C:14:8B:25:54:5C |
| `00` | Own address type | public |
| `18 00` / `28 00` | Connection interval min / max | 30 ms / 50 ms |
| `00 00` | Peripheral latency | 0 |
| `C8 00` | Supervision timeout | 2000 ms |
| `00 00` / `00 00` | Min / max CE length | 0 |

> **The peer address type must match what the device advertises.** Phones and
> many peripherals use a random address (`01`). If the connection never
> completes, you used the wrong type. Check the advertising report in `btmon`:
> `Address type: Random (0x01)` or `Public (0x00)`.

Success is an `LE Enhanced Connection Complete` / `LE Connection Complete` event
with `Status: Success (0x00)` and a connection **handle** (for example `0x0040`).
Note the handle for disconnecting.

Creating a connection stops the controller's scan. After connecting, restart
scanning with the 75% (or lower) parameters above if you still need to scan.

### Cancel a pending connection

If the peer is not found and the connect is still pending:

```bash
sudo hcitool -i hci1 cmd 0x08 0x000E
```

### Disconnect

`Disconnect` is OGF `0x01`, OCF `0x0006`. Parameters are the connection handle
(LE16) and the reason (`13` = remote user terminated, `16` = local host
terminated).

```bash
# Handle 0x0040, reason 0x13
sudo hcitool -i hci1 cmd 0x01 0x0006 40 00 13
```

Or use BlueZ: `sudo hcitool -i hci1 dc 7C:14:8B:25:54:5C`.

### Quick reference

| Action | Command |
|--------|---------|
| Scan off | `sudo hcitool -i hci1 cmd 0x08 0x000C 00 00` |
| Set 75% (max) | `sudo hcitool -i hci1 cmd 0x08 0x000B 01 80 00 60 00 00 00` |
| Scan on, duplicates reported | `sudo hcitool -i hci1 cmd 0x08 0x000C 01 00` |
| Scan on, duplicates filtered | `sudo hcitool -i hci1 cmd 0x08 0x000C 01 01` |
| Cancel connect | `sudo hcitool -i hci1 cmd 0x08 0x000E` |
| Disconnect handle 0x0040 | `sudo hcitool -i hci1 cmd 0x01 0x0006 40 00 13` |

### Pairing note

Pairing from `bluetoothctl` needs a registered agent. If you see
`No agent available` or `Numeric comparison failed`, run:

```
[bluetooth]# agent on
[bluetooth]# default-agent
```

Use a BLE sniffer or `sudo btmon` to inspect HCI traffic while testing.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Bridge: no output | Wrong serial port | Check `dmesg`, try other `/dev/ttyACM*` |
| otbr-agent: "no response from RCP" | Stale PTY or wrong firmware | Restart bridge, use new PTY, verify RCP firmware |
| `ot-ctl` hangs | otbr-agent crashed | Check otbr-agent terminal |
| `hciattach` fails | Wrong BLE PTY | Use PTY from bridge output |
| BLE scan: "Authentication Failed" | HCI timeout under Thread load | Expected under very heavy ping load; reduce ping rate |
| Sniffer can't see device | Advertising not started | Run `advertise on` and `discoverable on` |
| `bluetoothctl` shows wrong adapter | Multiple adapters | Use `list` then `select <address>` |
| Thread drops / packet loss while scanning | Scan duty cycle too high | Keep scan window / interval at 75% or lower (see above) |
| Connect command never completes | Wrong peer address type | Use `01` for random, `00` for public; check `btmon` |
