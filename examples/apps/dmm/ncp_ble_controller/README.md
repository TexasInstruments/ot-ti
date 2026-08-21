# OpenThread NCP-BLE-Controller Example

## Introduction

This example demonstrates running both the Thread (NCP) and BLE Controller stacks
simultaneously on a single TI CC27XX device connected to a Linux host.

Following this guide you will be able to:

- Create a Thread network via OTBR + NCP-BLE-Controller setup
- Join a node to the Thread network
- Send messages between two Thread network devices
- Scan for active BLE peripheral devices via BlueZ
- Advertise the device as a BLE peripheral (visible on phone)
- Connect and disconnect BLE devices

## Architecture

The NCP-BLE-Controller uses the Combined Serial MUX to multiplex Thread Spinel
and BLE HCI over a single UART port. The `spinel_bridge.py` daemon on the host
demultiplexes the single UART into per-stack virtual PTYs.

```
┌─────────────────────────────────────────────────────┐
│  CC27XX Device                                      │
│  ┌──────────┐  ┌──────────┐  ┌───────────────────┐ │
│  │ Thread   │  │  BLE     │  │  Combined Serial  │ │
│  │ NCP/FTD  │  │ Controller│  │  MUX Task         │ │
│  └────┬─────┘  └────┬─────┘  └────────┬──────────┘ │
│       │              │                  │            │
│       └──────────────┴──────────────────┘            │
│                     Single UART (921600)              │
└─────────────────────────┬───────────────────────────┘
                          │
┌─────────────────────────┴───────────────────────────┐
│  Linux Host                                          │
│  ┌────────────────────────────────────────────────┐  │
│  │  spinel_bridge.py (MUX demux)                  │  │
│  │    NLI=0 → Thread PTY → otbr-agent             │  │
│  │    NLI=1 → BLE PTY    → hciattach / BlueZ     │  │
│  └────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────┘
```

**Key difference from RCP:** In NCP mode the full Thread stack runs on the device.
The host (`otbr-agent`) sends high-level commands (form network, join, etc.)
rather than raw radio commands. This makes the device more autonomous.

## FreeRTOS Task Priorities (Tested Configuration)

This implementation has been validated with the following task priority
configuration:

| Task | Priority | File | Role |
|------|----------|------|------|
| BLE Stack | 9 | SDK internal | BLE Link Layer radio scheduling |
| OT APP task | 9 | `freertos_main.c` | OpenThread NCP event loop |
| MUX TX task | 7 | `mux_task_app.h` | Keepalive transmission |
| BleRxDlvr task | 7 | `mux_virt_uart.c` | Delivers BLE HCI RX from ISR to task context |
| NPI task | 7 | `npi_task.c` | BLE HCI TX response drain |
| Timer/Daemon task | 6 | `FreeRTOSConfig.h` | FreeRTOS software timers and deferred ISR events |

## Software Prerequisites

- [OpenThread Border Router (ot-br-posix)](https://github.com/openthread/ot-br-posix)
- [UniFlash](https://www.ti.com/tool/UNIFLASH) or CCS for flashing
- Linux host (Ubuntu 22.04 recommended)
- [BlueZ](https://github.com/bluez/bluez) (`sudo apt install bluez`)
- Python 3.7+ with `pyserial` (`pip3 install pyserial`)

## Hardware Prerequisites

### NCP-BLE-Controller Device

- [LP_EM_CC2755P20](https://www.ti.com/tool/LP-EM-CC2755P20)
- [LP_EM_CC2755P10](https://www.ti.com/tool/LP-EM-CC2755P10)
- LP_EM_CC2755R10_BG
- [LP_EM_CC2745R10_Q1](https://www.ti.com/tool/LP-EM-CC2745R10-Q1)

### Secondary Thread Device (for network testing)

Any CC27XX or CC13XX board running the `cli-ftd` example.

## Building the Firmware

From the root of the `ot-ti` repository:

```bash
./script/build LP_EM_CC2755P10 -DOT_APP_NCP_BLE_CONTROLLER=1
```

Output binary: `build/bin/ot-ncp-ble-controller-ftd.out`

Flash to the device using CCS or UniFlash.

## Example Usage

### Step 1: Identify the Serial Port

Plug in the LaunchPad and identify the application UART:

```bash
ls /dev/ttyACM*
```

Confirm which port is the application UART (interface 00):

```bash
udevadm info /dev/ttyACM0 | grep ID_USB_INTERFACE_NUM
# Interface 00 = Application UART ← use this one
```

### Step 2: Start the MUX Bridge

```bash
cd examples/apps/dmm/ncp_ble_controller/host
python3 spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600
```

Sample output:

```
10:19:40 [INFO] [ble]    PTY: /dev/pts/9   (NLI=1, mode=hci)
10:19:40 [INFO] [thread] PTY: /dev/pts/28  (NLI=0, mode=spinel)
10:19:40 [INFO] Bridge running.  ble=/dev/pts/9  thread=/dev/pts/28
```

Note the **Thread PTY** and **BLE PTY** paths. Keep this terminal running.

### Step 3: Start OTBR Agent

```bash
sudo otbr-agent -I wpan0 -B eth0 \
    "spinel+hdlc+uart:///dev/pts/28" --verbose
```

Replace `/dev/pts/28` with your Thread PTY and `eth0` with your LAN interface.

### Step 4: Form a Thread Network

```bash
sudo ot-ctl dataset init new
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
```

Wait ~30 seconds and verify:

```bash
sudo ot-ctl state
# Expected: leader
```

### Step 5: Join a Second Device

Get the dataset from the leader:

```bash
sudo ot-ctl dataset active -x
# Output: 0e080000000000010000...
```

On the second device CLI:

```
dataset set active 0e080000000000010000...
dataset commit active
ifconfig up
thread start
```

### Step 6: Ping Between Devices

Get the peer's RLOC:

```bash
sudo ot-ctl ipaddr rloc
```

Ping:

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

### Step 8: BLE Scan

```bash
sudo bluetoothctl
```

Select the UART adapter (if multiple adapters exist):

```
[bluetooth]# list
[bluetooth]# select <BD-Address-of-UART-adapter>
```

Start LE scan:

```
[bluetooth]# scan le
```

Expected output:

```
[NEW] Device 6E:24:88:0C:B4:35 JBL Tune 520BT-LE
[NEW] Device 80:C3:BA:4E:3A:96 MOMENTUM 4
```

Stop scan:

```
[bluetooth]# scan off
```

### Step 9: BLE Advertise (with device name)

Make the device visible to phones and other BLE scanners with a custom name:

```
[bluetooth]# power on
[bluetooth]# menu advertise
[bluetooth]# name TI-CC27XX-DMM
[bluetooth]# back
[bluetooth]# advertise on
```

Now open any BLE scanner app on your phone (e.g. nRF Connect, LightBlue) and
filter by name **TI-CC27XX-DMM** — the device should appear.

Stop advertising:

```
[bluetooth]# advertise off
```

### Step 10: Connect to a BLE Peripheral

After scanning, connect to a discovered device:

```
[bluetooth]# connect 6E:24:88:0C:B4:35
```

Verify connection:

```
[bluetooth]# info 6E:24:88:0C:B4:35
```

Disconnect:

```
[bluetooth]# disconnect 6E:24:88:0C:B4:35
```

### Step 11: Verify Concurrent Operation

With Thread and BLE both running:

| Check | Command |
|-------|---------|
| Thread state | `sudo ot-ctl state` → `leader` or `router` |
| Thread ping | `sudo ot-ctl ping <address> 100 10 1 64 5` |
| BLE scan | `bluetoothctl scan le` → devices appear |
| BLE advertise | Sniffer sees "TI-CC27XX-DMM" |
| HCI traffic | `sudo btmon` → shows events |

### Cleanup

```bash
# Stop BLE
sudo killall hciattach

# Stop otbr-agent
# Ctrl+C in the otbr-agent terminal

# Stop bridge
# Ctrl+C in the bridge terminal
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Bridge: no output | Wrong serial port | Check `dmesg`, try other `/dev/ttyACM*` |
| otbr-agent: "no response from RCP" | Stale PTY or wrong firmware | Restart bridge, use new PTY, verify NCP firmware |
| `ot-ctl` hangs | otbr-agent crashed | Check otbr-agent terminal |
| `hciattach` fails | Wrong BLE PTY | Use PTY from bridge output |
| BLE scan: "Authentication Failed" | HCI timeout under Thread load | Expected under very heavy ping load; reduce ping rate |
| Sniffer can't see device | Advertising not started | Run `advertise on` and `discoverable on` |
| `bluetoothctl` shows wrong adapter | Multiple adapters | Use `list` then `select <address>` |
