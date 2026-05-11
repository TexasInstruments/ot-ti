# Zigbee NCP + BLE Controller

This example runs a Zigbee network co-processor (NCP) and a Bluetooth LE controller at the same
time on a single TI CC27XX device. A Linux host runs the Zigbee and Bluetooth host stacks, and the
device shares its radio between the two protocols.

| Stack | Runs on the device | Runs on the Linux host |
|-------|--------------------|------------------------|
| Zigbee | ZBOSS NCP | TI Zigbee Linux Host SDK |
| Bluetooth LE | Controller (HCI) | BlueZ |

Both stacks communicate with the host over one USB serial connection. The `spinel_bridge.py`
script on the host splits that connection into one virtual serial port per stack.

```
┌─────────────────────────────────────────────────────────────────────┐
│  CC27XX Device                                                      │
│           ┌────────────┐  ┌────────┐  ┌─────────────────┐           │
│           │ BLE        │  │ Zigbee │  │ Combined Serial │           │
│           │ Controller │  │ NCP    │  │ MUX Task        │           │
│           └──────┬─────┘  └────┬───┘  └────────┬────────┘           │
│                  │             │               │                    │
│                  └─────────────┴───────────────┘                    │
│                      Single UART (921600 baud)                      │
└──────────────────────────────────┬──────────────────────────────────┘
┌──────────────────────────────────┴──────────────────────────────────┐
│  Linux Host                                                         │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │  spinel_bridge.py (MUX demux)                                 │  │
│  │    NLI=1 → BLE PTY    → hciattach / BlueZ                     │  │
│  │    NLI=2 → Zigbee PTY → Zigbee host application               │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Requirements

**Hardware**

- One of these LaunchPads, with a USB cable:
  - [LP-EM-CC2755P20](https://www.ti.com/tool/LP-EM-CC2755P20)
  - [LP-EM-CC2755P10](https://www.ti.com/tool/LP-EM-CC2755P10)
  - LP-EM-CC2755R10-BG
  - [LP-EM-CC2745R10-Q1](https://www.ti.com/tool/LP-EM-CC2745R10-Q1)
- A second Zigbee device to form a Zigbee network


**Host software**

- Linux (Ubuntu 22.04 recommended)
- Python 3.7 or later
- [BlueZ](https://github.com/bluez/bluez): `sudo apt install bluez`
- TI Zigbee Linux Host SDK
- [UniFlash](https://www.ti.com/tool/UNIFLASH) or Code Composer Studio, to flash the device

Install the build toolchain by following the instructions in the top-level `README.md`.

## Build the Firmware

From the root of the repository:

```bash
./script/build LP_EM_CC2755P20 -DOT_APP_ZIGBEE_NCP_BLE_CONTROLLER=1
```

The firmware image is `build/bin/zigbee-ncp-ble-controller.out`.

## Flash the Firmware

Flash `zigbee-ncp-ble-controller.out` to the LaunchPad using UniFlash or Code Composer Studio.

## Run the Example

### 1. Find the serial port

Connect the LaunchPad and list the serial ports:

```bash
ls /dev/ttyACM*
```

The LaunchPad creates two ports. Use the application UART, which is interface `00`:

```bash
udevadm info /dev/ttyACM0 | grep ID_USB_INTERFACE_NUM
```

### 2. Start the bridge

```bash
python3 spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --zb
```

The bridge prints the virtual serial port of each stack:

```
BLE PTY: /dev/pts/3
ZB PTY:  /dev/pts/4
```

Keep this terminal open while you use the example.

### 3. Start the Zigbee host application

In a new terminal, point the Zigbee host application at the Zigbee port and start it. This example
uses the `onoff_switch_ed` application from the TI Zigbee Linux Host SDK:

```bash
export NCP_SLAVE_PTY=<ZB PTY>
./onoff_switch_ed
```

To form a network, start a coordinator on a second Zigbee device, for example `onoff_light_zc` with
`NCP_SLAVE_PTY` set to that device's serial port.

### 4. Attach the Bluetooth controller

```bash
sudo hciattach <BLE PTY> any 921600
hciconfig
```

`hciconfig` lists a new adapter with `Bus: UART` in the `UP RUNNING` state.

### 5. Use Bluetooth LE

```bash
sudo bluetoothctl
```

If more than one adapter is listed, select the UART adapter:

```
[bluetooth]# list
[bluetooth]# select <adapter address>
```

Scan for devices:

```
[bluetooth]# scan le
[bluetooth]# scan off
```

Advertise as a peripheral. A phone app such as nRF Connect shows the device as `TI-CC27XX-DMM`:

```
[bluetooth]# power on
[bluetooth]# menu advertise
[bluetooth]# name TI-CC27XX-DMM
[bluetooth]# back
[bluetooth]# advertise on
```

Connect to a device found by the scan:

```
[bluetooth]# connect <device address>
[bluetooth]# disconnect <device address>
```

## FreeRTOS Task Priorities

Higher numbers run first. The system has priority levels 0 to 9, and the idle task runs at 0.

| Task | Priority | Role |
|------|----------|------|
| BLE stack | 9 | BLE link layer radio scheduling |
| APP | 9 | Sets the DMM scheduling state at startup, then suspends |
| BLE ICall worker | 8 | BLE stack service calls |
| MUX TX | 7 | Keepalive transmission |
| BLE RX delivery | 7 | Passes received BLE HCI data from the UART interrupt to the BLE stack |
| NPI | 7 | Sends BLE HCI responses to the host |
| ZBOSS (Zigbee NCP) | 5 | Zigbee NCP firmware |
| Timer daemon | 5 | FreeRTOS software timers |

## Verify Concurrent Operation

Run Zigbee and Bluetooth at the same time:

| Check | Expected result |
|-------|-----------------|
| Zigbee host application joins or forms a network | Network is formed and data frames are acknowledged |
| `scan le` in `bluetoothctl` while Zigbee traffic is running | Nearby BLE devices are listed |
| Connect a BLE device while Zigbee traffic is running | Both connections stay active |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| The bridge prints no Zigbee port | Start the bridge with `--zb`. |
| The bridge prints no output | Check the serial port with `ls /dev/ttyACM*` and use the application UART. |
| The Zigbee host application gets no response | Use the Zigbee port printed by the bridge, and check that the host libraries use 921600 baud. |
| `hciattach` fails | Use the BLE port printed by the bridge. |
| Zigbee frames are lost | Increase the bridge option `--pace-us-per-byte`. |
| A BLE scan fails with an authentication error under heavy Zigbee traffic | Reduce the Zigbee traffic rate. |

After restarting the bridge, the virtual serial ports change. Restart the host applications with
the new ports.
