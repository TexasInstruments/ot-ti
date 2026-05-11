# Zigbee NCP + OpenThread NCP + BLE Controller

This example runs a Zigbee network co-processor (NCP), an OpenThread NCP, and a Bluetooth LE
controller at the same time on a single TI CC27XX device. A Linux host runs the host stack of each
protocol, and the device shares its radio between the three protocols.

| Stack | Runs on the device | Runs on the Linux host |
|-------|--------------------|------------------------|
| Thread | OpenThread NCP | OpenThread Border Router (`otbr-agent`) |
| Bluetooth LE | Controller (HCI) | BlueZ |
| Zigbee | ZBOSS NCP | TI Zigbee Linux Host SDK |

All stacks communicate with the host over one USB serial connection. The `spinel_bridge.py` script
on the host splits that connection into one virtual serial port per stack.

```
┌─────────────────────────────────────────────────────────────────────┐
│  CC27XX Device                                                      │
│    ┌─────────┐  ┌────────────┐  ┌────────┐  ┌─────────────────┐     │
│    │ Thread  │  │ BLE        │  │ Zigbee │  │ Combined Serial │     │
│    │ NCP     │  │ Controller │  │ NCP    │  │ MUX Task        │     │
│    └────┬────┘  └──────┬─────┘  └────┬───┘  └────────┬────────┘     │
│         │              │             │               │              │
│         └──────────────┴─────────────┴───────────────┘              │
│                      Single UART (921600 baud)                      │
└──────────────────────────────────┬──────────────────────────────────┘
┌──────────────────────────────────┴──────────────────────────────────┐
│  Linux Host                                                         │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │  spinel_bridge.py (MUX demux)                                 │  │
│  │    NLI=0 → Thread PTY → otbr-agent                            │  │
│  │    NLI=1 → BLE PTY    → hciattach / BlueZ                     │  │
│  │    NLI=2 → Zigbee PTY → Zigbee host application               │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Requirements

**Hardware**

- One of these LaunchPads, with a USB cable:
  - [LP-EM-CC2755P20](https://www.ti.com/tool/LP-EM-CC2755P20)
- A second Thread device to form a Thread network, for example a CC27XX LaunchPad running
  the `cli-ftd` example
- A second Zigbee device to form a Zigbee network

**Host software**

- Linux (Ubuntu 22.04 recommended)
- Python 3.7 or later
- [OpenThread Border Router](https://github.com/openthread/ot-br-posix)
- [BlueZ](https://github.com/bluez/bluez): `sudo apt install bluez`
- TI Zigbee Linux Host SDK
- [UniFlash](https://www.ti.com/tool/UNIFLASH) or Code Composer Studio, to flash the device

Install the build toolchain by following the instructions in the top-level `README.md`.

## Build the Firmware

From the root of the repository:

```bash
./script/build LP_EM_CC2755P20 \
  -DOT_APP_ZIGBEE_NCP_OT_NCP_BLE_CONTROLLER=1 \
  -DNCP_BUILD_ENABLED=ON \
  -DOT_NCP_DNSSD=ON \
  -DOT_NCP_CLI_STREAM=ON \
  -DOT_NCP_INFRA_IF=ON \
  -DOT_BORDER_ROUTING=ON \
  -DOT_BORDER_ROUTER=ON \
  -DOT_PING_SENDER=ON
```

Replace `LP_EM_CC2755P20` with the name of your LaunchPad. The build produces two images:

| Image | Thread device type |
|-------|--------------------|
| `build/bin/ot-zigbee-ncp-ble-controller-ftd.out` | Full Thread device |
| `build/bin/ot-zigbee-ncp-ble-controller-mtd.out` | Minimal Thread device |

To build only one image, add `-DOT_MTD=OFF` or `-DOT_FTD=OFF` to the command.

## Flash the Firmware

Flash the image you want to use to the LaunchPad using UniFlash or Code Composer Studio.

## Run the Example

Start the components in the order given below: Zigbee, then Thread and Bluetooth. Starting
Thread or Bluetooth before Zigbee can crash the device.

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
THREAD PTY: /dev/pts/3
BLE PTY: /dev/pts/4
ZB PTY: /dev/pts/5
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

### 4. Start the OpenThread Border Router

```bash
sudo otbr-agent -I wpan0 -B <network interface> "spinel+hdlc+uart://<THREAD PTY>" --verbose
```

Replace `<network interface>` with your LAN interface, for example `eth0`.

### 5. Form a Thread network

```bash
sudo ot-ctl dataset init new
sudo ot-ctl dataset commit active
sudo ot-ctl ifconfig up
sudo ot-ctl thread start
```

After about 30 seconds, check the device state:

```bash
sudo ot-ctl state
```

The state is `leader`.

### 6. Join a second Thread device

Read the active dataset from the leader:

```bash
sudo ot-ctl dataset active -x
```

On the second device, enter the dataset and start Thread:

```
dataset set active <dataset>
dataset commit active
ifconfig up
thread start
```

Get the address of the second device and ping it from the host:

```bash
sudo ot-ctl ipaddr rloc
sudo ot-ctl ping <rloc address>
```

### 7. Attach the Bluetooth controller

```bash
sudo hciattach <BLE PTY> any 921600
hciconfig
```

`hciconfig` lists a new adapter with `Bus: UART` in the `UP RUNNING` state.

### 8. Use Bluetooth LE

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
| BLE ICall worker | 8 | BLE stack service calls |
| MUX TX | 7 | Keepalive transmission |
| BLE RX delivery | 7 | Passes received BLE HCI data from the UART interrupt to the BLE stack |
| NPI | 7 | Sends BLE HCI responses to the host |
| APP (OpenThread NCP) | 6 | OpenThread NCP event loop |
| ZBOSS (Zigbee NCP) | 5 | Zigbee NCP firmware |
| Timer daemon | 5 | FreeRTOS software timers |

## Verify Concurrent Operation

Run all three protocols at the same time:

| Check | Command | Expected result |
|-------|---------|-----------------|
| Thread state | `sudo ot-ctl state` | `leader` or `router` |
| Thread ping | `sudo ot-ctl ping <rloc address>` | Replies are received |
| BLE scan | `scan le` in `bluetoothctl` | Nearby BLE devices are listed |
| BLE advertising | Phone BLE scanner app | `TI-CC27XX-DMM` is visible |
| Zigbee | Zigbee host application log | Network is formed and data frames are acknowledged |

## Notes

A reset requested by `otbr-agent` resets only the Thread stack. The device does not reboot, so the
Zigbee and Bluetooth stacks keep running.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| The bridge prints no Zigbee port | Start the bridge with `--zb`. |
| The bridge prints no output | Check the serial port with `ls /dev/ttyACM*` and use the application UART. |
| `otbr-agent` reports no response from the NCP | Restart the bridge and use the new Thread port. Check that the device runs this example's firmware. |
| The Zigbee host application gets no response | Use the Zigbee port printed by the bridge, and check that the host libraries use 921600 baud. |
| `hciattach` fails | Use the BLE port printed by the bridge. |
| Thread pings are dropped during heavy Zigbee traffic | Reduce the Zigbee traffic rate. |
| A BLE scan fails with an authentication error under heavy traffic | Reduce the Thread and Zigbee traffic rate. |

After restarting the bridge, the virtual serial ports change. Restart the host applications with
the new ports.
