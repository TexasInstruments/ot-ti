# OpenThread RCP-HCI Example


# Introduction
This document describes how to communicate with both Thread and BLE devices using a single TI CC1354P10-6 attached to a Linux machine.
This is accomplished by the TI CC1354P10-6 device running the DMM (RCP + BLE-HCI) example interfacing with a Linux host running OTBR and BlueZ.
Following this document you will be able to:
* Create a Thread network via OTBR + RCP-HCI setup
* Join a node to the Thread network created
* Send messages between the two Thread network devices
* Scan for active BLE peripheral devices via BlueZ + RCP-HCI setup
* Connect to a BLE peripheral device

<div style="text-align: center;">
  <img src="resources/rcp-hci-setup.png" alt="Figure 1. Example RCP-HCI Setup">
  <div class="caption">Figure 1. Example RCP-HCI Setup</div>
</div>

# Software Prerequisites
- [Border Router software](https://github.com/openthread/ot-br-posix)
- [UniFlash](https://www.ti.com/tool/UNIFLASH)
- x86 based Linux environment for application builds

# Hardware Prerequisites
This application has been verified to work with the following OTBR/BLE host setups
Border Router:

    - [Raspberry Pi](https://www.raspberrypi.com/)
    - OR Generic Linux Ubuntu 22.04 host

- UART FTDI cable (Pins: DIO46-TX, DIO47-RX)
- [SimpleLink CC1354P10-6 Launchpad](https://www.ti.com/tool/LP-EM-CC1354P10)

Linux BLE Controller Host:
- Raspberry PI 4
- OR Generic Linux Ubuntu 22.04 host
Note: Host must have compatible Linux Ubuntu 22.04+ OR Debian image with Native BlueZ & HCI Config support

A Beaglebone Black may be used for a host, but has not been verified with this application:
- [Beagle Bone Black](https://www.beagleboard.org/boards/beaglebone-black)

FTD/MTD: Boards listed below for secondary Thread device which will join the network started by the OTBR + TI Device running the DMM (RCP+BLE-HCI) example
 
- [SimpleLink CC1352P2 Launchpad](https://www.ti.com/tool/LAUNCHXL-CC1352P)
- [SimpleLink CC1352P4 Launchpad](https://www.ti.com/tool/LAUNCHXL-CC1352P)
- [SimpleLink CC1352P7 Launchpad](https://www.ti.com/tool/LP-CC1352P7)
- [SimpleLink CC1352P7-4 Launchpad](https://www.ti.com/tool/LP-CC1352P7)
- [SimpleLink CC1354P10-1 Launchpads](https://www.ti.com/tool/LP-EM-CC1354P10)
- [SimpleLink CC26X2R1 Laundpads](https://www.ti.com/tool/LAUNCHXL-CC26X2R1)
- [SimpleLink CC2652PSIP Launchpad](https://www.ti.com/tool/LP-CC2652RSIP)
- [SimpleLink CC2652R7 Launchpad](https://www.ti.com/tool/LP-CC2652R7)
- [SimpleLink CC2652RB Launchpad](https://www.ti.com/tool/LP-CC2652RB)  
- [SimpleLink CC2652RSIP Launchpad](https://www.ti.com/tool/LP-CC2652RSIP)

Serial Terminal
- [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)
- [Tera Term](https://osdn.net/projects/ttssh2/releases)
- [RealTerm](https://sourceforge.net/projects/realterm/)
- [Windows PowerShell](https://learn.sparkfun.com/tutorials/terminal-basics/command-line-windows-mac-linux)

# Building the Example
Instructions may be found in the Building section within the top level [README](../../../README.md) for the CC1354P10-6 platform.

# Example Usage

##  Pre-work

Check to see if any Bluetooth adapters are already active
```bash
> sudo hciconfig
hci0:   Type: Primary  Bus: UART
        BD Address: XX:XX:XX:XX:XX:XX  ACL MTU: 255:5  SCO MTU: 0:0
        UP RUNNING 
        RX bytes:2505 acl:0 sco:0 events:77 errors:0
        TX bytes:283 acl:0 sco:0 commands:34 errors:0
```

Disable existing adapters, since later on we wish to use the TI Device as the sole Bluetooth LE adapter.
```bash
> sudo hciconfig hci0 down
```

## 0. Identify relevant UART Ports

The RCP-HCI application utilizes two UART ports on the device; the FTDI cable is used for the RCP->OTBR connection and the XDS110 UART is used for BLE HCI->Linux Host connection.
Plug in the RCP-HCI Launchpad to the Linux Host machine using the XDS110 UART cable and FTDI cable.

Identify which ports are active on the command line via:
```bash
ls /dev/tty*
```
Note: You should see three ports, two from the Launchpad (usually /dev/ttyACM0 for HCI) and one for the FTDI cable (usually /dev/ttyUSB0 for RCP).

## 1. Set Up Linux Host

Note: Only complete Task 1 from the Simplelink Academy link below to set up the border router (OTBR).

[Border router set up guide](https://openthread.io/guides/border-router/build) 
When running the OTBR, make sure to use the UART port associated with the FTDI cable, this should be /dev/ttyUSB0.

Note: For initial setup of an RPI follow the [Matter User's Guide](https://github.com/project-chip/certification-tool/blob/main/docs/Matter_TH_User_Guide/Matter_TH_User_Guide.adoc#fresh_install)

The following link can be used to setup [BlueZ](https://github.com/project-chip/connectedhomeip/blob/master/docs/guides/BUILDING.md) on a fresh install.

## 2. Build and Flash RCP + HCI-BLE

Navigate to the root of the ot-ti repository.
To obtain a list of supported platforms, execute ./script/build

On first time build, run bootstrap script.
```bash
cd ot-ti
./script/bootstrap
```
Build image for your platform.
```bash
cd ot-ti
./script/build <Platform> -DBLE_HCI=1
```
For this document, the specific platform is CC1354P10-6:
```bash
./script/build LP_EM_CC1354P10_6 -DBLE_HCI=1
```
Once built the images will be in ot-ti/build/bin.

Flash the board using UniFlash with the image generated (ot-rcp-hci.out).
## 3. Start RCP

Connect RCP to the Linux Host running OTBR (setup from step 1), then restart OTBR to make sure all configurations have taken effect.
```bash
sudo systemctl restart otbr-agent.service
```

Open a separate terminal within the OTBR and set up Thread network information.

```bash
sudo ot-ctl dataset init new
Done
sudo ot-ctl dataset panid 0xface
Done
sudo ot-ctl dataset channel 11
Done
sudo ot-ctl dataset networkkey 00112233445566778899aabbccddeeff
Done
sudo ot-ctl dataset commit active
Done
```

Bring up the IPv6 interface:
```bash
sudo ot-ctl ifconfig up
Done
```

Start Thread protocol operation:
```bash
sudo ot-ctl thread start
Done
```

Check status of network after a few seconds
```bash
sudo ot-ctl state
leader
Done
```

## 4. Start Thread Node 1
Set up the basic Thread network information for the node.
(Note you will need to build this image similarly to step 2. Recommended to use the cli-ftd example.)

```bash
> networkkey 00112233445566778899aabbccddeeff
Done
> panid 0xface
Done
> channel 11
Done
```

Bring up the IPv6 interface:

```bash
> ifconfig up
Done
```

Start Thread protocol operation:

```bash
> thread start
Done
```

Wait a few seconds and verify that the device has become a Thread child:

```bash
> state
child
Done
```

## 5. Ping Node 1 from RCP

Get IP address of Node 1

```bash
> ipaddr rloc
fd9e:6062:a089:68d:0:ff:fe00:4800
Done
```

Ping Node 1 from RCP
```bash
sudo ot-ctl ping fd9e:6062:a089:68d:0:ff:fe00:4800
18 bytes from fd9e:6062:a089:68d:0:ff:fe00:4800: icmp_seq=1 hlim=64 time=24ms
```
## 6. Configure BLE on the Linux Host

Note: Replace the device with the corresponding LP UART port. This should be /dev/ttyACM0
```bash
sudo hciattach /dev/ttyACM0 any 115200
```

## 7. Scan for BLE devices

Find BDADDR (XX:XX:XX:XX:XX:XX) of HCI adapter.

```bash
> sudo hciconfig
hci0:   Type: Primary  Bus: UART
        BD Address: XX:XX:XX:XX:XX:XX  ACL MTU: 255:5  SCO MTU: 0:0
        UP RUNNING 
        RX bytes:2505 acl:0 sco:0 events:77 errors:0
        TX bytes:283 acl:0 sco:0 commands:34 errors:0
```

Select default BT adapter, where BDADDR is the Bluetooth adapter address.

```bash
bluetoothctl

> select XX:XX:XX:XX:XX:XX
```

To make sure this is setup correctly, try to scan for Bluetooth LE devices.

```bash
bluetoothctl

> scan le
```

## 8. Connect to target device

Any Bluetooth LE peripheral may be connected to with RCP-HCI.
If you want to connect to another TI Simplelink Peripheral, you can set that device up using this [Simplelink Academy](https://dev.ti.com/tirex/explore/node?node=A__AX8fD.bKB7yAgQmXFl2j4A__com.ti.SIMPLELINK_ACADEMY_CC13XX_CC26XX_SDK__AfkT0vQ__LATEST).

You can perform another scan to find the address of the target peripheral to connect with.

```bash
bluetoothctl

> connect XX:XX:XX:XX:XX:XX
```