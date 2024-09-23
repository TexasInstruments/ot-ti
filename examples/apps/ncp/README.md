# OpenThread NCP Example


# Introduction
This document describes how to setup the NCP + OTBR setup working with a CLI-FTD application. Following this document you will be able to: 
* Create a Thread network via OTBR + NCP setup
* Join a node to the thread network created 
* Send messages between the two thread network devices

# Software Prerequisites
- [Border Router software](https://github.com/openthread/ot-br-posix)
- [Wpantund software](https://github.com/openthread/wpantund)
- [UniFlash](https://www.ti.com/tool/UNIFLASH)
# Hardware Prerequisites
Border Router:
- [Beagle Bone Black](https://www.beagleboard.org/boards/beaglebone-black)
- [Raspberry Pi](https://www.raspberrypi.com/)

NCP/FTD: Two launch pads of boards listed below
 
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

# Functional Description

- [NCP Overview](https://openthread.io/platforms/co-processor)

- [Wpantund Overview](https://github.com/openthread/wpantund/tree/master)


# Example Usage

## 1. Set up Border Router and wpantund

Note: Only complete Task 1 from the simplelink academy link below to set up the border router. Other tasks are not required to get the NCP working. 

[Border router set up guide](https://dev.ti.com/tirex/explore/node?node=A__AUviXt3yUXFwOz5WHh5IlQ__com.ti.SIMPLELINK_ACADEMY_CC13XX_CC26XX_SDK__AfkT0vQ__LATEST&search=thread) 

[WPANTUND install guide](https://github.com/openthread/wpantund/blob/master/INSTALL.md)


## 2. Build and flash NCP

To obtain a list of supported platforms input ./script/build

```bash
cd ot-ti
./script/build <Platform>
```
Once built the images will be in ot-ti/build/bin.

Flash the board using UniFlash with the image generated.
## 3. Start NCP

From the border router set up basic network info to create a network
```bash
sudo wpanctl set Network:Key 00112233445566778899aabbccddeeff
sudo wpanctl set Network:PANID 0xface
sudo wpanctl set NCP:Channel 11
```
Form the network
```bash
sudo wpanctl form test
```

Check status of network
```
sudo wpanctl status
wpan0 => [
    "NCP:State" => "associated"
    "Daemon:enable" => true
    "NCP:Version" => "OPENTHREAD/1.3.0.1; CC13XX_CC26XX; Jan 19 2024 09:38:17"
    "Daemon:Version" => "0.08.00d (0.07.01-366-ge2fd726; Jan 11 2024 19:05:22)"
    "Config:NCP:DriverName" => "spinel"
    "NCP:HardwareAddress" => [00124B0014FE56E1]
    "Network:NodeType" => "leader"
    "Network:Name" => "test"
    "Network:XPANID" => 0xD7936715C2F0F95B
    "Network:PANID" => 0xface
    "IPv6:LinkLocalAddress" => "fe80::3cb1:87fc:9a38:d8bc"
    "IPv6:MeshLocalAddress" => "fdd7:9367:15c2:0:bdb5:360c:49f1:c854"
    "IPv6:MeshLocalPrefix" => "fdd7:9367:15c2::/64"
]
```
Note: the information displayed for your system will be different.

## 4. Start node 1
Set up the basic network information for the node.

Note: Make sure the channel and Panid listed from status command match what is input here.

```bash
> networkkey 00112233445566778899aabbccddeeff
Done
>panid 0xface
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



## 5. Ping NCP from Node 1

```bash
> ping fe80::3cb1:87fc:9a38:d8bc
16 bytes from fe80:0:0:0:3cb1:87fc:9a38:d8bc: icmp_seq=2 hlim=64 time=25ms
```
