# OpenThread NCP Example


# Introduction
This document describes how to setup the NCP + OTBR setup working with a CLI-FTD application. Following this document you will be able to: 
* Create a Thread network via OTBR + NCP setup
* Join a node to the thread network created 
* Send messages between the two thread network devices
# NOTE: THE BORDER ROUTER WITH NCP IS STILL EXPERIMENTAL AND UNDER DEVELOPMENT BY OPENSOURCE COMMUNITY. PLEASE REFER TO HERE FOR UPDATES: [OTBR WEBSITE](https://github.com/openthread/ot-br-posix/issues/2398)

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


Connect NCP to border router and restart border router to make sure all configurations have taken effect.

Open a terminal within the border router and set up network information.

```bash
sudo ot-ctl panid 0xface
Done
sudo ot-ctl channel 11
Done
sudo ot-ctl networkkey 00112233445566778899aabbccddeeff
Done
sudo ot-ctl ifconfig up
Done
sudo ot-ctl thread start
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

## 4. Start node 1
Set up the basic network information for the node.


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



## 5. Ping Node 1 from NCP

Get IP address of Node 1

```bash
> ipaddr rloc
fd9e:6062:a089:68d:0:ff:fe00:4800
Done
```

Ping Node 1 from NCP
```bash
sudo ot-ctl ping fd9e:6062:a089:68d:0:ff:fe00:4800
18 bytes from fd9e:6062:a089:68d:0:ff:fe00:4800: icmp_seq=1 hlim=64 time=24ms
```



## 5. Ping NCP from Node 1

```bash
> ping fe80::3cb1:87fc:9a38:d8bc
16 bytes from fe80:0:0:0:3cb1:87fc:9a38:d8bc: icmp_seq=2 hlim=64 time=25ms
```
