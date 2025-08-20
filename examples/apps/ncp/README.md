# OpenThread NCP Example


# Introduction
This document describes how to setup the NCP + OTBR setup working with a CLI-FTD application. Following this document you will be able to: 
* Create a Thread network via OTBR + NCP setup
* Join a node to the thread network created 
* Send messages between the two thread network devices
# NOTE: THE BORDER ROUTER WITH NCP IS STILL EXPERIMENTAL AND UNDER DEVELOPMENT BY OPENSOURCE COMMUNITY. PLEASE REFER TO HERE FOR UPDATES: [OTBR WEBSITE](https://github.com/openthread/ot-br-posix/issues/2398)

# Software Prerequisites
- [Border Router software](https://github.com/openthread/ot-br-posix)
- [UniFlash](https://www.ti.com/tool/UNIFLASH)
# Hardware Prerequisites
Border Router:
- [Beagle Bone Black](https://www.beagleboard.org/boards/beaglebone-black)
- [Raspberry Pi](https://www.raspberrypi.com/)

NCP/FTD: Two LaunchPads from the list of supported [TI devices](../../../README.md#openthread-simplelink-example)

Serial Terminal
- [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)
- [Tera Term](https://osdn.net/projects/ttssh2/releases)
- [RealTerm](https://sourceforge.net/projects/realterm/)
- [Windows PowerShell](https://learn.sparkfun.com/tutorials/terminal-basics/command-line-windows-mac-linux)

# Functional Description

- [NCP Overview](https://openthread.io/platforms/co-processor)

# Example Usage

## 1. Set up Border Router

Note: Only complete Task 1 from the simplelink academy link below to set up the border router. Other tasks are not required to get the NCP working. 

[Border router set up guide](https://dev.ti.com/tirex/explore/node?node=A__AUviXt3yUXFwOz5WHh5IlQ__com.ti.SIMPLELINK_ACADEMY_CC13XX_CC26XX_SDK__AfkT0vQ__LATEST&search=thread) 

## 2. Build and flash NCP

To obtain a list of supported platforms input ./script/build

Uncomment the following code from ot-ti/script/build to properly build the NCP.
```
"-DNCP_BUILD_ENABLED=ON"
"-DOT_NCP_DNSSD=ON"
"-DOT_NCP_CLI_STREAM=ON"
"-DOT_NCP_INFRA_IF=ON"
"-DOT_BORDER_ROUTING=ON"
```

Once uncommented run the following commands to build the image for your platform.

```bash
cd ot-ti
./script/build <Platform>
```
Once built the images will be in ot-ti/build/bin.

Flash the board using UniFlash or CCS with the image generated.
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
