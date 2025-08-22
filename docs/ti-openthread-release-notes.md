# TI Openthread thread-v1.4-ti-1.0-EA-1.2 Release

## Introduction

This Texas Instruments OpenThread GitHub repository contains the software development tools that enable engineers to develop Thread Device and is the starting point for Thread development on all SimpleLink™ Thread devices.

- Thread 1.4 networking stack based on [OpenThread](https://github.com/TexasInstruments/ot-ti)

## Supported TI devices and Thread roles

| Device                   | RCP | MTD | FTD | NCP | RCP-BLE Controller | Production | Evaluation Only |
| ------------------------ | --- | --- | --- | --- | -------------- | --------------   | --------------  |
| [CC2652R][cc2652r]       | x   |     |     |     |                |                  |         X       |
| [CC2652RB][cc2652rb]     | x   |     |     |     |                |                  |         X       |
| [CC2652P][cc2652p]       | x   |     |     |     |                |                  |         X       |
| [CC2652RSIP][cc2652rsip] | x   |     |     |     |                |                  |         X       |
| [CC2652PSIP][cc2652psip] | x   |     |     |     |                |                  |         X       |
| [CC2652R7][cc2652r7]     | x   | x   | x   | x   |                |                  |         X       |
| [CC2652P7][cc2652p7]     | x   | x   | x   | x   |                |                  |         X       |
| [CC2674R10][cc2674r10]   | x   | x   | x   | x   | x              |                  |         X       |
| [CC2674P10][cc2674p10]   | x   | x   | x   | x   | x              |                  |         X       |
| [CC2340R5][cc2340r5]     | x   |     |     |     |                |                  |         X       |
| [CC2340R53][cc2340r53]   | x   | x   | x   | x   |                |                  |         X       |
| [CC2755R10][cc2755r10]   | x   | x   | x   | x   | x              |                  |         X       |
| CC2755P20 (coming soon)   | x   | x   | x   | x   |                |                  |         X       |

[cc2652r]: https://www.ti.com/product/CC2652R
[cc2652rb]: https://www.ti.com/product/CC2652RB
[cc2652p]: https://www.ti.com/product/CC2652P
[cc2652rsip]: https://www.ti.com/product/CC2652RSIP
[cc2652psip]: https://www.ti.com/product/CC2652PSIP
[cc2652r7]: https://www.ti.com/product/CC2652R7
[cc2652p7]: https://www.ti.com/product/CC2652P7
[cc2674r10]: https://www.ti.com/product/CC2674R10
[cc2674p10]: https://www.ti.com/product/CC2674P10
[cc2340r5]: https://www.ti.com/product/CC2340R5
[cc2340r53]: https://www.ti.com/product/CC2340R5
[cc2755r10]: https://www.ti.com/product/CC2755R10

Note: Previous Production Release: [TI-OpenThread v1.3-1.0 Release](https://github.com/TexasInstruments/ot-ti/releases/tag/thread-ti-v1.3-1.0)

## What's New
<!-- new features labelled with nf_thread-v1.4-ti-1.0-ea-1.2) -->

- TIOP-1455: Add suppport for CC2755P20 Platform
- TIOP-1439: Add configuration to simplify NCP build in CMake file and instructions in NCP example readme. 
  
## Fixed Issues
<!-- fixed issues labelled with fi_thread-v1.4-ti-1.0-ea-1.2) -->
- N/A
## Known Issues
<!-- known issues labelled with ki_thread-v1.4-ti-1.0-ea-1.2) -->
- TIOP-1452: SSED device is not supported on the CC23xx/CC27xx platform
- TIOP-1438: SSED may not function properly in high baudrate scenarios (921600) for CC1354/CC2674 platforms.

## Versioning 

* openthread commit: [c9c19aa9fa5877cf1532c35a584618900e5c99c7](https://github.com/openthread/openthread/commit/c9c19aa9fa5877cf1532c35a584618900e5c99c7)
* Simplelink F2 SDK (repo_cc13xx_cc26xx): 7.40.00.77 [374a26a] (https://github.com/TexasInstruments/simplelink-lowpower-f2-sdk/commit/374a26a45a5b05cd87c62d9a5da04d9e6d0ed319)
* Simplelink F3 SDK (repo_cc23xx_cc27xx): lpf3-9.20.00.10_ea
* FreeRTOS: v11.1.0
* SysConfig: 1.24.1
* Thread Border Router (OTBR) commit: [53125a4] (https://github.com/openthread/ot-br-posix/commit/53125a45f7f5a5b896c5bac5b85e6f9e8712bbe2)

## Technical Support and Product Updates
- [TI SimpleLink Solutions](https://www.ti.com/wireless-connectivity/overview.html)
- [TI E2E Community](https://e2e.ti.com/)
- [ti.com](https://www.ti.com/)
