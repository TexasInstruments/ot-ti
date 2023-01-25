[![Build][ot-gh-action-build-svg]][ot-gh-action-build]

[ot-gh-action-build]: https://github.com/openthread/ot-cc13x2-cc26x2/actions?query=workflow%3ABuild+branch%3Amain+event%3Apush
[ot-gh-action-build-svg]: https://github.com/openthread/ot-cc13x2-cc26x2/workflows/Build/badge.svg?branch=main&event=push

---

# OpenThread CC13XX_CC26XX Example

This directory contains the platform drivers necssary to run OpenThread on the Texas Instruments CC13XX_CC26XX family of
Connected MCUs. These drivers use the TI SimpleLink™ SDK for the RTOS enabled platform drivers. The example applications
build with FreeRTOS to enable an environment for the standard device drivers to operate.

## Toolchain

In a Bash terminal, follow these instructions to install the GNU toolchain and other dependencies.

```bash
$ cd <path-to-ot-cc13x2-cc26x2>
$ git submodule update --init
$ ./script/bootstrap
```

## Building

In a Bash terminal, follow these instructions to build the cc13xx_cc26xx examples. Where the SimpleLink board is a
reference development kit available on ti.com.

```bash
$ cd <path-to-ot-cc13x2-cc26x2>
$ ./script/build LP_CC2652R7
```

## Flash Binaries

If the build completed successfully, the `elf` files may be found in `<path-to-ot-cc13x2-cc26x2>/build/bin/`. These
files do not have any file extension.

Loading the built image onto a LaunchPad is supported through two methods; Uniflash and Code Composer Studio (CCS).
UniFlash can be used to load the image. Code Composer Studio can be used to load the image and debug the source code.

### Code Composer Studio

Programming with CCS will allow for a full debug environment within the IDE. This is accomplished by creating a target
connection to the XDS110 debugger and starting a project-less debug session. The CCS IDE will attempt to find the source
files on the local machine based on the debug information embedded within the ELF. CCS may prompt you to find the source
code if the image was built on another machine or the source code is located in a different location than is recorded
within the ELF.

Download and install [Code Composer Studio][ccs].

First open CCS and create a new workspace.

Create a target connection (sometimes called the CCXML) for your target SoC and debugger as described in the [Manual
Method][ccs_manual_method] section of the CCS User's Guide.

Next initiate a project-less debug session as described in the [Manual Launch][ccs_manual_launch] section of the CCS
User's Guide.

CCS should switch to the debug view described in the [After Launch][ccs_after_launch] section of the User's Guide. The
SoC core will likely be disconnected and symbols will not be loaded. Connect to the core as described in the [Debug
View][ccs_debug_view] section of the User's Guide. Once the core is connected, use the `Load` button on the toolbar to
load the ELF image.

Note that the default configuration of the CCXML uses 2-wire cJTAG instead of the full 4-wire JTAG connection to match
the default jumper configuration of the LaunchPad.

[ccs]: https://www.ti.com/tool/CCSTUDIO
[ccs_after_launch]: https://software-dl.ti.com/ccs/esd/documents/users_guide/ccs_debug-main.html?configuration#after-launch
[ccs_debug_view]: https://software-dl.ti.com/ccs/esd/documents/users_guide/ccs_debug-main.html?configuration#debug-view
[ccs_manual_launch]: https://software-dl.ti.com/ccs/esd/documents/users_guide/ccs_debug-main.html?configuration#manual-launch
[ccs_manual_method]: https://software-dl.ti.com/ccs/esd/documents/users_guide/ccs_debug-main.html?configuration#manual-method

### UniFlash

Uniflash is Texas Instrument's uniform programming tool for embedded processors. This will allow you to erase, flash,
and inspect the SoC without setting up a debugging environment.

Download and install [UniFlash][uniflash].

First open UniFlash. Debug probes connected to the computer will usually be displayed under the Detected Devices due to
the automatic device detection feature. If your device does not show up in this view it my be disconnected, or you may
have to create a New Configuration. If you already have a CCXML for your SoC and debug connection you can use that in
the section at the bottom. Once your device is selected, click the `Start` button within the section to launch the
session.

Select the ELF image to load on the device with the `Browse` button. Make sure to deselect the binary check-box,
Uniflash assumes a file without an extension is a binary file and not an elf. Click the `Load Image` button to load the
executable image onto the device. You should be able to see the log output over the XDS110 User UART.

Note that programming the device through JTAG sets the Halt-in-Boot flag and may cause issues when performing a software
reset. This flag can be reset by power-cycling the LaunchPad.

[uniflash]: https://www.ti.com/tool/download/UNIFLASH

## Interact

By default the terminal output will be sent to the Application/User UART. This can be found in Windows in the Device
Manager or in Linux as the `/dev/ttyACM0` device. Open a terminal emulator to that port to see the output with the
following options:

| Parameter    | Value    |
| ------------ | -------- |
| Speed (baud) | `115200` |
| Data bits    | `8`      |
| Stop bits    | `1`      |
| Parity       | `None`   |
| Flow control | `None`   |

1. Open terminal to the com port associated with the User UART.
2. Type `help` for list of commands.
3. See [OpenThread CLI Reference README.md][cli] to learn more.

[cli]: https://github.com/openthread/openthread/blob/main/src/cli/README.md

## TI Support

For technical support, please consider creating a post on TI's [E2E forum][e2e]. Additionally, we welcome any feedback.

[e2e]: https://e2e.ti.com/support/wireless-connectivity/zigbee-and-thread

# License

OpenThread is released under the [BSD 3-Clause
license](https://github.com/openthread/ot-cc13x2-cc26x2/blob/main/LICENSE). See the
[`LICENSE`](https://github.com/openthread/ot-cc13x2-cc26x2/blob/main/LICENSE) file for more information.

Please only use the OpenThread name and marks when accurately referencing this software distribution. Do not use the
marks in a way that suggests you are endorsed by or otherwise affiliated with Nest, Google, Texas Instruments or The
Thread Group.

# Need help?

OpenThread support is available on GitHub:

- Bugs and feature requests pertaining to the OpenThread on CC13x2/CC26x2 Examples — [submit to the
  openthread/ot-cc13x2-cc26x2 Issue Tracker](https://github.com/openthread/ot-cc13x2-cc26x2/issues)
- OpenThread bugs and feature requests — [submit to the OpenThread Issue
  Tracker](https://github.com/openthread/openthread/issues)
- Community Discussion - [ask questions, share ideas, and engage with other community
  members](https://github.com/openthread/openthread/discussions)
