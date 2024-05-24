# Example Applications

This section provides an overview of the out-of-box example applications.

## Factory Reset 

Some of the example applications contain convenience functionality to
reset the `non-volatile storage <NVS>` on
the internal flash, as well as a reset to some stored factory image.
Refer to each respective section for instructions on how to perform each
kind of factory reset. Factory reset is only supported via CLI command only, `factoryreset`. 

### Reset of Non-Volatile Storage 

`Non-volatile storage (NVS) <NVS>` in the
internal flash stores information about the Thread network which allows
Thread devices to restart network operations after a reset without user
intervention. However, in order for a Thread device to *forget* a Thread
network it has been commissioned to, the NVS must be erased.

For the end-product examples, a convenience functionality has been added
in order to easily erase NVS and quickly start fresh. Given that the
running application supports this functionality, use cli command `factoryreset`
to reset `NVS`. 

**_Warning:_** Reset of NVS does not erase the actual application image on the internal
flash.

## End Product Examples 

### Command Line Interface (CLI)

The CLI projects can be used to interact with and explore the different
aspects of the Thread protocol.
The CLI interface OpenThread presents is also used for certification
with the Thread Group test harness.

-   [CLI README](https://github.com/TexasInstruments/ot-ti/blob/main/examples/apps/cli/README.md)

### Network Co-Processor (NCP) and Radio Co-Processor (RCP)

The project is used to connect the to a host processor; for more information about the interface
between the NCP/RCP and host processor: 

-   [NCP README](https://github.com/TexasInstruments/ot-ti/blob/main/examples/apps/ncp/README.md)
-   [RCP README](https://github.com/TexasInstruments/ot-ti/blob/main/examples/apps/rcp/README.md)