# Examples Directory

This directory should be a faithful copy of the examples directory from
OpenThread with a few changes to support the specific build requirements for
the TI SimpleLink devices. These include;

1. Addition of the `*.out` suffix to executables
2. Enabling RTOS operation

## Adding out to the Executable

Both CCS and Uniflash recognize files with the `*.out` suffix as programmable
files. An executable file with no extension is not always recognized by default
in the file explorer of the loading programs. And when loading a file with no
suffix Uniflash will default to interpreting the file as a binary image. While
an extension of `*.elf` may be more descriptive, it is not recognized by
default in some programs.

## Enabling an RTOS

The SimpleLink drivers require a kernel running underneath them for certain
primitives like semaphores. This requires the `main()` function to be used to
setup the drivers and start the kernel. It is not easy to re-write the
SimpleLink SDK `ResetISR` to create a new entry point. Instead a file has
been added to each example, one to bootstrap FreeRTOS. The `main()` function in the example applications have been re-named
to `app_main()` to follow the conventions set in the SimpleLink SDK.

### Possible changes

Currently the `otSysProcessDrivers()` in `src/system.c` pends on a processing
queue for driver events. Once a driver generates an event and places that on
the queue, the function will continue to process this event. The two examples
in this repository also define `otTaskletsSignalPending()` and handle
processing tasklets within their main loops. This is handled by checking if
there are any pending tasklets before pending on the queue and returning if
there are any.

A weak implementation of the `otTaskletsSignalPending()` function is provided
in the `system.c` file and `OT_TASKLET_SIGNALING` should be defined if the
application code does not handle this functionality.

