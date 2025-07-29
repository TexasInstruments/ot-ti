# NCP Interface

The OpenThread stack runs on the while the application is executed on a
separate external host processor. The host processor communicates with
the NCP device via a serial interface, such as UART or SPI, using the
[Spinel Protocol](https://github.com/openthread/spinel-spec).

# Spinel Protocol 

Spinel is a general management protocol for enabling a host device to
communicate with and manage an NCP. For more information, see the
Internet-Draft for the [Spinel Protocol](https://github.com/openthread/spinel-spec).

A Python CLI tool called Pyspinel is available for testing purposes. For
more information, see the [Pyspinel GitHub repository](https://github.com/openthread/pyspinel).


# Serial Interface Configuration 

TI-OpenThread supports Spinel to be configured over UART.