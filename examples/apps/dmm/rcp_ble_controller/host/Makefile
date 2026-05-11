# Makefile for host_mux_ot_ble — Linux host daemon for DMM RCP+BLE Controller
#
# Prerequisites:
#   - GCC (or set CC to your cross-compiler for arm/x86)
#   - TI SimpleLink SDK path set via TI_SIMPLELINK_SDK_DIR environment variable
#     or passed on the make command line.
#
# Usage:
#   TI_SIMPLELINK_SDK_DIR=/path/to/sdk make
#   ./host_mux_ot_ble --device /dev/ttyACM0 --baud 921600

CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -O2 -g

# Path to the TI SimpleLink SDK (must be set by the user)
SDK     ?= $(TI_SIMPLELINK_SDK_DIR)

COMBINED_SERIAL_DIR = $(SDK)/source/ti/dmm/combined_serial

INCLUDES = \
    -I$(COMBINED_SERIAL_DIR) \
    -I$(COMBINED_SERIAL_DIR)/host

SRCS = \
    host_mux_ot_ble.c \
    $(COMBINED_SERIAL_DIR)/hdlc_spinel.c \
    $(COMBINED_SERIAL_DIR)/mux_buffer.c \
    $(COMBINED_SERIAL_DIR)/host/host_mux_uart.c

TARGET = host_mux_ot_ble

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^

clean:
	rm -f $(TARGET)
