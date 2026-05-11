# Design Proposal: NPI MUX Transport (`NPI_USE_MUX`)

## Problem

The DMM RCP+BLE Controller application uses a single physical UART shared between
OpenThread (spinel) and BLE (HCI) via the Combined Serial MUX.  To route BLE HCI
bytes through the MUX, the current build applies a **preprocessor shim**
(`mux_npi_uart_shim.h`) force-included only on `npi_tl_uart.c`:

```c
// mux_npi_uart_shim.h
#define UART2_open        MuxVirtUart_npiOpen
#define UART2_close       MuxVirtUart_npiClose
#define UART2_read        MuxVirtUart_npiRead
#define UART2_readCancel  MuxVirtUart_npiReadCancel
#define UART2_write       MuxVirtUart_npiWrite
```

This redirects every `UART2_*` call inside `npi_tl_uart.c` to a virtual UART
implementation (`mux_virt_uart.c`) that tunnels traffic through the MUX task.

### Problems with the shim approach

| # | Issue |
|---|-------|
| 1 | **Build trick** — requires a special `-include` compile flag applied to a single source file; not visible in the source code itself |
| 2 | **Fragile** — if `NPI_FLOW_CTRL=1` is ever enabled, `npi_tl_uart.c` directly dereferences `uartHandle->hwAttrs` for `UARTCharAvailable()`; this path cannot be intercepted by the shim |
| 3 | **Indirection** — data goes NPI → fake UART2 handle → ring buffer → MUX task, instead of NPI → MUX task directly |
| 4 | **Dead code** — `npi_tl_uart.c` is still compiled into `ble-stack-lib` even though its UART2 calls are replaced; the real UART is never opened |

---

## Proposed Solution

Add `NPI_USE_MUX` as a **first-class third transport** alongside the existing
`NPI_USE_UART` and `NPI_USE_SPI`, following the exact same pattern already
established in the NPI transport layer.

### How the existing transport pattern works

`npi_tl.h` uses compile-time macros to select a transport:

```c
#if defined(NPI_USE_UART)
#define transportInit         NPITLUART_initializeTransport
#define transportRead         NPITLUART_readTransport
#define transportWrite        NPITLUART_writeTransport
#define transportStopTransfer NPITLUART_stopTransfer
#define transportMrdyEvent    NPITLUART_handleMrdyEvent
#elif defined(NPI_USE_SPI)
#define transportInit         NPITLSPI_initializeTransport
...
#endif
```

`npi_tl.c` includes the matching header:

```c
#if defined(NPI_USE_SPI)
#include "npi_tl_spi.h"
#elif defined(NPI_USE_UART)
#include "npi_tl_uart.h"
#else
#error Must define an underlying serial bus for NPI
#endif
```

`npi_tl.c` then calls the transport through the macros:

```c
transportInit(npiRxBuf, npiTxBuf, NPITL_transmissionCallBack);  // open
transportWrite(len);                                              // TX
transportRead();                                                  // kick RX
```

Every transport must implement the same five functions:

| Function | Signature | Purpose |
|----------|-----------|---------|
| `initializeTransport` | `(Char *rxBuf, Char *txBuf, npiCB_t cb)` | Open transport, save buffers + callback |
| `readTransport`       | `(void)` | Post/kick an asynchronous read |
| `writeTransport`      | `(uint16 len) → uint16` | Transmit `len` bytes from txBuf |
| `stopTransfer`        | `(void)` | Cancel pending read (flow control only) |
| `handleMrdyEvent`     | `(void)` | Handle MRDY edge (flow control only) |

---

## Proposed Changes

### 1. `npi_tl.h` — add `NPI_USE_MUX` branch

```c
#elif defined(NPI_USE_MUX)
#define transportInit         NPITLMUX_initializeTransport
#define transportRead         NPITLMUX_readTransport
#define transportWrite        NPITLMUX_writeTransport
#define transportStopTransfer NPITLMUX_stopTransfer
#define transportMrdyEvent    NPITLMUX_handleMrdyEvent
```

### 2. `npi_tl.c` — add `NPI_USE_MUX` include

```c
#elif defined(NPI_USE_MUX)
#include "ti/dmm/combined_serial/embedded/npi_tl_mux.h"
```

### 3. New file: `npi_tl_mux.h` (in `combined_serial/embedded/`)

Declares the five transport functions with signatures matching `npi_tl_uart.h`.

### 4. New file: `npi_tl_mux.c` (in `combined_serial/embedded/`)

Implements the five functions:

| Function | Behaviour |
|----------|-----------|
| `initializeTransport` | Save rxBuf/txBuf/callback. Register `rxNotify` with MUX task via `MuxTask_registerRxCb(MUX_NLI_BLE, ...)` |
| `readTransport` | No-op — MUX is push-based; data arrives via `rxNotify` |
| `writeTransport` | Call `MuxTask_sendPacket(MUX_NLI_BLE, txBuf, len)`. Signal TX complete via callback synchronously |
| `stopTransfer` | No-op (NPI_FLOW_CTRL = 0; no UART read to cancel) |
| `handleMrdyEvent` | No-op (no MRDY pin) |
| `rxNotify` *(internal)* | Copy MUX-decoded bytes into npiRxBuf. Call `npiTransmitCB(len, 0)` |

### 5. `ble.cmake` — swap sources, remove shim

- Replace `npi_tl_uart.c` with `npi_tl_mux.c` in `BLE_CONTROLLER_SOURCE_FILES`
- Remove `mux_virt_uart.c` from `BLE_CONTROLLER_SOURCE_FILES`
- Remove the `set_source_files_properties(npi_tl_uart.c ... -include shim.h)` block
- Remove `CONFIG_DISPLAY_UART=CONFIG_UART2_0` (only needed to intercept `UART2_open`)

### 6. `ble_controller.opt`

```
-DNPI_USE_UART   →   -DNPI_USE_MUX
```

### 7. `freertos_main.c`

Remove the explicit `MuxTask_registerRxCb(MUX_NLI_BLE, MuxVirtUart_rxNotify)` call.
Registration moves into `NPITLMUX_initializeTransport()`, called automatically during
`BLEController_main() → NPITask_createTask() → NPITL_initTL()`.

---

## Data Flow Comparison

### Current (shim)

```
NPI layer
  UART2_write(handle, txBuf, len)
    ↓  [preprocessor: -include mux_npi_uart_shim.h]
  MuxVirtUart_npiWrite(handle, txBuf, len)
    ↓
  MuxTask_sendPacket(MUX_NLI_BLE, txBuf, len)
    ↓
  Physical UART TX

Physical UART RX
    ↓
  MuxVirtUart_rxNotify(buf, len)
    ↓  [ring buffer + fake UART handle]
  NPITLUART_readCallBack(fakeHandle, buf, len)
    ↓
  npiTransmitCB(len, 0)
    ↓
  NPI layer
```

### Proposed (NPI_USE_MUX)

```
NPI layer
  transportWrite(len)
    ↓  [macro: transportWrite = NPITLMUX_writeTransport]
  NPITLMUX_writeTransport(len)
    ↓
  MuxTask_sendPacket(MUX_NLI_BLE, txBuf, len)
    ↓
  Physical UART TX

Physical UART RX
    ↓
  NPITLMUX_rxNotify(buf, len)
    ↓  [direct memcpy to npiRxBuf]
  npiTransmitCB(len, 0)
    ↓
  NPI layer
```

---

## Files Changed Summary

| File | Location | Change |
|------|----------|--------|
| `npi_tl.h` | `ti/ble/app_util/npi/` | Add `NPI_USE_MUX` macro branch |
| `npi_tl.c` | `ti/ble/app_util/npi/src/` | Add `NPI_USE_MUX` include branch |
| `npi_tl_mux.h` *(new)* | `ti/dmm/combined_serial/embedded/` | Transport header |
| `npi_tl_mux.c` *(new)* | `ti/dmm/combined_serial/embedded/` | Transport implementation |
| `ble.cmake` | `examples/apps/dmm/rcp_ble_controller/cc23xx_cc27xx/` | Swap sources, remove shim |
| `ble_controller.opt` | `examples/apps/dmm/rcp_ble_controller/cc23xx_cc27xx/` | `NPI_USE_UART` → `NPI_USE_MUX` |
| `freertos_main.c` | `examples/apps/dmm/rcp_ble_controller/cc23xx_cc27xx/` | Remove explicit BLE RX callback registration |

Files **removed from build** (kept on disk for other potential users):
- `mux_virt_uart.c` / `mux_virt_uart.h`
- `mux_npi_uart_shim.h`

---

## Assumptions

- `NPI_FLOW_CTRL = 0` (no MRDY/SRDY pins) — matches current configuration.
- `MuxTask_create()` is called in `main()` before `BLEController_main()`, so
  the MUX RX callback table exists when `NPITLMUX_initializeTransport()` runs.
- `HCI_TL_FULL` is defined (guards `npi_tl_uart.c` content; same guard used in `npi_tl_mux.c`).
