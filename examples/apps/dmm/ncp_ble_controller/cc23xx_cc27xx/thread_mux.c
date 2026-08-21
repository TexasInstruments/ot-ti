/*
 *  Copyright (c) 2025, Texas Instruments Incorporated
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file  thread_mux.c
 * @brief Thread (OpenThread) side of the Combined Serial MUX integration.
 *
 * Analogous to zb_mux.c in the SDK combined_serial module.
 *
 * Data flow:
 *
 *   TX (device → host):
 *     otNcpHdlc → otPlatUartSend() → [uart.c USE_COMBINED_SERIAL path]
 *       → MuxTask_sendPacket(MUX_NLI_OT, ...) → HDLC+Spinel encode → UART
 *
 *   RX (host → device):
 *     UART → MUX task decode → ThreadMux_rxNotify()
 *       → platformUartMuxDeliver() → OT POSIX mqueue
 *       → otSysProcessDrivers → platformUartProcess → otPlatUartReceived()
 *       → otNcpHdlcReceive()
 */

#include "thread_mux.h"

#include <stdint.h>

#include "ti/dmm/combined_serial/embedded/mux_task_app.h"
#include "ti/dmm/combined_serial/mux_common.h"

/* Forward declaration — defined in src/uart.c under USE_COMBINED_SERIAL.
 * Avoids pulling system.h (which has OT header dependencies) into the
 * ble-stack-lib build context. */
extern void platformUartMuxDeliver(const uint8_t *buf, uint16_t len);

/*---------------------------------------------------------------------------
 * Internal RX callback — called from MUX task context
 *--------------------------------------------------------------------------*/

/*!
 * @brief MUX RX callback for NLI_OT frames arriving from the Linux host.
 *
 * Called by the MUX task when a decoded NLI_OT packet is available.
 * Forwards the bytes into the OpenThread processing loop via the platform
 * UART delivery mechanism (POSIX mqueue + event signal).
 *
 * @param buf  Decoded payload (HDLC-framed spinel bytes from OTBR).
 * @param len  Payload length in bytes.
 */
static void ThreadMux_rxNotify(const uint8_t *buf, uint16_t len)
{
    platformUartMuxDeliver(buf, len);
}

/*---------------------------------------------------------------------------
 * Public API
 *--------------------------------------------------------------------------*/

void ThreadMux_init(void)
{
    MuxTask_registerRxCb(MUX_NLI_OT, ThreadMux_rxNotify);
}
