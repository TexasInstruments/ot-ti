/*
 *  Copyright (c) 2026, Texas Instruments Incorporated
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

/**
 * @file
 *   This file implements the ZBOSS NCP serial port on top of the Combined Serial MUX (NLI_ZB).
 *
 *   It provides the zb_osif_serial_* functions that the ZBOSS NCP library expects from the platform.
 *   Transmit runs in the calling (ZBOSS) task; receive runs in the UART ISR via the MUX callback.
 *   The SDK's ti_f3_serial.c defines the same symbols and must not be linked with this file.
 */

#include "ti_zigbee_mux.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ti/dmm/combined_serial/embedded/mux_task_app.h"
#include "ti/dmm/combined_serial/mux_common.h"

#define ZB_TRACE_FILE_ID 30004
#include "zb_common.h"

static volatile bool          sPortOpen;
static volatile zb_callback_t sByteReceivedCb;

/* Called from the UART ISR for every decoded NLI_ZB frame. */
static void zigbeeMuxRxNotify(const uint8_t *aBuf, uint16_t aLen)
{
    zb_callback_t callback = sByteReceivedCb;

    if (callback != NULL)
    {
        for (uint16_t i = 0U; i < aLen; i++)
        {
            callback(aBuf[i]);
        }
    }
}

void ZigbeeMux_init(void)
{
    MuxTask_registerRxCb(MUX_NLI_ZB, zigbeeMuxRxNotify);
}

void zb_osif_serial_init(void)
{
    sPortOpen = true;
}

void zb_osif_serial_deinit(void)
{
    sPortOpen = false;
}

void zb_osif_set_uart_byte_received_cb(zb_callback_t aCallback)
{
    sByteReceivedCb = aCallback;
}

void zb_osif_set_user_io_buffer(zb_byte_array_t *aBuf, zb_ushort_t aCapacity)
{
    (void)aBuf;
    (void)aCapacity;
}

void zb_osif_serial_put_bytes(const zb_uint8_t *aBuf, zb_short_t aLen)
{
    /* zb_short_t is a signed 8-bit type, so a write always fits in one MUX frame. */
    if (sPortOpen && (aBuf != NULL) && (aLen > 0))
    {
        (void)MuxTask_sendPacket(MUX_NLI_ZB, aBuf, (uint16_t)aLen);
    }
}
