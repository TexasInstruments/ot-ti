/*
 *  Copyright (c) 2017, Texas Instruments Incorporated
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

#include <openthread/config.h>

#include <stddef.h>

#include <utils/code_utils.h>
#include <utils/uart.h>

#include <ti/drivers/dpl/SemaphoreP.h>

#include "ti_drivers_config.h"
#include <ti/drivers/UART2.h>
#include <ti/drivers/dpl/HwiP.h>

#include "system.h"
#include <string.h>
#include <mqueue.h>

/* Ensure all bytes are written in blocking mode before notifying the stack it
 * can send more data. Less efficient than callback mode. Necessary for certain
 * versions of the UART2 driver.
 */
#define TI_PLAT_UART_BLOCKING 1

#define PLATFORM_UART_EVENT_TX_DONE (1U << 0)
#define PLATFORM_UART_EVENT_RX_DONE (1U << 1)

#define PLATFORM_UART_RECV_BUF_LEN 256
#define PLATFORM_UART_RECV_MQUEUE_LEN 6

static uint8_t PlatformUart_receiveBuffer[PLATFORM_UART_RECV_BUF_LEN];

static UART2_Handle PlatformUart_uartHandle;

#if !TI_PLAT_UART_BLOCKING
static SemaphoreP_Struct PlatformUart_writeSem;
static SemaphoreP_Handle PlatformUart_writeSemHandle;
#endif

/* Local Message Queue for Incoming Data */
const  char  UART_procQueueName[] = "/uartrx_process";
static mqd_t UART_procQueueDesc;
struct UART_procQueueMsg {
    uint8_t rxBuffer[PLATFORM_UART_RECV_BUF_LEN];
    uint8_t readLen;
};

static void uartReadCallback(UART2_Handle aHandle, void *aBuf, size_t aLen, void *userArg, int_fast16_t status)
{
    (void)aHandle;
    (void)aBuf;
    (void)userArg;
    (void)status;
    struct UART_procQueueMsg rxMsg;

    memcpy(rxMsg.rxBuffer, PlatformUart_receiveBuffer, aLen);
    rxMsg.readLen = aLen;

    /* Process incoming data in task context */
    mq_send(UART_procQueueDesc, (char *)&rxMsg, sizeof(struct UART_procQueueMsg), 0);
    platformUartSignal(PLATFORM_UART_EVENT_RX_DONE);

    UART2_read(PlatformUart_uartHandle, PlatformUart_receiveBuffer, sizeof(PlatformUart_receiveBuffer), NULL);
}

#if !TI_PLAT_UART_BLOCKING
static void uartWriteCallback(UART2_Handle aHandle, void *aBuf, size_t aLen, void *userArg, int_fast16_t status)
{
    (void)aHandle;
    (void)aBuf;
    (void)aLen;
    (void)userArg;

    SemaphoreP_post(PlatformUart_writeSemHandle);
    platformUartSignal(PLATFORM_UART_EVENT_TX_DONE);
}
#endif /* !TI_PLAT_UART_BLOCKING */

otError otPlatUartEnable(void)
{
    UART2_Params params;
    struct mq_attr attr;

#if !TI_PLAT_UART_BLOCKING
    PlatformUart_writeSemHandle = SemaphoreP_constructBinary(&PlatformUart_writeSem, 1U);
#endif

    UART2_Params_init(&params);

    params.readMode       = UART2_Mode_CALLBACK;
    params.readCallback   = uartReadCallback;
    params.readReturnMode = UART2_ReadReturnMode_PARTIAL;
    params.eventMask      = UART2_EVENT_TX_FINISHED;
    params.baudRate       = 115200;
    params.dataLength     = UART2_DataLen_8;
    params.stopBits       = UART2_StopBits_1;
    params.parityType     = UART2_Parity_NONE;

#if TI_PLAT_UART_BLOCKING
    params.writeMode     = UART2_Mode_BLOCKING;
    params.writeCallback = NULL;
#else
    params.writeMode     = UART2_Mode_CALLBACK;
    params.writeCallback = uartWriteCallback;
#endif

    attr.mq_curmsgs = 0;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = PLATFORM_UART_RECV_MQUEUE_LEN;
    attr.mq_msgsize = sizeof(struct UART_procQueueMsg);

    UART_procQueueDesc = mq_open(UART_procQueueName, (O_RDWR | O_NONBLOCK | O_CREAT), 0, &attr);

    PlatformUart_uartHandle = UART2_open(CONFIG_UART2_0, &params);

    UART2_read(PlatformUart_uartHandle, PlatformUart_receiveBuffer, sizeof(PlatformUart_receiveBuffer), NULL);

    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    UART2_close(PlatformUart_uartHandle);

    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    int_fast16_t ret;

    /* Block any incoming Tx requests if one is already in progress */
#if !TI_PLAT_UART_BLOCKING
    SemaphoreP_pend(PlatformUart_writeSemHandle, UINT32_MAX);
#endif
    ret = UART2_write(PlatformUart_uartHandle, aBuf, aBufLength, NULL);

#if TI_PLAT_UART_BLOCKING
    platformUartSignal(PLATFORM_UART_EVENT_TX_DONE);
#endif

    return OT_ERROR_NONE;
}

void platformUartProcess(uintptr_t arg)
{
    if (arg & PLATFORM_UART_EVENT_TX_DONE)
    {
        otPlatUartSendDone();
    }

    if (arg & PLATFORM_UART_EVENT_RX_DONE)
    {
        struct UART_procQueueMsg rxMsg;

        mq_receive(UART_procQueueDesc, (char *)&rxMsg, sizeof(struct UART_procQueueMsg), NULL);
        otPlatUartReceived(rxMsg.rxBuffer, rxMsg.readLen);
    }
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}
