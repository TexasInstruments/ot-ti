/*
 * Copyright (c) 2024, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "Queue_freertos.h"
#include <ti/drivers/dpl/HwiP.h>

QueueHandle_t Queue_create(uint32_t QueueLen, uint32_t msgSize)
{
    QueueHandle_t queueHandle;
    uintptr_t key;

    key         = HwiP_disable();
    queueHandle = xQueueCreate((UBaseType_t)QueueLen, (UBaseType_t)msgSize);
    HwiP_restore(key);

    if (queueHandle == NULL) { while (1) {}; }
    return queueHandle;
}

uint8_t Queue_enqueue(QueueHandle_t msgQueue, char *pMsg)
{
    BaseType_t status;
    uintptr_t  key = HwiP_disable();
    status         = xQueueSend(msgQueue, pMsg, 0);
    HwiP_restore(key);
    return (status != pdTRUE) ? FAILURE : SUCCESS;
}

uint8_t *Queue_dequeue(QueueHandle_t msgQueue)
{
    uint8_t   *pMsg;
    uintptr_t  key = HwiP_disable();
    if (xQueueReceive(msgQueue, (char *)&pMsg, 0) != pdTRUE) { pMsg = NULL; }
    HwiP_restore(key);
    return pMsg;
}

uint8_t Queue_empty(QueueHandle_t msgQueue)
{
    uintptr_t key     = HwiP_disable();
    uint8_t   waiting = uxQueueMessagesWaiting(msgQueue);
    HwiP_restore(key);
    return (waiting == 0) ? TRUE : FALSE;
}
