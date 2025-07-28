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
 *
 */
#include "Queue_freertos.h"
#include <ti/drivers/dpl/HwiP.h>

/*********************************************************************
 * @fn      Queue_create - OS abstraction Api
 *
 * @brief   Create queue handle.
 * @param   QueueLen The maximum number of items the queue can hold at any one time.
 * @param   msgSize  The size, in bytes, required to hold each item in the queue.
 */
QueueHandle_t Queue_create(uint32_t QueueLen, uint32_t msgSize)
{
    QueueHandle_t queueHandle;
    uintptr_t key;

    key = HwiP_disable();
    queueHandle = xQueueCreate((UBaseType_t)QueueLen, (UBaseType_t)msgSize);
    HwiP_restore(key);

    if (queueHandle == NULL) {
        // error creating Queue
        while(1){};
    }
    return queueHandle;

}

/*********************************************************************
 * @fn      Queue_enqueue - OS abstraction Api
 *
 * @brief   Post an item on a queue.
 * @param   msgQueue The handle to the queue on which the item is to be posted.
 * @param   pMsg     A pointer to the item that is to be placed on the queue.
 */
uint8_t Queue_enqueue(QueueHandle_t msgQueue, char *pMsg)
{
    BaseType_t  status;
    TickType_t  timeout = 0 ; // for NON Blocking Queue
    uintptr_t key;

    key = HwiP_disable();
    status = xQueueSend(msgQueue, pMsg, timeout);
    HwiP_restore(key);

    if (status != pdTRUE) {
        return (FAILURE);
    }
    return (SUCCESS);
}

/*********************************************************************
 * @fn      Queue_dequeue - OS abstraction Api
 *
 * @brief   Post an item on a queue.
 * @param   msgQueue The handle to the queue from which the item is to be received.
 * @param   pMsg     Pointer to the buffer into which the received item will be copied.
 */
// maybe should be named Queue_get ??
uint8_t* Queue_dequeue(QueueHandle_t msgQueue)
{
    uint8_t *pMsg;
    BaseType_t  status;
    TickType_t  timeout = 0; // non blocking
    uintptr_t key;

    key = HwiP_disable();
    status = xQueueReceive(msgQueue, (char*)&pMsg, timeout);
    HwiP_restore(key);

    if (status != pdTRUE) {
        pMsg = NULL;
        //return FAILURE;
    }
    //return SUCCESS;
    return pMsg;
}

/*********************************************************************
 * @fn      Queue_empty - OS abstraction Api
 *
 * @brief   recieve an item from Queue and check if empty.
 *          returns true if Queue is empte.
 * @param   msgQueue The handle to the queue from which the item is to be received.
 * @param   pMsg     Pointer to the buffer into which the received item will be copied.
 *
 */
uint8_t Queue_empty(QueueHandle_t msgQueue)
{
    uintptr_t key;
    uint8_t msgWaiting;

    key = HwiP_disable();
    msgWaiting = uxQueueMessagesWaiting(msgQueue);
    HwiP_restore(key);

    if ( msgWaiting == 0 )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

