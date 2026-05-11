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
 *   This file implements the FreeRTOS task that runs the ZBOSS NCP firmware.
 *
 *   The SDK's ti_f3_main.c normally runs main_task_function() in a pthread. Here it runs in a
 *   statically allocated FreeRTOS task instead, so no FreeRTOS heap is consumed.
 */

#include "ti_zigbee_task.h"

#include <stddef.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#define ZIGBEE_TASK_STACK_BYTES 6144U
#define ZIGBEE_TASK_STACK_WORDS (ZIGBEE_TASK_STACK_BYTES / sizeof(StackType_t))
#define ZIGBEE_TASK_PRIORITY    5U

/* Defined in the SDK's ti_f3_main.c, which has no header. */
extern void *main_task_function(void *aArg);

static StackType_t  sZigbeeStack[ZIGBEE_TASK_STACK_WORDS];
static StaticTask_t sZigbeeTaskBuffer;

static void zigbeeTask(void *aArg)
{
    (void)main_task_function(aArg);

    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void ZigbeeTask_create(void)
{
    if (xTaskCreateStatic(zigbeeTask, "ZBOSS", ZIGBEE_TASK_STACK_WORDS, NULL, ZIGBEE_TASK_PRIORITY, sZigbeeStack,
                          &sZigbeeTaskBuffer) == NULL)
    {
        for (;;)
        {
        }
    }
}
