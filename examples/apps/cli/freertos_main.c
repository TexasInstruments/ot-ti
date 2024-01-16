/*
 *  Copyright (c) 2016, Texas Instruments Incorporated
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

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

/* Driver Header files */
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/UART2.h>

#include <ti/drivers/AESECB.h>
#include <ti/drivers/ECDH.h>
#include <ti/drivers/ECDSA.h>
#include <ti/drivers/SHA2.h>

//#include <icall.h>
#include "bleAppTask.h"
#include <portmacro.h>

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
#include <icall.h>

// BLE user defined configuration
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#endif // USE_DEFAULT_USER_CFG

// The entry point for the application
extern int app_main(int argc, char *argv[]);
#define APP_STACK_SIZE (2048)

StackType_t  appStack[APP_STACK_SIZE];
StaticTask_t appTaskBuffer;

#define BLEAPP_TASK_STACK_SIZE (2048)
static TaskHandle_t BLEAPPTaskHandle;
#define BLEAPP_TASK_PRIORITY 4

#include <bget.h>
#define TOTAL_ICALL_HEAP_SIZE (0xf700)


__attribute__((section(".heap"))) uint8_t GlobalHeapZoneBuffer[TOTAL_ICALL_HEAP_SIZE];
uint32_t heapSize = TOTAL_ICALL_HEAP_SIZE;

void vApplicationStackOverflowHook(void)
{
    while (1)
    {
        ;
    }
}

void vTaskCode(void *pvParameters)
{
    (void)pvParameters;
    app_main(0, NULL);
}

int main(void)
{
    Board_init();
    bpool((void *) GlobalHeapZoneBuffer, TOTAL_ICALL_HEAP_SIZE);

    GPIO_init();

    NVS_init();

    ECDH_init();

    ECDSA_init();

    AESECB_init();

    SHA2_init();

    user0Cfg.appServiceInfo->timerTickPeriod     = ICall_getTickPeriod();
    user0Cfg.appServiceInfo->timerMaxMillisecond = ICall_getMaxMSecs();

#if 1
    /* Initialize ICall module */
    ICall_init();

    /* Start tasks of external images */
    ICall_createRemoteTasks();
#endif 

    if (NULL ==
        xTaskCreateStatic(vTaskCode, "APP", APP_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, appStack, &appTaskBuffer))
    {
        while (1)
            ;
    }

    bleAppTask_init();
    vTaskStartScheduler();

    // Should never get here.
    while (1)
        ;
}
