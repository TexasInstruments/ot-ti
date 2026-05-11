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
#if !defined(DeviceFamily_CC23X0R5)
#include <ti/drivers/SHA2.h>
#include <ti/drivers/ECDSA.h>
#endif

#if !defined(DeviceFamily_CC23X0R5) && !defined (DeviceFamily_CC27XXX10) && !defined(DeviceFamily_CC27XXX20)
#include <ti/drivers/ECDH.h>
#include <ti/drivers/ECJPAKE.h>
#endif
#include <app_main.h>

#include <dmm_scheduler.h>
#include <dmm_policy.h>
#include <ti_dmm_application_policy.h>
#include <dmm_priority_ble_thread.h>

#ifdef USE_COMBINED_SERIAL
#include "ti_drivers_config.h"   /* CONFIG_UART2_0 */
#include "thread_mux.h"
#include "ti/dmm/combined_serial/embedded/mux_task_app.h"
#include "ti/dmm/combined_serial/embedded/mux_virt_uart.h"
#endif
// The entry point for the application
extern int app_main(int argc, char *argv[]);
#define APP_STACK_SIZE (2048)

StackType_t  appStack[APP_STACK_SIZE];
StaticTask_t appTaskBuffer;

void vTaskCode(void *pvParameters)
{
    (void)pvParameters;

    /* Update to balanced policy */
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_ThreadFtd, DMMPOLICY_BALANCED_BLE_THREAD);
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BALANCED_BLE_THREAD);

    app_main(0, NULL);
}

int main(void)
{
    DMMPolicy_Params dmmPolicyParams;
    DMMPolicy_Status policyRet;

    Board_init();

    GPIO_init();

    NVS_init();
#if !defined(DeviceFamily_CC23X0R5)
    ECDSA_init();

    SHA2_init();
#endif
    AESECB_init();

#if !defined(DeviceFamily_CC23X0R5) && !defined (DeviceFamily_CC27XXX10) && !defined(DeviceFamily_CC27XXX20)
    ECDH_init();

    ECJPAKE_init();
#endif

    if (NULL ==
        xTaskCreateStatic(vTaskCode, "APP", APP_STACK_SIZE, NULL, 7, appStack, &appTaskBuffer))
    {
        while (1)
            ;
    }

    /* Initialize RCL */
    DMMSch_RCL_init();

    /* Initial Policy Manager for tests that require priority handling */
    DMMPolicy_init();
    DMMPolicy_Params_init(&dmmPolicyParams);
    dmmPolicyParams.numPolicyTableEntries = DMMPolicy_ApplicationPolicySize;
    dmmPolicyParams.policyTable = DMMPolicy_ApplicationPolicyTable;
    dmmPolicyParams.globalPriorityTable = globalPriorityTable_bleLthreadH;
    policyRet = DMMPolicy_open(&dmmPolicyParams);

    /* Initialize DMM scheduler */
    DMMSch_init();

    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_ThreadFtd, DMMPOLICY_THREAD_IDLE);
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BLE_IDLE);

#ifdef USE_COMBINED_SERIAL
    /* Create MUX task and open the single physical UART first.
     * MuxTask_create() zeroes gMuxTask (including rxCbs[]), so callbacks
     * MUST be registered AFTER this call, not before. */
    if (MuxTask_create(CONFIG_UART2_0, 921600) != MUX_SUCCESS)
    {
        while (1)
            ;
    }

    /* Register Thread NLI_OT callback — delivers decoded spinel frames to OT. */
    ThreadMux_init();

    /* Register BLE NLI_BLE callback — delivers decoded HCI bytes to NPI
     * virtual UART ring buffer, which triggers the NPI read callback. */
    MuxTask_registerRxCb(MUX_NLI_BLE, MuxVirtUart_rxNotify);
#endif /* USE_COMBINED_SERIAL */

    BLEController_main(0);

    vTaskStartScheduler();

    // Should never get here.
    while (1)
        ;
}
