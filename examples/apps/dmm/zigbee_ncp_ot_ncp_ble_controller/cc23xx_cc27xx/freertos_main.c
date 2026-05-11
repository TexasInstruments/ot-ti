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

/*!
 * @file  freertos_main.c
 * @brief FreeRTOS entry point: Zigbee NCP + OpenThread NCP + BLE Controller.
 *
 * Three stacks over one UART (Combined Serial MUX):
 *   NLI 0 — OpenThread NCP FTD (Spinel/HDLC)
 *   NLI 1 — BLE Controller     (HCI)
 *   NLI 2 — Zigbee NCP         (ZBNCP)
 *
 * DMM arbitrates all three stacks (BLE + Thread + Zigbee).
 * OT self-registers with DMM inside otPlatRadioEnable() (radio.c).
 * Zigbee registers via DMMSch_registerClient() below.
 * BLE registers inside BLEController_main().
 */

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/AESECB.h>
#if !defined(DeviceFamily_CC23X0R5)
#include <ti/drivers/SHA2.h>
#include <ti/drivers/ECDSA.h>
#endif
#if !defined(DeviceFamily_CC23X0R5) && !defined(DeviceFamily_CC27XXX10) && !defined(DeviceFamily_CC27XXX20)
#include <ti/drivers/ECDH.h>
#include <ti/drivers/ECJPAKE.h>
#endif

#include <app_main.h>
#include <dmm_scheduler.h>
#include <dmm_policy.h>
#include <ti_dmm_application_policy.h>
#include "dmm_priority_ble_thread_zigbee.h"

/* ZRCL_client — Zigbee platform RCL client (ti_f3_mac.c).
 * OT self-registers in otPlatRadioEnable(). BLE registers in BLEController_main(). */
extern RCL_Client ZRCL_client;

#ifdef USE_COMBINED_SERIAL
#include "ti_drivers_config.h"
#include "thread_mux.h"
#include "ti_zigbee_mux.h"
#include "ti_zigbee_task.h"
#include "ti/dmm/combined_serial/embedded/mux_task_app.h"
#include "ti/dmm/combined_serial/embedded/mux_virt_uart.h"
#endif

extern int app_main(int argc, char *argv[]);

TaskHandle_t otGetAppTaskHandle(void);

#define APP_STACK_SIZE (2048)
StackType_t  appStack[APP_STACK_SIZE];
StaticTask_t appTaskBuffer;
TaskHandle_t appTaskHandle = NULL;

void vTaskCode(void *pvParameters)
{
    (void)pvParameters;
    /* Transition all three stacks to balanced policy */
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_ThreadFtd,    DMMPOLICY_BALANCED_BLE_THREAD_ZIGBEE);
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BALANCED_BLE_THREAD_ZIGBEE);
    DMMPolicy_updateApplicationState(DMM_ROLE_ZIGBEE,                   DMMPOLICY_BALANCED_BLE_THREAD_ZIGBEE);
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
#if !defined(DeviceFamily_CC23X0R5) && !defined(DeviceFamily_CC27XXX10) && !defined(DeviceFamily_CC27XXX20)
    ECDH_init();
    ECJPAKE_init();
#endif

    appTaskHandle = xTaskCreateStatic(vTaskCode, "APP", APP_STACK_SIZE, NULL, 6,
                                      appStack, &appTaskBuffer);
    if (NULL == appTaskHandle)
    {
        while (1) ;
    }

    /* Initialise RCL */
    DMMSch_RCL_init();

    /* Initialise DMM — 3 stacks: BLE + Thread + Zigbee */
    DMMPolicy_init();
    DMMPolicy_Params_init(&dmmPolicyParams);
    dmmPolicyParams.numPolicyTableEntries = DMMPolicy_ApplicationPolicySize;
    dmmPolicyParams.policyTable           = DMMPolicy_ApplicationPolicyTable;
    dmmPolicyParams.globalPriorityTable   = globalPriorityTable_bleThreadZigbeeH;
    policyRet = DMMPolicy_open(&dmmPolicyParams);
    DMMSch_init();

    /* Register Zigbee RCL client. OT self-registers. BLE registers internally. */
    DMMSch_registerClient(&ZRCL_client, DMM_ROLE_ZIGBEE, DMMPolicy_Id_Zigbee);

    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_ThreadFtd,    DMMPOLICY_THREAD_IDLE);
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BLE_IDLE);
    DMMPolicy_updateApplicationState(DMM_ROLE_ZIGBEE,                   DMMPOLICY_ZIGBEE_IDLE);

#ifdef USE_COMBINED_SERIAL
    ThreadMux_init();
    ZigbeeMux_init();
    MuxTask_registerRxCb(MUX_NLI_BLE, MuxVirtUart_rxNotify);
    if (MuxTask_create(CONFIG_UART2_0, 921600) != MUX_SUCCESS)
    {
        while (1) ;
    }
#endif

    BLEController_main(0);
    ZigbeeTask_create();

    vTaskStartScheduler();
    while (1) ;
}

TaskHandle_t otGetAppTaskHandle(void)
{
    return appTaskHandle;
}
