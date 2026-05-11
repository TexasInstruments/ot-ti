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
 * @brief FreeRTOS entry point for the Zigbee NCP + BLE Controller example.
 *
 * Initialises the CC27XX hardware, DMM, the Combined Serial MUX, the BLE
 * Controller stack, and the ZBOSS Zigbee NCP firmware, then hands control to
 * the FreeRTOS scheduler.
 *
 * Two stacks run simultaneously:
 *   - BLE Controller  (NLI 1): HCI traffic forwarded to/from a host BlueZ via the MUX
 *   - Zigbee NCP      (NLI 2): raw ZBNCP byte stream forwarded to/from the TI Zigbee
 *                               Linux Host SDK via the MUX
 *
 * There is no OpenThread in this example.  Thread-related files
 * (thread_mux.c, main.c, ncp.c) are deliberately absent.
 *
 * FreeRTOS task priorities (configMAX_PRIORITIES = 10):
 *   9  BLE stack / APP policy task
 *   7  MUX TX keepalive / BleRxDlvr / NPI
 *   6  FreeRTOS timer daemon
 *   4  ZBOSS (Zigbee NCP firmware)
 */

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

/* TI Driver headers */
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/AESECB.h>
#if !defined(DeviceFamily_CC23X0R5)
#include <ti/drivers/SHA2.h>
#include <ti/drivers/ECDSA.h>
#endif

/* BLE entry point */
#include <app_main.h>

/* DMM */
#include <dmm_scheduler.h>
#include <dmm_policy.h>
#include <ti_dmm_application_policy.h>
#include <dmm_priority_ble_zigbee.h>

/*
 * ZRCL_client is the RCL client handle used by the Zigbee platform library
 * (defined in the SDK's ti/zigbee/mac/ti_f3_mac.c, which is part of the
 * prebuilt Zigbee libraries).  We register it with DMM here so the scheduler
 * knows about the Zigbee radio client and can arbitrate radio time for it.
 * BLE registers its own rfClient inside BLEController_main() → app_main.c.
 */
extern RCL_Client ZRCL_client;

#ifdef USE_COMBINED_SERIAL
#include "ti_drivers_config.h"       /* CONFIG_UART2_0 */
#include "ti_zigbee_mux.h"
#include "ti_zigbee_task.h"
#include "ti/dmm/combined_serial/embedded/mux_task_app.h"
#include "ti/dmm/combined_serial/embedded/mux_virt_uart.h"
#endif /* USE_COMBINED_SERIAL */

/* Small APP task: transitions DMM to balanced mode once both stacks are started. */
#define APP_STACK_SIZE  256U

static StackType_t  appStack[APP_STACK_SIZE];
static StaticTask_t appTaskBuffer;

static void vTaskCode(void *pvParameters)
{
    (void)pvParameters;

    /* Both stacks are now active; switch from idle to balanced scheduling. */
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BALANCED_BLE_ZIGBEE);
    DMMPolicy_updateApplicationState(DMM_ROLE_ZIGBEE,                   DMMPOLICY_BALANCED_BLE_ZIGBEE);

    /* Task done — suspend forever. Zigbee and BLE each run in their own tasks. */
    vTaskSuspend(NULL);
}

int main(void)
{
    DMMPolicy_Params dmmPolicyParams;

    Board_init();
    GPIO_init();
    NVS_init();
#if !defined(DeviceFamily_CC23X0R5)
    ECDSA_init();
    SHA2_init();
#endif
    AESECB_init();

    /* APP task: sets balanced DMM policy and suspends itself. */
    if (NULL == xTaskCreateStatic(vTaskCode, "APP", APP_STACK_SIZE, NULL, 9U,
                                  appStack, &appTaskBuffer))
    {
        while (1) ;
    }

    /* Initialise RCL (radio controller layer). */
    DMMSch_RCL_init();

    /* Initialise DMM policy and scheduler. */
    DMMPolicy_init();
    DMMPolicy_Params_init(&dmmPolicyParams);
    dmmPolicyParams.numPolicyTableEntries = DMMPolicy_ApplicationPolicySize;
    dmmPolicyParams.policyTable           = DMMPolicy_ApplicationPolicyTable;
    dmmPolicyParams.globalPriorityTable   = globalPriorityTable_bleZigbeeH;
    (void)DMMPolicy_open(&dmmPolicyParams);
    DMMSch_init();

    /*
     * Register the Zigbee RCL client with the DMM scheduler.
     *
     * ZRCL_client is the handle the Zigbee platform library (ti_f3_mac.c) uses
     * for every RCL call.  Registering it here gives the DMM scheduler
     * visibility into Zigbee radio activity so it can arbitrate time between
     * Zigbee and BLE.  The Zigbee NCP libraries wrap RCL through the OVRDE_RCL_*
     * hooks (rcl_stub_dmm.c / rcl_override_dmm.c), which already route through
     * the DMM wrappers — this registration is the missing piece that associates
     * the client handle with the ZigbeeRouter policy role and Id_Zigbee priority.
     *
     * BLE registers its own rfClient inside BLEController_main() below.
     */
    DMMSch_registerClient(&ZRCL_client, DMM_ROLE_ZIGBEE, DMMPolicy_Id_Zigbee);

    /* Set both stacks to idle before the scheduler starts. */
    DMMPolicy_updateApplicationState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BLE_IDLE);
    DMMPolicy_updateApplicationState(DMM_ROLE_ZIGBEE,                   DMMPOLICY_ZIGBEE_IDLE);

#ifdef USE_COMBINED_SERIAL
    /*
     * Register all MUX RX callbacks BEFORE MuxTask_create().  Bytes decoded
     * from the UART ISR are dispatched immediately — registering after create()
     * would leave a window where inbound frames are silently dropped.
     */

    /* NLI 2: Zigbee NCP raw byte stream → ZBOSS byte-received callback. */
    ZigbeeMux_init();

    /* NLI 1: BLE HCI → NPI virtual UART ring buffer. */
    MuxTask_registerRxCb(MUX_NLI_BLE, MuxVirtUart_rxNotify);

    /* Open the single physical UART and start the MUX TX keepalive task. */
    if (MuxTask_create(CONFIG_UART2_0, 921600) != MUX_SUCCESS)
    {
        while (1) ;
    }
#endif /* USE_COMBINED_SERIAL */

    /* Start BLE Controller stack (registers with DMM, creates NPI task). */
    BLEController_main(0);

    /* Start the ZBOSS Zigbee NCP firmware task. */
    ZigbeeTask_create();

    vTaskStartScheduler();

    /* Should never reach here. */
    while (1) ;
}
