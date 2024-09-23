/*
 *  Copyright (c) 2024, Texas Instruments Incorporated
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
#include <ti/drivers/ECJPAKE.h>
#include <ti/drivers/SHA2.h>
#include <ti/drivers/AESCTRDRBG.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>

/* Include BLE module */
#include <icall.h>
#include "hal_assert.h"
#include "hci_test_app.h"
#include "inc/npi_task.h"
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

/* Include DMM module */
#include <dmm/dmm_policy.h>
#include <dmm/dmm_scheduler.h>

#include "ti_dmm_application_policy.h"
#include <dmm/dmm_priority_ble_thread.h>

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
// BLE user defined configuration
#ifdef ICALL_JT
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#else
bleUserCfg_t user0Cfg = BLE_USER_CFG;
#endif
#endif // USE_DEFAULT_USER_CFG

// The entry point for the application
extern int app_main(int argc, char *argv[]);

// Forward declarations
void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

#define THREAD_APP_STACK_SIZE (2048)
#define THREAD_APP_TASK_PRIORITY (tskIDLE_PRIORITY + 6)
#define BLE_STARTUP_TASK_PRIORITY (tskIDLE_PRIORITY + 3)

// Globals
TaskHandle_t bleHciTaskHandle = NULL;
StackType_t  threadAppStack[THREAD_APP_STACK_SIZE];
StaticTask_t threadAppTaskBuffer;

void vTaskCode(void *pvParameters)
{
    (void)pvParameters;
    app_main(0, NULL);
}

int main(void)
{
    Board_init();

    GPIO_init();

    NVS_init();

    ECDH_init();

    ECDSA_init();

    AESECB_init();

    ECJPAKE_init();

    SHA2_init();

    AESCTRDRBG_init();

    // Register Application callback to trap asserts raised in the Stack
    RegisterAssertCback( AssertHandler );

    Board_initGeneral();

    // enable ICache prefetching
    VIMSConfigure(VIMS_BASE, TRUE, TRUE);

    // enable ICache
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

    // Thread Stack init
    if (NULL ==
        xTaskCreateStatic(vTaskCode, "APP", THREAD_APP_STACK_SIZE, NULL, THREAD_APP_TASK_PRIORITY, threadAppStack, &threadAppTaskBuffer))
    {
        while (1)
            ;
    }

    #ifdef ICALL_JT
    /* Update User Configuration of the stack */
    user0Cfg.appServiceInfo->timerTickPeriod = ICall_getTickPeriod();
    user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();
    #endif

    // DMM Init
    DMMPolicy_Params dmmPolicyParams;
    DMMSch_Params dmmSchedulerParams;

    /* Initialize and open the DMM policy manager */
    DMMPolicy_init();
    DMMPolicy_Params_init(&dmmPolicyParams);
    dmmPolicyParams.numPolicyTableEntries = DMMPolicy_ApplicationPolicySize;
    dmmPolicyParams.policyTable           = DMMPolicy_ApplicationPolicyTable;
    dmmPolicyParams.globalPriorityTable   = globalPriorityTable_bleLthreadH;
    DMMPolicy_open(&dmmPolicyParams);

    /* Initialize and open the DMM scheduler */
    DMMSch_init();
    DMMSch_Params_init(&dmmSchedulerParams);

    // Copy stack roles and index table
    memcpy(dmmSchedulerParams.stackRoles, DMMPolicy_ApplicationPolicyTable.stackRole,
           sizeof(DMMPolicy_StackRole) * DMMPOLICY_NUM_STACKS);
    dmmSchedulerParams.indexTable = DMMPolicy_ApplicationPolicyTable.indexTable;
    DMMSch_open(&dmmSchedulerParams);

    // Create BLE application task
    HCI_TestApp_createTask();

    NPITask_createTask(ICALL_SERVICE_CLASS_BLE);

    vTaskStartScheduler();

    // Should never get here.
    while (1)
        ;
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack Wrapper
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8_t assertCause, uint8_t assertSubcause)
{
    while(1);
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    AssertHandler(HAL_ASSERT_CAUSE_STACK_OVERFLOW_ERROR, 0);
}