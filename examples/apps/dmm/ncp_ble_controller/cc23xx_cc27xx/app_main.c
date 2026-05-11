/******************************************************************************

 @file  app_main.c

 @brief This file contains the ble controller application for use with the
        Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************

 Copyright (c) 2024-2025, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "app_main.h"
#include "ti/ble/controller/hci/hci_tl.h"
#include "ti/ble/app_util/npi/npi_task.h"
#include "ti/ble/stack_util/icall/app/icall.h"
#include "ti/ble/stack_util/health_toolkit/assert.h"
#ifndef USE_DEFAULT_USER_CFG
#include "ti/ble/app_util/config/ble_user_config.h"
#endif // USE_DEFAULT_USER_CFG
#include "ti/ble/controller/hci/hci_api.h"
#include "ti/ble/stack_util/ble_init.h"
#include <dmm_scheduler.h>
#include <dmm_policy.h>
#include <ti_dmm_application_policy.h>
#include "ti/ble/controller/ll/ll_scheduler.h"
/*******************************************************************************
 * GLOBAL VARIABLES
 */
bleServicesParams_t bleServicesParams;
extern RCL_Client rfClient;

/*******************************************************************************
 * EXTERNS
 */
/*********************************************************************
 * FUNCTION PROTOTYPES
 */
static int BLEController_sendControllerToHost(uint8_t *pHciPkt, uint16_t pktLen);
static void BLEController_sendNpiToController(uint8_t *pNPIPkt);
void BLEController_AssertHandler(uint8 assertCause, uint8 assertSubcause);
void BLEController_main(uint32 syncInitTimeoutTicks);
/*********************************************************************
 * FUNCTIONS
 */
/*******************************************************************************
 * @fn          BLEController_sendControllerToHost
 *
 * @brief       Call back that register in HCI driver.
 *              Events/Data send from Controller and transport to NPI.
 *
 * input parameters
 *
 * @param       uint8_t *pHciPkt packet.
 * @param       uint16_t pktLen packet len.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static int BLEController_sendControllerToHost(uint8_t *pHciPkt, uint16_t pktLen)
{
    hciRawData_t rawHciPkt;

    if( (pHciPkt == NULL) || (pktLen == 0) )
    {
        return 0;
    }

    rawHciPkt.pData = pHciPkt;
    rawHciPkt.pktLen = pktLen;

    /* Send HCI packet over NPI. */
    NPITask_sendToHost((uint8_t *)&rawHciPkt);

    return 0;
}

/*******************************************************************************
 * @fn          BLEController_sendNpiToController
 *
 * @brief       Callback registered into NPI driver.
 *              Commands/Data received from NPI and passed to Controller.
 *
 * input parameters
 *
 * @param       uint8_t *pHciPkt - represents a raw HCI packet
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static void BLEController_sendNpiToController(uint8_t *pHciPkt)
{
  if(pHciPkt != NULL)
  {
    hciRawData_t* pRawHciPkt = (hciRawData_t*)pHciPkt;
    /* Send hci data packet to stack. */
    HCI_HostToControllerSend(pRawHciPkt->pData, pRawHciPkt->pktLen);
  }
}

/*******************************************************************************
 * @fn          BLEController_main
 *
 * @brief       Start ble controller application
 *
 * input parameters
 *
 * @param       syncInitTimeoutTicks - Timeout in Ticks for the BLE services
 *                                     to wait for the initialization (0 - no wait).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void BLEController_main(uint32 syncInitTimeoutTicks)
{
  uint32 status = FAILURE;
  /* Init all required TI-Drivers initialization */
  Board_initGeneral();

  DMMSch_registerClient(&rfClient, DMMPolicy_StackRole_BlePeripheral, DMMPolicy_Id_Ble);

  /* Init BLE Services params structure */
  status = BLE_ServicesParamsInit(&bleServicesParams, sizeof (bleServicesParams_t));

  if (SUCCESS == status)
  {
    /* Register Application callback to trap asserts raised in the Stack */
    bleServicesParams.assertCallback = BLEController_AssertHandler;
    // Async functionality (no wait).
    bleServicesParams.syncInitTimeoutTicks = syncInitTimeoutTicks;
    /* Register call back for event from controller to NPI */
    bleServicesParams.hciCbs.send = BLEController_sendControllerToHost;

    /* Init the BLE services */
    status = BLE_ServicesInit(&bleServicesParams);
  }

  if(SUCCESS == status)
  {
    /* Register call back for command/data from NPI to controller */
    NPITask_registerIncomingRXEventAppCB(BLEController_sendNpiToController,
                                           INTERCEPT);

    /* Start NPI Task */
    NPITask_createTask(ICALL_SERVICE_CLASS_BLE);
  }
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
 *              for use by their own application.  To do this, add assert.c
 *              to your project workspace, the path to assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in assert.h.
 * @param       assertSubcause - Optional assert subcause (see assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void BLEController_AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  // check the assert cause
  switch (assertCause)
  {
    // This assert is raised from the BLE Stack when a malloc failure occurs.
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
    {
      // ERROR: OUT OF MEMORY
      HAL_ASSERT_SPINLOCK;
      break;
    }

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
    {
      // check the subcause
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
        // ERROR: INTERNAL FW ERROR
        HAL_ASSERT_SPINLOCK;
      }
      else
      {
        // ERROR: INTERNAL ERROR
        HAL_ASSERT_SPINLOCK;
      }
      break;
    }

    // An assert originating from an ICall failure.
    case HAL_ASSERT_CAUSE_ICALL_ABORT:
    {
      // ERROR: ICALL ABORT
      HAL_ASSERT_SPINLOCK;
      break;
    }

    default:
    {
     // ERROR: DEFAULT SPINLOCK
      HAL_ASSERT_SPINLOCK;
      break;
    }
  }

  return;
}
