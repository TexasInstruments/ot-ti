/******************************************************************************

 @file  app_main.c

 @brief BLE Controller application for the Zigbee NCP + BLE Controller example.
        Registers the BLE stack with DMM, wires NPI to the Combined Serial MUX.

 Group: WCS, BTS
 Target Device: cc23xx / cc27xx

 ******************************************************************************
 Copyright (c) 2024-2025, Texas Instruments Incorporated
 All rights reserved.
 ******************************************************************************/

#include "app_main.h"
#include "ti/ble/controller/hci/hci_tl.h"
#include "ti/ble/app_util/npi/npi_task.h"
#include "ti/ble/stack_util/icall/app/icall.h"
#include "ti/ble/stack_util/health_toolkit/assert.h"
#ifndef USE_DEFAULT_USER_CFG
#include "ti/ble/app_util/config/ble_user_config.h"
#endif
#include "ti/ble/controller/hci/hci_api.h"
#include "ti/ble/stack_util/ble_init.h"
#include <dmm_scheduler.h>
#include <dmm_policy.h>
#include <ti_dmm_application_policy.h>
#include "ti/ble/controller/ll/ll_scheduler.h"

bleServicesParams_t bleServicesParams;
extern RCL_Client rfClient;

static int  BLEController_sendControllerToHost(uint8_t *pHciPkt, uint16_t pktLen);
static void BLEController_sendNpiToController(uint8_t *pNPIPkt);
void        BLEController_AssertHandler(uint8 assertCause, uint8 assertSubcause);

static int BLEController_sendControllerToHost(uint8_t *pHciPkt, uint16_t pktLen)
{
    hciRawData_t rawHciPkt;
    if ((pHciPkt == NULL) || (pktLen == 0)) { return 0; }
    rawHciPkt.pData  = pHciPkt;
    rawHciPkt.pktLen = pktLen;
    NPITask_sendToHost((uint8_t *)&rawHciPkt);
    return 0;
}

static void BLEController_sendNpiToController(uint8_t *pHciPkt)
{
    if (pHciPkt != NULL)
    {
        hciRawData_t *pRawHciPkt = (hciRawData_t *)pHciPkt;
        HCI_HostToControllerSend(pRawHciPkt->pData, pRawHciPkt->pktLen);
    }
}

void BLEController_main(uint32 syncInitTimeoutTicks)
{
    uint32 status = FAILURE;
    Board_initGeneral();
    DMMSch_registerClient(&rfClient, DMMPolicy_StackRole_BlePeripheral, DMMPolicy_Id_Ble);
    status = BLE_ServicesParamsInit(&bleServicesParams, sizeof(bleServicesParams_t));
    if (SUCCESS == status)
    {
        bleServicesParams.assertCallback          = BLEController_AssertHandler;
        bleServicesParams.syncInitTimeoutTicks    = syncInitTimeoutTicks;
        bleServicesParams.hciCbs.send             = BLEController_sendControllerToHost;
        status = BLE_ServicesInit(&bleServicesParams);
    }
    if (SUCCESS == status)
    {
        NPITask_registerIncomingRXEventAppCB(BLEController_sendNpiToController, INTERCEPT);
        NPITask_createTask(ICALL_SERVICE_CLASS_BLE);
    }
}

void BLEController_AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
    switch (assertCause)
    {
        case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:    HAL_ASSERT_SPINLOCK; break;
        case HAL_ASSERT_CAUSE_INTERNAL_ERROR:   HAL_ASSERT_SPINLOCK; break;
        case HAL_ASSERT_CAUSE_ICALL_ABORT:      HAL_ASSERT_SPINLOCK; break;
        default:                                HAL_ASSERT_SPINLOCK; break;
    }
}
