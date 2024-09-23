/******************************************************************************

 @file hci_test_app.c

 @brief This file contains the HCI interface application

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2013-2024, Texas Instruments Incorporated
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
#include <FreeRTOS.h>
#include <task.h>

#include <stdarg.h>
#include <Queue_freertos.h>
#include <ti/drivers/UART2.h> //For UART application
#include <ti/drivers/dpl/EventP.h>
#if defined(USE_FPGA) || defined(DEBUG_SW_TRACE)
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include <string.h>

#include <icall.h>
#include "util.h"

/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include <icall_hci_tl.h>

#include "inc/npi_task.h"
#include "inc/npi_ble.h"

#if defined(USE_RCOSC)
#include "rcosc_calibration.h"
#endif // USE_RCOSC


/* Include DMM module */
//#include "ti_dmm_application_policy.h"
#include <dmm/dmm_policy.h>
#include <dmm/dmm_priority_ble_thread.h>
#include <dmm/dmm_scheduler.h>

/*********************************************************************
 * CONSTANTS
 */

// LE Event Lengths
#define HCI_CMD_COMPLETE_EVENT_LEN              3
#define HCI_CMD_VS_COMPLETE_EVENT_LEN           2

// Task configuration
#define HTA_TASK_STACK_SIZE                     1256
#define BLE_HCI_TASK_PRIORITY 1

// Device Sleep Clock Accuracy
#define DEVICE_SCA                              20 // in PPM

#define HTA_ICALL_EVT                         0x00010000  // Event_Id_31
#define HTA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define APP_READ_BDADDR                       0x01
#define HCI_TL_CALLBACK_EVENT                 0x02

#define HTA_ALL_EVENTS                        (HTA_ICALL_EVT | HTA_QUEUE_EVT | \
                                               APP_READ_BDADDR | \
                                               HCI_TL_CALLBACK_EVENT)

#ifdef ICALL_LITE
#define HTA_NO_OPCODE_SENT                      0xFFFFFFFF
#else /* !ICALL_LITE */
#define HTA_NO_OPCODE_SENT                      0xFFFF
#endif /* ICALL_LITE */

// Queue configuration
#define HTA_QUEUE_LEN                           32
#define HTA_MSG_LEN                             sizeof(uint8_t*)
/*********************************************************************
 * TYPEDEFS
 */
// Queue record structure
typedef struct  HCI_TestAPP_QueueRec_t
{
    aeExtAdvRptEvt_t *pData;
    void* callbackFctPtr;
} HCI_TestAPP_QueueRec;


/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle HCI_TestApp_syncEvent;

// Stack build revision
ICall_BuildRevision buildRev;

// Handle for the Callback Event Queue
/*Non blocking queue */
 QueueHandle_t HCI_testApp_Queue;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// task configuration
uint32_t * Task_Handle;
TaskHandle_t htaTask = NULL;

// device address
uint8 myBDADDR[HCI_BDADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };



#ifdef ICALL_LITE
extern uint32 lastAppOpcodeIdxSent;
#else /* !ICALL_LITE */
extern uint16 lastAppOpcodeSent;
#endif /* ICALL_LITE */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void HCI_TestApp_init(void);
static void HCI_TestApp_taskFxn(void *a0);

static uint8 HCI_TestApp_processEvent(uint8 *);

#if ! defined(Display_DISABLE_ALL)
void hciDisplayBDADDR(uint8 *, uint8);
void hciDisplayLCD(void);
#endif // Display_DISABLE_ALL

#ifdef ICALL_LITE
static void HCI_TestApp_processStackMsg(hciPacket_t *pBuf);
static void HCI_TestApp_handleNPIRxInterceptEvent(uint8_t *pMsg);
static void HCI_TestApp_sendToNPI(uint8_t *buf, uint16_t len);
#else /* !ICALL_LITE */
void HCI_TestApp_ProcessNPIMsg(uint8 * msg);
#endif /* ICALL_LITE */

static uint8_t  HCI_TestApp_Queue_ProcessCbkEvent(void);
static uint8_t  HCI_TestApp_postCallbackEvent(void *pData, void* callbackFctPtr);
static void HciTestApp_sendToNPI(uint8_t *buf, uint16_t len);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HCI_TestApp_createTask
 *
 * @brief   Task creation function for the HCI Test App.
 *
 * @param   none
 *
 * @return  none
 */
void HCI_TestApp_createTask(void)
{
  BaseType_t xReturned;

  /* Create the task, storing the handle. */
  xReturned = xTaskCreate(
              HCI_TestApp_taskFxn,                     /* Function that implements the task. */
              "HCI_Test APP",                          /* Text name for the task. */
              HTA_TASK_STACK_SIZE / sizeof(uint32_t),  /* Stack size in words, not bytes. */
              ( void * ) NULL,                         /* Parameter passed into the task. */
              BLE_STACK_TASK_PRIORITY,                       /* Priority at which the task is created. */
              &htaTask );                              /* Used to pass out the created task's handle. */

  if(xReturned == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
  {
    /* Creation of FreeRTOS task failed */
    while(1);
  }
}

/*********************************************************************
 * @fn      HCI_TestApp_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   none
 *
 * @return  none
 */
static void HCI_TestApp_init(void)
{

    // initialize ICall module
    ICall_init();

    // start tasks of external images
    ICall_createRemoteTasks();

    TaskHandle_t bleStackTaskHandle = (TaskHandle_t) (*((TaskHandle_t *) ICall_getRemoteTaskHandle(0)));
    DMMSch_registerClient(bleStackTaskHandle, DMMPolicy_StackRole_BlePeripheral);
  
    vTaskPrioritySet(xTaskGetCurrentTaskHandle(), BLE_HCI_TASK_PRIORITY);

    /* Set the stacks in default states */
    //DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_BLE_IDLE);

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &HCI_TestApp_syncEvent);

#if defined(USE_RCOSC)
  // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
  HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#ifdef ICALL_LITE
  // Intercept NPI RX events.
  NPITask_registerIncomingRXEventAppCB(HCI_TestApp_handleNPIRxInterceptEvent,
                                       INTERCEPT);

  HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) HciTestApp_sendToNPI,
               HCI_TestApp_postCallbackEvent, selfEntity);

  HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG,
                                                     selfEntity));
#else /* !ICALL_LITE */
  NPITask_registerIncomingTXEventAppCB(HCI_TestApp_ProcessNPIMsg,INTERCEPT);
#endif /* ICALL_LITE */

#ifdef USE_FPGA
  #if defined(CC26X2)
    // configure RF Core SMI Data Link
    IOCPortConfigureSet(IOID_20, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_18, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

    // configure RF Core SMI Command Link
    IOCPortConfigureSet(IOID_22, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_21, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

    // configure RF Core tracer IO
    // Maps to SmartRF06EB RF1.10.
    IOCPortConfigureSet(IOID_19, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #else
    // configure RF Core SMI Data Link
    IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

    // configure RF Core SMI Command Link
    IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

    // configure RF Core tracer IO
    // Maps to J9 Header.
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
  #endif
#else // !USE_FPGA
  #if defined(DEBUG_SW_TRACE)
    // configure RF Core tracer IO
    // Note: Pleaes see board files for additional IO mapping information.

    // SmartRF06EB: Maps to RF1.20
    // Note: Must disable LCD Display!
    //IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);

    // SmartRF06EB: Maps to RF2.10
    // No conflict with LCD.
    //IOCPortConfigureSet(IOID_24, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);

    // SmartRF06EB: Maps to RF2.12
    // No conflict with LCD.
    IOCPortConfigureSet(IOID_30, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

#if ! defined(Display_DISABLE_ALL)
  //open LCD
  dispHandle = Display_open(Display_Type_LCD, NULL);

  // get this device's address
#ifdef ICALL_LITE
  EMBEDDED_HOST(HCI_ReadBDADDRCmd)
#else /* !ICALL_LITE */
  HCI_ReadBDADDRCmd();
#endif /* ICALL_LITE */
#endif // ! Display_DISABLE_ALL

  // Get build revision
#ifdef ICALL_LITE
  VOID buildRevision(&buildRev);
#else /* !ICALL_LITE */
  VOID Util_buildRevision(&buildRev);
#endif /* ICALL_LITE */

  // set this device's Sleep Clock Accuracy, if a Peripheral
  if (buildRev.ctrlInfo & ADV_CONN_CFG)
  {
    //HCI_EXT_SetSCACmd(DEVICE_SCA);
  }

  // Create a Tx Queue instance
  HCI_testApp_Queue = Queue_create(HTA_QUEUE_LEN, HTA_MSG_LEN);

  return;
}

/*********************************************************************
 * @fn      HCI_TestApp_taskFxn
 *
 * @brief   Application task entry point for the HCI Test App.
 *
 * @param   none
 *
 * @return  none
 */
static void HCI_TestApp_taskFxn(void *a0)
{
  // Initialize application
  HCI_TestApp_init();

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(500);

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = EventP_pend(HCI_TestApp_syncEvent, HTA_ALL_EVENTS, 0, ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
#ifdef ICALL_LITE
        ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

        if(pEvt->signature != 0xFFFF)
        {
          // Message
          HCI_TestApp_processStackMsg((hciPacket_t *)pMsg);
        }
#endif /* ICALL_LITE */

        // Free the received message.
        ICall_freeMsg(pMsg);
      }
    }

    // send a Read BDADDR to get this devices address
    if (events & APP_READ_BDADDR)
    {
      // get this device's new address and display it
#ifdef ICALL_LITE
      EMBEDDED_HOST(HCI_ReadBDADDRCmd)
#else /* !ICALL_LITE */
      HCI_ReadBDADDRCmd();
#endif /* ICALL_LITE */
    }

    if( events & HCI_TL_CALLBACK_EVENT)
    {
      while (!Queue_empty(HCI_testApp_Queue))
      {
        // Push the pending Async Msg to the host.
        HCI_TestApp_Queue_ProcessCbkEvent();
        }
      }
    // If we still have new messages, we need to trigger eventP_post again, else we may miss an event
    if(!ICall_IsQueueEmpty())
    {
      EventP_post(HCI_TestApp_syncEvent, HCI_TL_CALLBACK_EVENT);
    }
  }
}

#ifdef ICALL_LITE

/*********************************************************************
 * @fn      HCI_TestApp_processStackMsg
 *
 * @brief   Process incoming HCI Event or Data Message.
 *
 * @param   pEvt - pointer to event structure.
 *
 * @return  none
 */
void HCI_TestApp_processStackMsg(hciPacket_t *pBuf)
{
  // This is the only event that is expected.
  if (pBuf->hdr.event == HCI_CTRL_TO_HOST_EVENT)
  {
    uint16_t len = 0;

    // Determine the packet length
    switch(pBuf->pData[0])
    {
      case HCI_EVENT_PACKET:
        len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
        break;

      case HCI_ACL_DATA_PACKET:
        len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
        break;

      default:
        break;
    }

    // See if the application has any interest in this event.
    if (HCI_TestApp_processEvent((uint8 *) pBuf) == FALSE)
    {
      // It did not. Forward to NPI.
      HCI_TestApp_sendToNPI(pBuf->pData, len);
    }

    switch (pBuf->pData[0])
    {
      case HCI_ACL_DATA_PACKET:
      case HCI_SCO_DATA_PACKET:
         BM_free(pBuf->pData);
      default:
        break;

    }
  }
}

/*********************************************************************
 * @fn      HCI_TestApp_sendToNPI
 *
 * @brief   Create an NPI packet and send to NPI to transmit.
 *
 * @param   buf - pointer HCI event or data.
 *
 * @param   len - length of buf in bytes.
 *
 * @return  none
 */
static void HCI_TestApp_sendToNPI(uint8_t *buf, uint16_t len)
{
  npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

  if (pNpiPkt)
  {
    // The event status code is in first byte of payload.
    pNpiPkt->hdr.event = buf[0];
    pNpiPkt->hdr.status = 0xFF;
    pNpiPkt->pktLen = len;
    pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

    memcpy(pNpiPkt->pData, buf, len);

    // Send to NPI
    // Note: there is no need to free this packet.  NPI will do that itself.
    NPITask_sendToHost((uint8_t *)pNpiPkt);
  }
}
#else /* !ICALL_LITE */

void HCI_TestApp_ProcessNPIMsg(uint8 * msg)
{
  ICall_HciExtEvt *pMsg = (ICall_HciExtEvt *)msg;

  switch ( pMsg->hdr.event )
  {
    case HCI_GAP_EVENT_EVENT:
      // TBD - We should not be receiving any GAP events
      ICall_abort();
      break;

    case HCI_ACL_DATA_PACKET:
    case HCI_SCO_DATA_PACKET:
        NPITask_sendToHost((uint8_t *)msg);
      break;

    case HCI_EVENT_PACKET:
    {
      // check if this message was not sent from the App
      if ( !HCI_TestApp_processEvent( (uint8 *)pMsg ) )
      {
        // it wasn't, so pass it on to the transport layer
        NPITask_sendToHost((uint8_t *)pMsg);
      }
      else // free the message!
      {
        ICall_freeMsg(pMsg);
      }
    }
    break;
  }
}
#endif /* ICALL_LITE */

/*******************************************************************************
 * @fn      HCI_TestApp_processEvent
 *
 * @brief   Process HCI Event, which can either be a vendor specific complete
 *          event, or a BLE event.
 *
 *          This routine only handles Read and Set BDADDR for LCD purposes.
 *
 * @param   None.
 *
 * @return  TRUE:  Event command opcode matches the last command opcode.
 *          FALSE: Event command opcode does not match last command opcode.
 */
static uint8 HCI_TestApp_processEvent(uint8 *pMsg)
{
  uint8 opcodesMatch = FALSE;

#if ! defined(Display_DISABLE_ALL)
  // check if not a vendor specific complete event
  if (((ICall_HciCmdCompleteEvtMsg *)pMsg)->evtCode != HCI_VE_EVENT_CODE)
  {
    ICall_HciReadBdaddrEvtMsg *pPkt = (ICall_HciReadBdaddrEvtMsg *)((ICall_HciCmdCompleteEvtMsg *)pMsg)->pData;

    // Check last sent opcode against the one in the received event
#ifdef ICALL_LITE
    opcodesMatch = HCI_TL_compareAppLastOpcodeSent(pPkt->opcode);
#else /* !ICALL_LITE */
    opcodesMatch = (pPkt->opcode == lastAppOpcodeSent);
#endif /* ICALL_LITE */

    // check the Command Opcode
    if ((pPkt->opcode == HCI_READ_BDADDR) && (pPkt->status == HCI_SUCCESS))
    {
      for (uint8 i=0; i<HCI_BDADDR_LEN; i++)
      {
        myBDADDR[i] = pPkt->bdAddr[i];
      }

      hciDisplayLCD();
    }
  }
  else // Vendor Specific Complete Event
  {
    ICall_HciSetBdaddrEvtMsg *pPkt = (ICall_HciSetBdaddrEvtMsg *)((ICall_HciCmdCompleteEvtMsg *)pMsg)->pData;

    // check last sent opcode against the one in the received event
#ifdef ICALL_LITE
    opcodesMatch = HCI_TL_compareAppLastOpcodeSent(pPkt->opcode);
#else /* !ICALL_LITE */
    opcodesMatch = (pPkt->opcode == lastAppOpcodeSent);
#endif /* ICALL_LITE */

    // check the Command Opcode
    if ((pPkt->opcode == HCI_EXT_SET_BDADDR) && (pPkt->status == HCI_SUCCESS))
    {
      Event_post(HCI_TestApp_syncEvent, APP_READ_BDADDR);
    }
  }
#endif // ! Display_DISABLE_ALL

// This test is only needed to intercept AE ext event and replace them by Legacy event.
// In this case, HCI_BLE_ENHANCED_CONNECTION_COMPLETE_EVENT ==> HCI_BLE_CONNECTION_COMPLETE_EVENT
#ifdef LEGACY_CONN_EVENT
  extern uint8_t legacyCmdStatusScan;
  if (legacyCmdStatusScan == HCI_LEGACY_CMD_STATUS_BT4)
  {
    if (((ICall_HciCmdCompleteEvtMsg *)pMsg)->evtCode == HCI_LE_EVENT_CODE)
    {
      // pMsg point to the raw osal message packet.
      hciPacket_t * myMsg = (hciPacket_t *)pMsg;

      // Based on function LL_EnhancedConnectionCompleteCback, index 3 contains the event.
      if (myMsg->pData[3] == HCI_BLE_ENHANCED_CONNECTION_COMPLETE_EVENT)
      {
        // the different field need to be updated;
        myMsg->pData[2] = HCI_CONNECTION_COMPLETE_EVENT_LEN;  // Update Data Length
        myMsg->pData[3] = HCI_BLE_CONNECTION_COMPLETE_EVENT;  // Update the Event
        myMsg->pData[8] &= LL_DEV_ADDR_TYPE_MASK;  // apply the address type mask
        myMsg->pData[15] = myMsg->pData[27];       // LO connInterval
        myMsg->pData[16] = myMsg->pData[28];       // HI connInterval
        myMsg->pData[17] = myMsg->pData[29];       // LO peripheralLatency
        myMsg->pData[18] = myMsg->pData[30];       // HI peripheralLatency
        myMsg->pData[19] = myMsg->pData[31];       // LO connTimeout
        myMsg->pData[20] = myMsg->pData[32];       // HI connTimeout
        myMsg->pData[21] = myMsg->pData[33];       // clockAccuracy
      }
    }
  }
#endif

  // clear last opcode
#ifdef ICALL_LITE
  lastAppOpcodeIdxSent = HTA_NO_OPCODE_SENT;
#else /* !ICALL_LITE */
  lastAppOpcodeSent = HTA_NO_OPCODE_SENT;
#endif /* ICALL_LITE */

  return(opcodesMatch);
}

//*****************************************************************************
// the function prototypes

#if ! defined(Display_DISABLE_ALL)

#define NYBBLE_TO_ASCII(c) ((c)+(((c)<=9)?'0':'7'))

/*******************************************************************************
 * @fn          hciDisplayLCD
 *
 * @brief       For SmartRF05, display company name, LL role, and BDADDR on LCD.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void hciDisplayLCD(void)
{
  uint8 i = 0;

  // header
  Display_print0(dispHandle, i++, 0, "CC2650 - HCITestApp");

  // display BLE Device Address
  hciDisplayBDADDR(myBDADDR, i++);

  // add a line spacer
  i++;

  // code version
#ifdef CC26XX_R2
  Display_print0(dispHandle, i++, 0, "TI LPRF BLE V3.0.0");
#else // !CC26XX_R2
  Display_print0(dispHandle, i++, 0, "TI LPRF BLE V2.2.0");
#endif // CC26XX_R2

  // check that there is a valid stack build configuration
  if (buildRev.ctrlInfo == 0x00)
  {
    Display_print0(dispHandle, i++, 0, "UNKNOWN BUILD CONFIG!");
    return;
  }

  // check for Inititator (i.e. Central)
  if (buildRev.ctrlInfo & INIT_CFG)
  {
    Display_print0(dispHandle, i++, 0, "INIT     (Central)");
  }

  // check for Connectable Advertising (i.e. Peripheral)
  if (buildRev.ctrlInfo & ADV_CONN_CFG)
  {
    Display_print0(dispHandle, i++, 0, "ADV_CONN (Peripheral)");
  }

  // check for Non-Connectable Advertising
  if (buildRev.ctrlInfo & ADV_NCONN_CFG)
  {
    Display_print0(dispHandle, i++, 0, "ADV_NCONN(Broadcast)");
  }

  // check for Scanner
  if (buildRev.ctrlInfo & SCAN_CFG)
  {
    Display_print0(dispHandle, i++, 0, "SCAN     (Observer)");
  }

  return;
}

/*******************************************************************************
 * @fn          hciDisplayBDADDR
 *
 * @brief       For SmartRF05, display BDADDR on LCD.
 *
 * input parameters
 *
 * @param       line - LCD line number to use for BDADDR.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void hciDisplayBDADDR(uint8 *bdAddr, uint8 line)
{
  uint8 i, j;
  uint8 str[(2*HCI_BDADDR_LEN)+2];

  str[0] = '0';
  str[1] = 'x';

  for (i=1, j=2; i<HCI_BDADDR_LEN+1; i++, j+=2)
  {
    // upper nybble
    str[j] = NYBBLE_TO_ASCII(bdAddr[HCI_BDADDR_LEN-i]>>4);

    // lower nybble
    str[j+1] = NYBBLE_TO_ASCII(bdAddr[HCI_BDADDR_LEN-i] & 0x0F);
  }

  Display_print0(dispHandle, line, 0, (char *)str);

  return;
}
#endif // ! Display_DISABLE_ALL

#ifdef ICALL_LITE
/*********************************************************************
 * @fn      HCI_TestApp_handleNPIRxInterceptEvent
 *
 * @brief   Intercept an NPI RX serial message and queue for this application.
 *
 * @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
 *
 * @return  none.
 */
void HCI_TestApp_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
  HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

  // Free the NPI message
  NPITask_freeNpiMsg(pMsg);
}

/*********************************************************************
 * @fn      HCI_TestApp_postCallbackEvent
 *
 * @brief   Post a event to the task so it can be parse out of the Swi context.
 *
 * @param   extAdvRpt - a pointer to the data to parse.
 *          callbackFctPtr - function pointer that will parse the message.
 *
 * @return  status:
 *            true if allocation and post succeed.
 *            false if allocation of event failed.
 */
uint8_t HCI_TestApp_postCallbackEvent(void *pData, void* callbackFctPtr)
{
  HCI_TestAPP_QueueRec *recPtr;

  recPtr = ICall_malloc(sizeof(HCI_TestAPP_QueueRec));
  if (recPtr)
  {
    recPtr->pData = pData;
    recPtr->callbackFctPtr = callbackFctPtr;
    if(Queue_enqueue(HCI_testApp_Queue, (char*)&(recPtr)) == FAILURE)
    {
        ICall_free(recPtr);
        return(false);
    }
    EventP_post(HCI_TestApp_syncEvent, HCI_TL_CALLBACK_EVENT);
    return(true);
  }
  else
  {
    return(false);
  }
}

/*********************************************************************
 * @fn      HCI_TestApp_Queue_ProcessCbkEvent
 *
 * @brief   Process the queue of Callback from HCI transport Layer.
 *
 * @param   None
 *
 * @return  TRUE: The list was not empty.
 *          FALSE: The list was empty.
 */
static uint8_t HCI_TestApp_Queue_ProcessCbkEvent(void)
{
  //Get the Queue elem atomicaly:
  HCI_TestAPP_QueueRec *pRec = (HCI_TestAPP_QueueRec*) Queue_dequeue(HCI_testApp_Queue);

  if (pRec != NULL)
  {
    //List not Empty
    ((void (*)(void*))(pRec->callbackFctPtr))(pRec->pData);
    ICall_free(pRec);
  }
  else
  {
    return(false);
  }
  return(true);
}

/*********************************************************************
 * @fn      HciTestApp_sendToNPI
 *
 * @brief   Create an NPI packet and send to NPI to transmit.
 *
 * @param   buf - pointer HCI event or data.
 *
 * @param   len - length of buf in bytes.
 *
 * @return  none
 */
static void HciTestApp_sendToNPI(uint8_t *buf, uint16_t len)
{
  npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

  if (pNpiPkt)
  {
    pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
    pNpiPkt->hdr.status = 0xFF;
    pNpiPkt->pktLen = len;
    pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

    memcpy(pNpiPkt->pData, buf, len);

    // Send to NPI
    // Note: there is no need to free this packet.  NPI will do that itself.
    NPITask_sendToHost((uint8_t *)pNpiPkt);
  }
}

#endif /* ICALL_LITE */

/*********************************************************************
*********************************************************************/

