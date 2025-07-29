/******************************************************************************

 @file  osal_icall_ble.c

 @brief This file contains function that allows user setup tasks

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


/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include <icall.h>
#include "hal_types.h"
#include "hal_mcu.h"
#include "osal.h"
#include "osal_tasks.h"
#include "osal_snv.h"
#include "osal_cbtimer.h"
#ifdef FREERTOS
#include "mqueue.h"
#endif // FREERTOS


/* LL */
#include "ll.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbtimer.h"
#endif /* OSAL_CBTIMER_NUM_TASKS */

/* Application */
#include "hci_tl.h"

#include "ble_user_config.h"
#include "ble_dispatch.h"

#ifdef USE_ICALL
#ifndef CC23X0
#ifdef ICALL_JT
#include "icall_jt.h"
#endif /* ICALL_JT */
#endif //CC23X0

#ifdef ICALL_LITE
#include "icall_lite_translation.h"
#include <ble_dispatch_lite.h>
#endif /* ICALL_LITE */
#endif /* USE_ICALL */

#if defined( DEBUG_SW_TRACE )
#include <inc/hw_ioc.h>
#endif // DEBUG_SW_TRACE

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
#ifndef ICALL_LITE
  HCI_ProcessEvent,                                                 // task 1
#endif //ICALL_LITE
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),           // task 2
#endif /* OSAL_CBTIMER_NUM_TASKS */
#ifdef ICALL_LITE
  ble_dispatch_liteProcess,                                         // task 3
#else /* !ICALL_LITE */
  bleDispatch_ProcessEvent                                          // task 3
#endif /* ICAL_LITE */
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  ICall_EntityID entity;
  ICall_SyncHandle syncHandle;
  uint8 taskID = 0;
  uint8 i;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  /* LL Task */
  LL_Init( taskID++ );

  /* HCI Task */
#ifdef ICALL_LITE
  HCI_Init( 0 );
#else /* !ICALL_LITE */
  HCI_Init( taskID++ );
#endif /* ICALL_LITE */

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

#ifdef ICALL_LITE
  ble_dispatch_liteInit(taskID++);
#else /* !ICALL_LITE */
  /* ICall BLE Dispatcher Task */
  bleDispatch_Init( taskID );
#endif /* ICALL_LITE */

  // ICall enrollment
  /* Enroll the service that this stack represents */
  ICall_enrollService(ICALL_SERVICE_CLASS_BLE, NULL, &entity, &syncHandle);

#ifndef ICALL_LITE
  /* Enroll the obtained dispatcher entity and OSAL task ID of HCI Ext App
   * to OSAL so that OSAL can route the dispatcher message into
   * the appropriate OSAL task.
   */
  osal_enroll_dispatchid(taskID, entity);
#endif /* ICALL_LITE */

  /* Register all other OSAL tasks to use the registered dispatcher entity
   * ID as the source of dispatcher messages, even though the other OSAL
   * tasks didn't register themselves to receive messages from application.
   */
  for (i = 0; i < taskID; i++)
  {
    osal_enroll_senderid(i, entity);
  }
}

/**
 * Main entry function for the stack image
 */
int stack_main( void *arg )
{
  /* User reconfiguration of BLE Controller and Host variables */
#ifdef ICALL_JT
  setBleUserConfig( (icall_userCfg_t *)arg );
#else /* !ICALL_JT */
  setBleUserConfig( (bleUserCfg_t *)arg );
#endif /* ICALL_JT */

  /* Establish OSAL for a stack service that requires accompanying
   * messaging service */
  if (ICall_enrollService(ICALL_SERVICE_CLASS_BLE_MSG,
                          (ICall_ServiceFunc) osal_service_entry,
                          &osal_entity,
                          &osal_syncHandle) != ICALL_ERRNO_SUCCESS)
  {
    /* abort */
    ICall_abort();
  }

#ifdef CC23X0
  if (LL_initRNGNoise() != LL_STATUS_SUCCESS)
  {
	/* abort */
    ICall_abort();
  }
#endif

  // Disable interrupts
  halIntState_t state;
  HAL_ENTER_CRITICAL_SECTION(state);

#if defined(ICALL_LITE) && (!defined(STACK_LIBRARY))
  {
    icall_liteTranslationInit((uint32_t*)bleAPItable);
  }
#endif  /* ICALL_LITE && !STACK_LIBRARY */

#ifdef ICALL_LITE
  {
    osal_set_icall_hook(icall_liteMsgParser);
  }
#endif  /* ICALL_LITE */

#if !defined( NO_OSAL_SNV ) && !defined( USE_FPGA )
  // Initialize NV System
  osal_snv_init( );
#endif // !NO_OSAL_SNV && !USE_FPGA

  // Initialize the operating system
  osal_init_system();

  HAL_EXIT_CRITICAL_SECTION(state);

  osal_start_system(); // No Return from here

  return(0); // Shouldn't get here.
}

/*********************************************************************
*********************************************************************/

