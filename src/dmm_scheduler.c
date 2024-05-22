/******************************************************************************

 @file dmm_scheduler.c

 @brief Dual Mode Manager Scheduler

 Group: WCS LPC
 Target Device: cc13xx_cc26xx

 ******************************************************************************

 Copyright (c) 2019-2023, Texas Instruments Incorporated

 All rights reserved not granted herein.
 Limited License.

 Texas Instruments Incorporated grants a world-wide, royalty-free,
 non-exclusive license under copyrights and patents it now or hereafter
 owns or controls to make, have made, use, import, offer to sell and sell
 ("Utilize") this software subject to the terms herein. With respect to the
 foregoing patent license, such license is granted solely to the extent that
 any such patent is necessary to Utilize the software alone. The patent
 license shall not apply to any combinations which include this software,
 other than combinations with devices manufactured by or for TI ("TI
 Devices"). No hardware patent is licensed hereunder.

 Redistributions must preserve existing copyright notices and reproduce
 this license (including the above copyright notice and the disclaimer and
 (if applicable) source code license limitations below) in the documentation
 and/or other materials provided with the distribution.

 Redistribution and use in binary form, without modification, are permitted
 provided that the following conditions are met:

   * No reverse engineering, decompilation, or disassembly of this software
     is permitted with respect to any software provided in binary form.
   * Any redistribution and use are licensed by TI for use only with TI Devices.
   * Nothing shall obligate TI to provide you with source code for the software
     licensed and provided to you in object code.

 If software source code is provided to you, modification and redistribution
 of the source code are permitted provided that the following conditions are
 met:

   * Any redistribution and use of the source code, including any resulting
     derivative works, are licensed by TI for use only with TI Devices.
   * Any redistribution and use of any object code compiled from the source
     code and any resulting derivative works, are licensed by TI for use
     only with TI Devices.

 Neither the name of Texas Instruments Incorporated nor the names of its
 suppliers may be used to endorse or promote products derived from this
 software without specific prior written permission.

 DISCLAIMER.

 THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

/***** Includes *****/

#ifndef FREERTOS
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Error.h>
#else
#include <FreeRTOS.h>
#include <task.h>
#endif


#include <ti/dmm/dmm_scheduler.h>
#include <ti/dmm/dmm_policy.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SwiP.h>
#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_ieee_coex.h)
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_ieee_cmd.h)

/***** Defines *****/

#define MAX_DMM_CLIENTS 2

#define RF_GET_SCHEDULE_MAP_CURR_CMD_IDX    RF_SCH_MAP_CURRENT_CMD_OFFSET
#define RF_GET_SCHEDULE_MAP_PENDING_CMD_IDX (RF_SCH_MAP_PENDING_CMD_OFFSET)

#define MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US 3000
#define MIN_TIME_FOR_BLOCK_MODE_US               1000

#define DMM_DEFAULT_PHY_SWITCHING_MARGIN        314  //This should match the define in the RF driver
#define DMM_CONFICT_HOOK_DELAY                  50
#define STACK_RF_CB_LATENCY                     500  // Latency of the stack callback

#define RF_NUM_RAT_TICKS_IN_1_US               4
/* (3/4)th of a full RAT cycle, in us */
#define RF_DISPATCH_MAX_TIME_US                (UINT32_MAX / RF_NUM_RAT_TICKS_IN_1_US * 3 / 4)
/* (1/4)th of a full RAT cycle, in us */
#define RF_DISPATCH_MAX_TIME_WRAPAROUND_US     (int32_t)(RF_DISPATCH_MAX_TIME_US - UINT32_MAX / RF_NUM_RAT_TICKS_IN_1_US)

#define RadioTime_To_ms(radioTime) (radioTime / (4000000/1000))
#define ms_To_RadioTime(ms)        (ms * (4000000/1000))

/* full RAT cycle (~17.8 min) */
#define MAX_RAT_TIME_US                         UINT32_MAX / RF_NUM_RAT_TICKS_IN_1_US
/* Schedule limit, (3/4)th of a full RAT cycle = ~13.4 min */
#define SCHEDULE_LIMIT_US                       (MAX_RAT_TIME_US * 3/4)
/* Schedule limit, (3/4)th of a full RAT cycle = ~13.4 min */
#define WRAPAROUND_LIMIT_US                     (MAX_RAT_TIME_US * 1/4)

/* //! \brief Option to enable RX Resume in DMM scheduler
 * defined: enable DMM scheduler Rx Resume
 * not defined: Use the 15.4 stack Rx resume
  */
#define xDMM_RX_RESUME                   ///< Automatically resume RX commands that were preempted by DMM


/***** Type declarations *****/
typedef struct {
#ifndef FREERTOS
    Task_Handle* pTaskHndl;
#else
    TaskHandle_t pTaskHndl;
#endif
    DMMPolicy_StackRole stackRole;
    RF_Handle clientHndl;
    RF_ClientCallback rfClientCb;
    bool isBlocked;
} DMMSch_Client_t;

#ifdef DMM_RX_RESUME
typedef struct {
    RF_Handle h;
    RF_CmdHandle cmdHndl;
    RF_Callback cb;
    RF_EventMask bmEvent;
    RF_ScheduleCmdParams SchParamsDmm;
    RF_Op* pOp;
    bool cmdResubmitted;
    List_List* pPendQ;
} RxResumeCmd_t;
#endif

/***** Variable declarations *****/

static DMMSch_Client_t dmmClients[MAX_DMM_CLIENTS] = {0};
static uint32_t numDmmClients = 0;

//Mutex for locking the RF driver resource
static SemaphoreP_Handle DmmSchedulerMutex;


#ifdef DMM_RX_RESUME
static RxResumeCmd_t rxResumeCmd;
static void rxResumeCmdCb(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
#endif

static bool dmmInitialized = false;

static DMMSch_PreemptionCb preemptionCb;
static uint8_t preemptCnt = 0;

/***** Private function definitions *****/
#if !CONFLICT_FLUSH_ALL
static RF_ScheduleStatus dmmSinglePreemptCheck(RF_Cmd* pCmdBg, RF_Cmd* pCmdNew, List_List* pPendQueue);
static DMM_ConflictStatus dmmCmdConflictCheck(RF_Cmd* prevCmd, RF_Cmd* newCmd, RF_Cmd* nextCmd);
static bool conflictCheckWithNextCmd(RF_Cmd* pNewCmd, RF_Cmd* pNextCmd);
static bool conflictCheckWithPrevCmd(RF_Cmd* pPrevCmd, RF_Cmd* pNewCmd);
#endif
static DMMSch_Client_t* getClient(void);
static DMMPolicy_StackRole getClientStackRole(RF_Handle clientHndl);
static int32_t getSwitchingTimeInUs(RF_Cmd* prevCmd, RF_Cmd* nextCmd);
static uint32_t getAbsTimeGap (uint32_t time1, uint32_t time2);
static bool cmdInsertCheckAfterPrevCmd(RF_Cmd* pPrevCmd, RF_Cmd* pNewCmd);
static bool cmdInsertCheckBeforeNextCmd(RF_Cmd* pNewCmd, RF_Cmd* pNextCmd);
static bool dmmCmdInsertCheck(RF_Cmd* prevCmd, RF_Cmd* newCmd, RF_Cmd* nextCmd);
static RF_ScheduleStatus dmmScheduleCmd(RF_Cmd* pCmdBg, RF_Cmd* pCmdNew, List_List* pPendQueue);
static RF_Cmd* dmmCheckPreemption(RF_Handle h2Handle, RF_Cmd* pCmdNew, RF_Cmd* pCmdBg, List_List* pPendQueue, bool* isQEmpty);
static RF_Stat dmmPreemptCmd(RF_Cmd* pCmdPreempt);
static bool isClientMatched(RF_Handle h, RF_Cmd* pCmd);
static bool isNewCmdHigherPriority(RF_Cmd* pCmdNew, RF_Cmd* pCompareCmd);
static void dmmAdjustCmdSettings(RF_Cmd* pCmd );
static bool isTimeInOrder (uint32_t earlyTime, uint32_t laterTime);
static void dmmAdjustCmdSettings(RF_Cmd* pCmd);
static bool isSameClient(RF_Cmd* pCmdNew, RF_Cmd* pCompareCmd);
static bool isBlockModeOn (RF_Cmd* pCmd);

RF_ExecuteAction   DMM_resolveConflictCb(RF_Cmd* pCmdBg, RF_Cmd* pCmdFg, List_List* pPendQueue, List_List* pDoneQueue, bool bConflict, RF_Cmd* pConflictCmd);
RF_ScheduleStatus  DMM_commandScheduleCb (RF_Cmd* pCmdNew, RF_Cmd* pCmdBg, RF_Cmd* pCmdFg, List_List* pPendQueue, List_List* pDoneQueue);

RFCC26XX_SchedulerPolicy RFCC26XX_schedulerPolicy =
{
 .submitHook   = DMM_commandScheduleCb,
 .executeHook = DMM_resolveConflictCb
};

#ifdef DMM_RX_RESUME
/*
 *  Only called within the wrapper function to run scheduleCmd within the callback workaround.
 */
/*
 *  This is the DMM level callback function wrapper that is passed to RF_scheduleCmd.
 *      This function wraps the regular client callback function passed from app/stack.
 *      The regular client callback function was saved in rxResumeCmd in schedule time.
 *      After the rf command is finished, this wrapper callback is called. Any DMM
 *      specific actions can be done here. After that, this function looks up and
 *      call the regular client callback function.
 *
 *  Input:  h                    - RF_Handle
 *          ch                   - Command handle
 *          e                    - event mask
 *
 *  Return: void
 */
static void rxResumeCmdCb(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    DMM_dbgLog4("rxResumeCmdCb: c=%x, ch=%x, e=0x%x-%x", h, ch, e >> 32, e & 0xFFFFFFFF);

    if(rxResumeCmd.cmdResubmitted == false)
    {
        if (rxResumeCmd.cmdHndl == ch)
        {
            if(e & RF_EventCmdPreempted)
            {
                bool IsRxInPendQ = false;

               /* check if a Rx command is in the Pend Q, if so, do not resume this Rx */
                RF_Cmd* pTemp = (RF_Cmd*)List_head(rxResumeCmd.pPendQ);

                while(pTemp != NULL)
                {
                    if((rxResumeCmd.pOp->commandNo == pTemp->pOp->commandNo)&&(pTemp->pCb != rxResumeCmdCb ))
                    {
                        /* A Rx command has been scheduled by the stack, so DMM should not resume this Rx command.
                         * call the client callback so that the stack makes a decision*/
                        IsRxInPendQ = true;
                        break;
                    }
                    pTemp = (RF_Cmd*)List_next((List_Elem*)pTemp);
                }

                if(IsRxInPendQ)
                {
                    /* call client callback */
                    if (rxResumeCmd.cb)
                    {
                        /* call client command callback */
                        rxResumeCmd.cb(h, ch, e);
                    }
                    /* invalidate RxResume Command */
                    rxResumeCmd.cmdHndl = RF_SCHEDULE_CMD_ERROR;
                    rxResumeCmd.cmdResubmitted = false;
                }
                else
                {
                    rfc_dataEntryGeneral_t* pDataEntry = NULL;
                    switch(rxResumeCmd.pOp->commandNo)
                    {
                        case (CMD_PROP_RX):
                            pDataEntry = (rfc_dataEntryGeneral_t*) (((rfc_CMD_PROP_RX_t*)(rxResumeCmd).pOp)->pQueue->pCurrEntry);
                            break;
                        case (CMD_PROP_RX_ADV):
                            pDataEntry = (rfc_dataEntryGeneral_t*) (((rfc_CMD_PROP_RX_ADV_t*)(rxResumeCmd).pOp)->pQueue->pCurrEntry);
                            break;
                        case (CMD_IEEE_RX):
                            pDataEntry = (rfc_dataEntryGeneral_t*) (((rfc_CMD_IEEE_RX_t*)(rxResumeCmd).pOp)->pRxQ->pCurrEntry);
                            break;
                        default:
                            break;
                    }
                    if( pDataEntry )
                    {
                        pDataEntry->status = DATA_ENTRY_PENDING;
                    }
                    /*  re-submit   */
                    RF_scheduleCmd(rxResumeCmd.h, rxResumeCmd.pOp, &rxResumeCmd.SchParamsDmm, rxResumeCmdCb, rxResumeCmd.bmEvent);
                    rxResumeCmd.cmdResubmitted = true;
                }
            }
            else if( (e & RF_EventLastCmdDone) || (e & RF_EventCmdAborted)  || (e & RF_EventCmdStopped) || (e & RF_EventCmdCancelled) )
            {
                /* The Rx command has been finished, so no need to reschedule it */
                if (rxResumeCmd.cb)
                {
                    /* call clients command callback */
                    rxResumeCmd.cb(h, ch, e);
                }
                /* invalidate RxResume Command */
                rxResumeCmd.cmdHndl = RF_SCHEDULE_CMD_ERROR;
                rxResumeCmd.cmdResubmitted = false;
            }
        }
    }
    else if (rxResumeCmd.cmdHndl == ch)
    {
        /* Note that the Rx command has been submitted */
        if (rxResumeCmd.cb)
        {
            /* call clients command callback */
            rxResumeCmd.cb(h, ch, e);
        }
        /* we may have more client callback before the command end's so only free the resumeCmd structure when it is ended */
        if( (e & RF_EventLastCmdDone) || (e & RF_EventCmdAborted)  || (e & RF_EventCmdStopped) || (e & RF_EventCmdCancelled)|| (e & RF_EventCmdPreempted))
        {
            /* invalidate RxResume Command */
            rxResumeCmd.cmdHndl = RF_SCHEDULE_CMD_ERROR;
            rxResumeCmd.cmdResubmitted = false;
        }
    }
}
#endif //#ifdef DMM_RX_RESUME
/*
 *  Find which client is running in the current task
 *
 *  Input:  void
 *
 *  Return: DMMSch_Client_t      - DMM client pointer
 *
 */

static DMMSch_Client_t* getClient(void)
{
    uint32_t clientIdx;
    /* map task handle to rfHandle returned */
    for(clientIdx = (uint32_t)0; clientIdx < MAX_DMM_CLIENTS; clientIdx++)
    {
        /* If the clients task equals this task and there is no
           * client handle registered
           */
#ifndef FREERTOS
        Task_Handle currentTask = Task_self();
        /* If the clients task equals this task and there is no
         * client handle registered
         */
        if (*(dmmClients[clientIdx].pTaskHndl) == currentTask)
        {
            return &(dmmClients[clientIdx]);
        }

#else
        TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();

        if (dmmClients[clientIdx].pTaskHndl == currentTask)
        {
            return &(dmmClients[clientIdx]);
        }
#endif

    }

    return NULL;
}

/*
 *  Find the given client stack role
 *
 *  Input:  clientHndl           - RF_Handle
 *
 *  Return: DMMPolicy_StackRole  - DMM stack role
 *
 */
static DMMPolicy_StackRole getClientStackRole(RF_Handle clientHndl)
{
    uint32_t clientIdx;

    /* map task handle to rfHandle returned */
    for(clientIdx = (uint32_t)0; clientIdx < MAX_DMM_CLIENTS; clientIdx++)
    {
        /* If the clients task equals this ask and there is no
         * client handle registered
         */
        if (dmmClients[clientIdx].clientHndl == clientHndl)

        {
            return dmmClients[clientIdx].stackRole;
        }
    }

    return DMMPolicy_StackRole_invalid;
}

static void dmmAdjustCmdSettings(RF_Cmd* pCmd)
{
#ifdef DMM_DEBUG_LOGGING
    pCmd->pOp->startTrigger.pastTrig = true;
#endif

    if(DMMPolicy_getGPTStatus())
    {
        /* GPT is available */
        if((pCmd->allowDelay ==RF_AllowDelayNone)||(pCmd->allowDelay ==RF_AllowDelayAny))
        {
            pCmd->allowDelay =0;
        }
    }
    else
    {
        /* GPT is not available. Legacy policy table */
        if(DMMPOLICY_TIME_CRITICAL == DMMPolicy_getTimeConstraintValue(pCmd->pClient->clientConfig.nID))
        {
            pCmd->allowDelay =0;
        }
        else if(pCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            /* force to assign enough allowDelay even for the ABS commands
             * this won't work for ABS command that master and device need to sync-up
             * User should be aware of this*/
            pCmd->allowDelay = SCHEDULE_LIMIT_US*RF_NUM_RAT_TICKS_IN_1_US;
        }
    }

    if (pCmd->pOp->commandNo == CMD_BLE5_SLAVE)
    {
        ((rfc_CMD_BLE5_SLAVE_t*)(pCmd->pOp))->pParams->endTrigger.pastTrig = (uint8_t)1;
        ((rfc_CMD_BLE5_SLAVE_t*)(pCmd->pOp))->pParams->timeoutTrigger.pastTrig = (uint8_t)1;
    }
}


/*
 *  Check if the requesting command can be scheduled by preempting the running command
 *  or any of the commands in the queue belonging to the other client.
 *
 *  Input:  h2Handle                - other client RF_Handle to be checked against
 *          pCmdNew                 - requesting command
 *          pCmdBg                  - Bg command
 *          pPendQueue              - Pend Queue
 *
 *  Return: RF_Cmd*                 - RF_Cmd pointer to the first command to be preempted.
 *                                     Return NULL pointer if there is no commands from the other client running/queued.
 *                                     isQEmpty = true when no command with h2 in the queue
 *                                     isQEmpty = false when there is a command with h2 in the queue
 */
static RF_Cmd* dmmCheckPreemption(RF_Handle h2Handle, RF_Cmd* pCmdNew, RF_Cmd* pCmdBg, List_List* pPendQueue, bool* isQEmpty)
{

   /* use pPendQueue */
     bool canBePreempted = false;
     RF_Cmd* pPreemptCmd = NULL;
     RF_Cmd* pRefCmd = (RF_Cmd*)List_head(pPendQueue);
     uint8_t cntH2ClientHigherthanNewCmd = 0;

    /* As default, no commands belonging to h2 is found in the queue */
    bool noH2Cmd = true;

    /* Check if we can preempt the Bg command */
    if (isClientMatched(h2Handle, pCmdBg))
    {
        noH2Cmd = false;
        canBePreempted = isNewCmdHigherPriority(pCmdNew, pCmdBg);
        if (canBePreempted)
        {
            pPreemptCmd = pCmdBg;
        }
    }

    while (pRefCmd)
    {
        if (isClientMatched(h2Handle, pRefCmd))
        {
            noH2Cmd = false;
            canBePreempted = isNewCmdHigherPriority(pCmdNew, pRefCmd);
            if((canBePreempted == true) && (pPreemptCmd == NULL))
            {
                pPreemptCmd = pRefCmd;
            }
            /* Any individual command found in the queue which has higher priority
               than the pCmdNew, the preemption criterion will clear the preemption pointer. */
            else if (canBePreempted == false)
            {
                cntH2ClientHigherthanNewCmd++;
            }
        }
        pRefCmd = (RF_Cmd*)List_next((List_Elem*)pRefCmd);
    }

    /* If there was no commands belonging to h2 in the queue,
       return with the requesting command */
    if(noH2Cmd == true)
    {
        pPreemptCmd = NULL;  // no preemption

        if (isQEmpty)
        {
            *isQEmpty = true;
        }
    }
    else if(cntH2ClientHigherthanNewCmd > 0)
    {
        pPreemptCmd = NULL;  // no preemption
    }
    else
    {
        if (isQEmpty)
        {
            *isQEmpty = false;
        }
    }

    return pPreemptCmd;
}


static RF_Stat dmmPreemptCmd(RF_Cmd* pCmdPreempt)
{

    /* Abort the running command per default*/
    uint8_t stopOrAbort = 0;

    /* Abort multiple radio commands implicitly, mark them as preempted */
    return (RF_flushCmd(pCmdPreempt->pClient, RF_CMDHANDLE_FLUSH_ALL,
                        (stopOrAbort | (uint8_t)RF_ABORT_PREEMPTION)));
}

/* check if the time1 and time2 are in order */
static bool isTimeInOrder(uint32_t earlyTime, uint32_t laterTime)
{
    uint32_t timeGap;
    bool result = false;

    if(laterTime > earlyTime)
    {
      timeGap = laterTime - earlyTime;
      timeGap = MAX_RAT_TIME_US - timeGap;
        if(timeGap < WRAPAROUND_LIMIT_US) {
          result = false;
        } else {
          result = true;
        }
    }
    else
    {
        // wrap around
        timeGap = earlyTime - laterTime;
        timeGap = MAX_RAT_TIME_US - timeGap;
        if(timeGap < WRAPAROUND_LIMIT_US) {
            result = true;
        }
        else {
            result = false;
        }
    }
    return(result);
}

/* calculate the time gap between time1 and time2, assuming time2 comes after time1 */
static uint32_t getAbsTimeGap (uint32_t earlyTime, uint32_t laterTime)
{
    uint32_t timeGap;
    if(laterTime > earlyTime)
        timeGap = laterTime - earlyTime;
    else
    {
        // wrap around
        timeGap = earlyTime - laterTime;
        timeGap = MAX_RAT_TIME_US - timeGap;
    }
    return (timeGap);
}

/* check if the time gap between time1 and time2 is bigger than the limitGap, assuming time2 comes after time1 */
static bool isGapOK(uint32_t earlyTime, uint32_t laterTime, uint32_t limitGap)
{
    uint32_t time1InUS = (uint32_t)RF_convertRatTicksToUs(earlyTime);
    uint32_t time2InUS = (uint32_t)RF_convertRatTicksToUs(laterTime);
    if(isTimeInOrder(time1InUS, time2InUS))
    {
        uint32_t timeGap = getAbsTimeGap(time1InUS, time2InUS);
        if(timeGap > limitGap)
            return true;
        else
            return false;
    }
    else
    {
        /* time is not in order */
        return false;
    }
}

static uint32_t getRequiredDelay(uint32_t earlyTime, uint32_t laterTime, uint32_t limitGap)
{
    uint32_t time1InUS = (uint32_t)RF_convertRatTicksToUs(earlyTime);
    uint32_t time2InUS = (uint32_t)RF_convertRatTicksToUs(laterTime);

    if(isTimeInOrder(time1InUS, time2InUS))
    {
        uint32_t timeGap = getAbsTimeGap(time1InUS, time2InUS);
        if(timeGap > limitGap)
            return (0);
        else
            return (uint32_t)((limitGap - timeGap)*RF_NUM_RAT_TICKS_IN_1_US);
    }
    else
    {
        /* time order is wrong, so required delay should be max, so that it cannot be scheudled */
        return ((uint32_t)(SCHEDULE_LIMIT_US*RF_NUM_RAT_TICKS_IN_1_US));
    }

}

/* check BLOCK from policy table */
static bool isBlockModeOn (RF_Cmd* pCmd)
{
    uint32_t clientIdx;
    bool status = false;

    if (pCmd)
    {
        for(clientIdx = (uint32_t)0; clientIdx < MAX_DMM_CLIENTS; clientIdx++)
        {
            if( pCmd->pClient == dmmClients[clientIdx].clientHndl)
            {
                status = dmmClients[clientIdx].isBlocked;
                break;
            }
        }
    }

    return(status);
}

/***** Public function definitions *****/

/** @brief  Function to initialize the DMMSch_Params
 *  *       This function is currently a placeholder. To follow the convention,
 *          we provide this function for application level.
 *
 *  @param  params      An pointer to RF_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 */
void DMMSch_Params_init(DMMSch_Params *params)
{
    memset(params, 0, sizeof(DMMSch_Params));
}

/** @brief  Function that initializes the DMMSch module
 *
 */
void DMMSch_init(void)
{
    // create semaphore instance if not already created
    if (!dmmInitialized)
    {
        //Create a semaphore for blocking commands
        SemaphoreP_Params semParams;

        // init params
        SemaphoreP_Params_init(&semParams);

        DmmSchedulerMutex = SemaphoreP_create(0, &semParams);

        /* Assert */
        DebugP_assert(DmmSchedulerMutex != NULL);


#ifdef DMM_RX_RESUME
        /* init cmd callback table */
        rxResumeCmd.cmdHndl = RF_SCHEDULE_CMD_ERROR;
        rxResumeCmd.cmdResubmitted = false;
        rxResumeCmd.pPendQ = NULL;
        rxResumeCmd.pOp = NULL;
#endif
        dmmInitialized = true;

        SemaphoreP_post(DmmSchedulerMutex);
    }
}

/** @brief  Function to open the DMMSch module
 *          This function is currently a placeholder. To follow the convention,
 *          we provide this function for application level. In the future,
 *          when it is needed, we can add more code to this function.
 *
 *  @param  params      An pointer to RF_Params structure for initialization
 */
void DMMSch_open(DMMSch_Params *params)
{
    DebugP_assert(params != NULL);

}

/** @brief  Register a DMM Scheduler client
 *
 *  @param  pTaskHndl  - Task handle that the stack is running in, used to map the
 *                       RF Client handle to a stack role
 *
 *  @param  StackRole  - stack role associated with Task handle
 */
#ifndef FREERTOS
void DMMSch_registerClient(Task_Handle* pTaskHndl, DMMPolicy_StackRole StackRole)
#else
void DMMSch_registerClient(TaskHandle_t pTaskHndl, DMMPolicy_StackRole StackRole)
#endif
{
    if(numDmmClients < MAX_DMM_CLIENTS)
    {
        dmmClients[numDmmClients].pTaskHndl = pTaskHndl;
        dmmClients[numDmmClients].stackRole = StackRole;

        numDmmClients++;
    }
}

/** @brief  Turn on Block mode. It aborts the current command if it belongs to the same client.
 *
 *  @param  stackRole   stack role associated with Task handle
 *  @return true: success, false: the stack role cannot be found
 */
bool DMMSch_setBlockModeOn(DMMPolicy_StackRole stackRole) {

    uint32_t clientIdx;
    for(clientIdx = (uint32_t)0; clientIdx < MAX_DMM_CLIENTS; clientIdx++)
    {
        /* find the stack role and set the isBlocked flag true */
        if (dmmClients[clientIdx].stackRole == stackRole)
        {
            dmmClients[clientIdx].isBlocked = true;

            /* Abort the running command */
            RF_InfoVal info;

            if(RF_StatSuccess == RF_getInfo(dmmClients[clientIdx].clientHndl, RF_GET_CURR_CMD, &info)) {

                uint8_t stopOrAbort = 0;

                RF_cancelCmd(dmmClients[clientIdx].clientHndl, info.ch, (stopOrAbort | (uint8_t)RF_ABORT_PREEMPTION));
            }
            return (true);
        }
    }
    // incorrect stack role
    return (false);
}

/** @brief  Turn off Block mode
 *
 *  @param  stackRole   stack role associated with Task handle
 *  @return true: success, false: the stack role cannot be found
 */
bool DMMSch_setBlockModeOff(DMMPolicy_StackRole stackRole) {

    uint32_t clientIdx;
    for(clientIdx = (uint32_t)0; clientIdx < MAX_DMM_CLIENTS; clientIdx++)
    {
        /* find the stack role and set the isBlocked flag false */
        if (dmmClients[clientIdx].stackRole == stackRole)
        {
            dmmClients[clientIdx].isBlocked = false;
            return (true);
        }
    }
    // incorrect stack role
    return(false);
}

/** @brief  Get Block mode status
 *
 *  @param  StackRole   stack role associated with Task handle
 *
 *  return  True: Block Mode is On, False: Block Mode is Off
 */
bool DMMSch_getBlockModeStatus(DMMPolicy_StackRole stackRole) {

    uint32_t clientIdx;
    for(clientIdx = (uint32_t)0; clientIdx < MAX_DMM_CLIENTS; clientIdx++)
    {
        /* find the stack role and return isBlocked flag*/
        if (dmmClients[clientIdx].stackRole == stackRole)
        {
            return (dmmClients[clientIdx].isBlocked);
        }
    }

    // If stackRole isn't found, return false
    return false;
}

/** @brief  allows policy manager to register a callback on command preemption
 *
 *  @param  dmmSchPreemptionCb     callback to register
 */
void DMMSch_registerPreemptionCb(DMMSch_PreemptionCb dmmSchPreemptionCb)
{
    preemptionCb = dmmSchPreemptionCb;
}

/** @brief  Intercepts calls from a stack to RF_postCmd (re-mapped to DMMSch_rfOpen),
 *          The DMMSch module uses this to tie
 *
 *  @param pObj         Pointer to a #RF_Object that will hold the state for this
 *                      RF client. The object must be in persistent and writeable
 *                      memory.
 *  @param pRfMode      Pointer to a #RF_Mode struct holding PHY information
 *  @param pRadioSetup  Pointer to the radio setup command used for this client.
 *                      This is re-executed by the RF Driver on each power-up.
 *  @param params       Pointer to an RF_Params object with the desired driver configuration.
 *                      A NULL pointer results in the default configuration being loaded.
 *  @return             A handle for further RF driver calls on success. Otherwise NULL.
 */
RF_Handle DMMSch_rfOpen(RF_Object *pObj, RF_Mode *pRfMode, RF_RadioSetup *pOpSetup, RF_Params *params)
{
    DMMSch_Client_t *dmmClient;
    RF_Handle clientHndl = NULL;
    uint32_t phySwitchingTime;
    /* block other callers until RF driver is opened for this client */
    SemaphoreP_pend(DmmSchedulerMutex, SemaphoreP_WAIT_FOREVER);

    /* override the RF_Mode and patches for multi-mode operation */
    pRfMode->cpePatchFxn = rf_patch_cpe_ieee_coex;
#if defined(CC13X2R1_LAUNCHXL) || defined(CC13X2P1_LAUNCHXL)
    pRfMode->rfMode = (uint8_t)RF_MODE_AUTO;
#else
    pRfMode->rfMode = (uint8_t)RF_MODE_MULTIPLE;
#endif

    dmmClient = getClient();

    clientHndl = RF_open(pObj, pRfMode, pOpSetup, params);

    /* if we found the client then save the handle, note that for 15.4 SLR Phy
     * mode the RF_open is called first from macTask and called again from the MAC app task.
     * The same RF Handle value (clientHndl) is used from both calls. So, we can save only once.
     *
     */
    if(dmmClient)
    {
        dmmClient->clientHndl = clientHndl;
        DMMPolicy_setStackID(params->nID, dmmClient->stackRole);
        dmmClient->isBlocked = false;
    }
    //release mutex
    SemaphoreP_post(DmmSchedulerMutex);

    phySwitchingTime = (STACK_RF_CB_LATENCY + DMM_CONFICT_HOOK_DELAY + DMM_DEFAULT_PHY_SWITCHING_MARGIN);
    RF_control(clientHndl, RF_CTRL_SET_PHYSWITCHING_DURATION_MARGIN, &phySwitchingTime);

    return clientHndl;
}

/**
 *  @brief  Handles calls from a stack to RF_postCmd (re-mapped to DMMSch_postCmd),
 *  adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *  @sa RF_pendCmd(), RF_runCmd(), RF_scheduleCmd(), RF_RF_cancelCmd(), RF_flushCmd(), RF_getCmdOp()
 *
 *  @param h         Driver handle previously returned by RF_open()
 *  @param pOp       Pointer to the RF operation command.
 *  @param ePri      Priority of this RF command (used for arbitration in multi-client systems)
 *  @param pCb       Callback function called during command execution and upon completion.
 *                   If RF_postCmd() fails, no callback is made.
 *  @param bmEvent   Bitmask of events that will trigger the callback or that can be pended on.
 *  @return          A handle to the RF command. Return value of RF_ALLOC_ERROR indicates error.
 */
RF_CmdHandle DMMSch_rfPostCmd(RF_Handle h, RF_Op* pOp, RF_Priority ePri, RF_Callback pCb, RF_EventMask bmEvent)
{
    RF_CmdHandle cmdHndl;
    RF_ScheduleCmdParams schParams;
    uint32_t hwiKey, swiKey;

    /* use default values for RF_ScheduleCmdParams for now. allowed delay will be determined based on trigger Type, priority will be based on policy table */
    schParams.allowDelay = RF_AllowDelayAny;
    schParams.endTime =0;

    hwiKey = HwiP_disable();
    swiKey = SwiP_disable();

    /* simply call RF driver. RF driver will invoke the DMM hook function later. */
    cmdHndl = RF_scheduleCmd(h, pOp, &schParams, pCb, bmEvent);

    SwiP_restore(swiKey);
    HwiP_restore(hwiKey);

    return cmdHndl;
}

/**
 *  @brief  Handles calls from a stack to RF_scheduleCmd (re-mapped to DMMSch_scheduleCmd),
 *          adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *  @param h         Handle previously returned by RF_open()
 *  @param pOp       Pointer to the #RF_Op. Must normally be in persistent and writeable memory
 *  @param pSchParams Pointer to the schedule command parameter structure
 *  @param pCb       Callback function called upon command completion (and some other events).
 *                   If RF_scheduleCmd() fails no callback is made
 *  @param bmEvent   Bitmask of events that will trigger the callback.
 *  @return          A handle to the RF command. Return value of RF_ALLOC_ERROR indicates error.
 */
uint8_t schDgbFlg = 0;
uint8_t schBleFLushFlg = 1;
uint32_t pSchErr = 0;
RF_CmdHandle DMMSch_rfScheduleCmd(RF_Handle h, RF_Op* pOp, RF_ScheduleCmdParams *pSchParams, RF_Callback pCb, RF_EventMask bmEvent)
{
    RF_CmdHandle cmdHndl;
    uint32_t hwiKey, swiKey;

    hwiKey = HwiP_disable();
    swiKey = SwiP_disable();

    /* simply call RF driver. RF driver will invoke the DMM hook function later. */
    cmdHndl = RF_scheduleCmd(h, pOp, pSchParams, pCb, bmEvent);

    SwiP_restore(swiKey);
    HwiP_restore(hwiKey);

    return cmdHndl;
}

/** @brief  Handles calls from a stack to RF_runCmd (re-mapped to DMMSch_runCmd),
 *          adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *  @param h         Driver handle previously returned by RF_open()
 *  @param pOp       Pointer to the RF operation command.
 *  @param ePri      Priority of this RF command (used for arbitration in multi-client systems)
 *  @param pCb       Callback function called during command execution and upon completion.
 *                   If RF_runCmd() fails, no callback is made.
 *  @param bmEvent   Bitmask of events that will trigger the callback or that can be pended on.
 *  @return          The relevant termination event.
 */
RF_EventMask DMMSch_rfRunCmd(RF_Handle h, RF_Op* pOp, RF_Priority ePri, RF_Callback pCb, RF_EventMask bmEvent)
{
    return RF_runCmd(h, pOp, ePri, pCb, bmEvent);
}

/**
 *  @brief  Handles calls from a stack to RF_runScheduleCmd (re-mapped to DMMSch_runScheduleCmd),
 *          adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *
 *  @param h         Handle previously returned by RF_open()
 *  @param pOp       Pointer to the #RF_Op. Must normally be in persistent and writeable memory
 *  @param pSchParams Pointer to the schedule command parameter structure
 *  @param pCb       Callback function called upon command completion (and some other events).
 *                   If RF_runScheduleCmd() fails, no callback is made.
 *  @param bmEvent   Bitmask of events that will trigger the callback.
 *  @return          The relevant command completed event.
 */
RF_EventMask DMMSch_rfRunScheduleCmd(RF_Handle h, RF_Op* pOp, RF_ScheduleCmdParams *pSchParams, RF_Callback pCb, RF_EventMask bmEvent)
{
    return RF_runScheduleCmd(h, pOp, pSchParams, pCb, bmEvent);
}

/**
 *  @brief  Abort/stop/cancel single command in command queue.
 *
 *  If command is running, aborts/stops it and posts callback for the
 *  aborted/stopped command. <br>
 *  If command has not yet run, cancels it it and posts callback for the
 *  cancelled command. <br>
 *  If command has already run or been aborted/stopped/cancelled, has no effect.<br>
 *  If RF_cancelCmd is called from a Swi context with same or higher priority
 *  than RF Driver Swi, when the RF core is powered OFF -> the cancel callback will be delayed
 *  until the next power-up cycle.<br>
 *
 *  @note Calling context : Task/SWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param ch           Command handle previously returned by RF_postCmd().
 *  @param mode         1: Stop gracefully, 0: abort abruptly
 *  @return             RF_Stat indicates if command was successfully completed
 */
RF_Stat DMMSch_rfCancelCmd(RF_Handle h, RF_CmdHandle ch, uint8_t mode)
{
    RF_Stat status = RF_StatError;

    DMM_dbgLog5("Cancel: Stack:%x ch:%x mode:%x", h->clientConfig.nID, ch, mode, 0, 0);

#ifdef DMM_RX_RESUME
    if (rxResumeCmd.cmdHndl == ch)
    {
        rxResumeCmd.cmdResubmitted = true;
        status = RF_cancelCmd(h, ch, mode);
        DMM_dbgLog1("DMMSch_rfCancelCmd: RF_cancelCmd status %x", status);

        if(status != RF_StatSuccess)
        {
            /* command was not running so flush pendQ */
            RF_flushCmd(h, ch, mode);
            status = RF_StatSuccess;
        }
    }
    else
    {
        status = RF_cancelCmd(h, ch, mode);
    }
#else
    status = RF_cancelCmd(h, ch, mode);
#endif
    return status;
}

/**
 *  @brief  Abort/stop/cancel command and any subsequent commands in command queue.
 *
 *  If command is running, aborts/stops it and then cancels all later commands in queue.<br>
 *  If command has not yet run, cancels it and all later commands in queue.<br>
 *  If command has already run or been aborted/stopped/cancelled, has no effect.<br>
 *  The callbacks for all cancelled commands are issued in chronological order.<br>
 *  If RF_flushCmd is called from a Swi context with same or higher priority
 *  than RF Driver Swi, when the RF core is powered OFF -> the cancel callback will be delayed
 *  until the next power-up cycle.<br>
 *
 *  @note Calling context : Task/SWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param ch           Command handle previously returned by RF_postCmd().
 *  @param mode         1: Stop gracefully, 0: abort abruptly
 *  @return             RF_Stat indicates if command was successfully completed
 */
RF_Stat DMMSch_rfFlushCmd(RF_Handle h, RF_CmdHandle ch, uint8_t mode)
{
    DMM_dbgLog3("DMMSch_rfFlushCmd: c=%x, ch=%x, mode=%x", h, ch, mode);

#ifdef DMM_RX_RESUME
    if (rxResumeCmd.cmdHndl == ch)
    {
        /* force submitted flag in case it is the process of being submitted */
        rxResumeCmd.cmdResubmitted = true;
    }
#endif

    return RF_flushCmd(h, ch, mode);
}

/**
 *  @brief Send any Immediate command. <br>
 *
 *  Immediate Comamnd is send to RDBELL, if radio is active and the RF_Handle points
 *  to the current client. <br>
 *  In other appropriate RF_Stat values are returned. <br>
 *
 *  @note Calling context : Task/SWI/HWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param pCmdStruct   Pointer to the immediate command structure
 *  @return             RF_Stat indicates if command was successfully completed
*/
RF_Stat DMMSch_rfRunImmediateCmd(RF_Handle h, uint32_t* pCmdStruct)
{
    return RF_runImmediateCmd(h, pCmdStruct);
}

/**
 *  @brief Send any Direct command. <br>
 *
 *  Direct Comamnd value is send to RDBELL immediately, if radio is active and
 *  the RF_Handle point to the current client. <br>
 *  In other appropriate RF_Stat values are returned. <br>
 *
 *  @note Calling context : Task/SWI/HWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param cmd          Direct command value.
 *  @return             RF_Stat indicates if command was successfully completed.
*/
RF_Stat DMMSch_rfRunDirectCmd(RF_Handle h, uint32_t cmd)
{
    return RF_runDirectCmd(h, cmd);
}

RF_Stat DMMSch_rfRequestAccess(RF_Handle h, RF_AccessParams *pParams)
{
    /* request access RF API should not be used in DMM */
    return RF_StatError;
}

/**
 *  Determines the method to resolve the schedule conflicts.
 *
 *  Input:  pCmdBg               - Running background command.
 *          pCmdFg               - Running foreground command.
 *          pPendQueue           - Pointer to the head structure of pend queue.
 *          pDoneQueue           - Pointer to the head structure of done queue.
 *          bConflict            - Boolean to indicate a conflict.
 *          pConflictCmd         - Pointer to the conflict command.
 *  Return: RF_Conflict          - Identifies the method to resolve the conflict.
 */
RF_ExecuteAction DMM_resolveConflictCb(RF_Cmd* pCmdBg, RF_Cmd* pCmdFg, List_List* pPendQueue, List_List* pDoneQueue, bool bConflict, RF_Cmd* pConflictCmd)
{
    RF_ExecuteAction conflictResolution =  RF_ExecuteActionNone;
    RF_Cmd* pPendCmd = NULL;
    RF_Cmd* pSearchCmd = (RF_Cmd*)List_head(pPendQueue);

    if(!bConflict)
    {
        /* check BLOCK mode first if no conflict */
        /* note that the pendQ cannot be NULL here */
        if (isBlockModeOn(pSearchCmd)) {
            conflictResolution = RF_ExecuteActionRejectIncoming;
        }
    }
    else
    {
        /* Look for the first abs command, conflicts will only happen if the next
         * command is an abs command. TRIG_NOW commands can always be delayed
         * there must be an ABS command if bConflict = true */
        while (pSearchCmd)
        {
            if (pSearchCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
            {
                pPendCmd = pSearchCmd;
                break;
            }
            else
            {
                pSearchCmd = (RF_Cmd*)List_next((List_Elem*)pSearchCmd);
            }
        }

        /* check BLOCK mode first even for the case of a conflict */
        if (isBlockModeOn(pSearchCmd))
        {
            conflictResolution = RF_ExecuteActionRejectIncoming;
        }
        else
        {
            /* Normal conflict Hook function */
            if(pCmdBg && pPendCmd)
            {
                if(isSameClient(pPendCmd, pCmdBg))
                {
                    /* the commands belong to the same stack, no conflict */
                }
                else if((!isNewCmdHigherPriority(pPendCmd, pCmdBg)) || (pCmdFg && (!isNewCmdHigherPriority(pPendCmd, pCmdFg))))
                {
                    conflictResolution = RF_ExecuteActionRejectIncoming;
                    if(preemptionCb)
                    {
                        /* inform policy manager that a stack command has been preempted */
                        preemptionCb(getClientStackRole(pPendCmd->pClient));
                    }
                }
                else
                {
                    conflictResolution = RF_ExecuteActionAbortOngoing;
                    if(preemptionCb)
                    {
                        /* inform policy manager that a stack command has been preempted */
                        preemptionCb(getClientStackRole(pCmdBg->pClient));
                    }

        #ifdef DMM_RX_RESUME
                    /* for rx resume */
                    if(pCmdBg->pCb  && ( (pCmdBg->pOp->commandNo == CMD_IEEE_RX) ||
                            (pCmdBg->pOp->commandNo == CMD_PROP_RX_ADV) ||
                            (pCmdBg->pOp->commandNo == CMD_PROP_RX)) )
                    {
                        uint32_t key;
                        RF_ScheduleCmdParams SchParamsResubmit;

                        SchParamsResubmit.allowDelay = pCmdBg->allowDelay;
                        SchParamsResubmit.endTime    = pCmdBg->endTime;

                        key = HwiP_disable();

                        /* since in this new scheduler, we re-submit the command
                         * immediately, we just need to save "cmdHndl" and "pCb" for
                        * dmm version call back function to find the command's original
                        * callback. */
                        rxResumeCmd.h = pCmdBg->pClient;
                        rxResumeCmd.cmdHndl = pCmdBg->ch;

                        /* Check that the client callback has not been set to
                         * rxResumeCmdCb already. This will happen if a previously
                         * resumed command is being resumed again. We do not want to
                         * rxResumeCmd.cb to rxResumeCmdCb or as it will reenter
                         * rxResumeCmdCb in a tight loop */
                        if(pCmdBg->pCb != rxResumeCmdCb)
                        {
                            /* save the original callback in the rxResumeCmd structure.
                             * So the re-submitted command callback is the original one.
                             * Then when the command has any events other than abort, the
                             * original callback will be invoked. */
                            rxResumeCmd.cb = pCmdBg->pCb;

                            /* change the client callback to DMM version of the callback so
                             * that the stack won't be notified for this current abort action
                             * (because it will be resumed by DMM soon). */
                            pCmdBg->pCb = rxResumeCmdCb;
                        }

                        rxResumeCmd.bmEvent = pCmdBg->bmEvent;
                        rxResumeCmd.SchParamsDmm.endTime = SchParamsResubmit.endTime;
                        rxResumeCmd.SchParamsDmm.allowDelay = SchParamsResubmit.allowDelay;
                        rxResumeCmd.SchParamsDmm.activityInfo = pCmdBg->activityInfo;
                        rxResumeCmd.SchParamsDmm.duration = pCmdBg->duration;
                        rxResumeCmd.SchParamsDmm.startType = RF_StartNotSpecified;  /* Trig Now for resumed Rx */
                        rxResumeCmd.SchParamsDmm.startTime = 0;                     /* Trig Now for resumed Rx */
                        rxResumeCmd.pOp = pCmdBg->pOp;
                        rxResumeCmd.pOp->startTrigger.triggerType = TRIG_NOW; /* Trig Now for resumed Rx */
                        rxResumeCmd.cmdResubmitted = false;
                        rxResumeCmd.pPendQ = pPendQueue;

                        /* Exit critical section. */
                        HwiP_restore(key);
                    }
        #endif // #ifdef DMM_RX_RESUME
                } //if(isSameClient(pPendCmd, pCmdBg))
            }
        } //if (isBlockModeOn(pSearchCmd))
    } //if(!bConflict)

    DMM_dbgLog5("conflictCb: Bg:%x Pend:%x ID%x:%x s%x",
                pCmdBg->pOp->commandNo, pPendCmd->pOp->commandNo, pCmdBg->pClient->clientConfig.nID, pPendCmd->pClient->clientConfig.nID, conflictResolution);

    return(conflictResolution);
}

/**
 *  Sorts and adds command to the RF driver internal command queue.
 *
 *  Input:  pCmdNew              - Pointer to the command to be submitted.
 *          pCmdBg               - Running background command.
 *          pCmdFg               - Running foreground command.
 *          pPendQueue           - Pointer to the head structure of pend queue.
 *          pDoneQueue           - Pointer to the head structure of done queue.
 *  Return: RF_ScheduleStatus    - Identifies the success or failure of enquing.
 */
RF_ScheduleStatus  DMM_commandScheduleCb(RF_Cmd* pCmdNew, RF_Cmd* pCmdBg, RF_Cmd* pCmdFg, List_List* pPendQueue, List_List* pDoneQueue)
{
//    RF_Cmd* pHeadCmd = (RF_Cmd*)List_head(pPendQueue);

    RF_ScheduleStatus schStatus = RF_ScheduleStatusError;
    RF_Handle h, h2;
    RF_Stat    rfPreemptStats = RF_StatError;
    bool isQEmpty = false;

    /* check BLOCK mode */
    if (isBlockModeOn(pCmdNew))
    {
        if(pCmdNew->pOp->startTrigger.triggerType == TRIG_ABSTIME) {
            /* check if the start time of the command is within MIN_TIME_FOR_BLOCK_MODE_US */
            uint32_t currentTime = RF_getCurrentTime();
            uint32_t cmdTime = (uint32_t)MIN_TIME_FOR_BLOCK_MODE_US;
            if(!isGapOK(currentTime, pCmdNew->startTime,cmdTime)) return(schStatus);
        }
        /* reject if the command is TRIG_NOW */
        else if(pCmdNew->pOp->startTrigger.triggerType == TRIG_NOW) return(schStatus);
    }

    /* check input parameters */
    if(pCmdNew->startType == RF_StartAbs)
    {
        /*check input time parameters */
        if(pCmdNew->duration > SCHEDULE_LIMIT_US*RF_NUM_RAT_TICKS_IN_1_US)
        {
            /* min duration is not specified or too long */
            return(RF_ScheduleStatusError);
        }

        if(pCmdNew->pOp->startTrigger.triggerType != TRIG_ABSTIME)
        {
            /* startType does not match */
            return(RF_ScheduleStatusError);
        }
    }

    dmmAdjustCmdSettings(pCmdNew);

    /* schedule the new command*/
    schStatus =  dmmScheduleCmd(pCmdBg, pCmdNew, pPendQueue);

    if(schStatus < RF_ScheduleStatusNone)
    {
        /* Schedule failed, Now check whether commands in the queue can be preempted. */

        h = pCmdNew->pClient;

        if( h == dmmClients[0].clientHndl)
        {
            h2 = dmmClients[1].clientHndl;
        }
        else
        {
            h2 = dmmClients[0].clientHndl;
        }

#if CONFLICT_FLUSH_ALL
        RF_Cmd* pCmdPreempted = dmmCheckPreemption(h2, pCmdNew, pCmdBg, pPendQueue, &isQEmpty);

        if (pCmdPreempted)
        {
            preemptCnt++;
            rfPreemptStats = dmmPreemptCmd(pCmdPreempted);
            // if we preempted some commands
            if ( rfPreemptStats == RF_StatSuccess)
            {
                // try to insert command one more time.
                if (isClientMatched(h2, pCmdBg))
                {
                    // this time, BgCmd must be NULL, but RF driver can be in the middle of aborting the BgCmd
                    schStatus = dmmScheduleCmd(NULL, pCmdNew, pPendQueue);
                }
                else
                {
                    schStatus = dmmScheduleCmd(pCmdBg, pCmdNew, pPendQueue);
                }
            }
        }
        else if (isQEmpty == (bool)true)
        {
            /* This may happen if there was a command in the queue when RF_scheduleCmd was called
             * but the command was served/completed before the program checks the preemption
             * dmmCheckPreemption(). So, the Q is empty. In this case, we should re-schedule.*/
            schStatus =dmmScheduleCmd(pCmdBg, pCmdNew, pPendQueue);
        }

#else
        /* reject command individually */
        if (RF_STACK_ID_BLE == h2->clientConfig.nID)
        {
          /* Find a single command to preempt or reject. The new cmd must be absolute time cmd and BG exists, so no need to check the condition */
            schStatus = dmmSinglePreemptCheck(pCmdBg, pCmdNew, pPendQueue);
            if(schStatus == RF_ScheduleStatusNone)
            {
                /* cannot find conflict, so try schedule again */
                /* This may happen if there was a command in the queue when RF_scheduleCmd was called
                 * but the command was served/completed before the program checks the preemption
                 * In this case, we should re-schedule.*/
                schStatus = dmmScheduleCmd(pCmdBg, pCmdNew, pPendQueue);
            }
            else if (schStatus > RF_ScheduleStatusNone)
            {
                preemptCnt++;
            }
        }
        else
        {
            RF_Cmd* pCmdPreempted = dmmCheckPreemption(h2, pCmdNew, pCmdBg, pPendQueue, &isQEmpty);

            if (pCmdPreempted)
            {
                preemptCnt++;
                rfPreemptStats = dmmPreemptCmd(pCmdPreempted);
                // if we preempted some commands
                if ( rfPreemptStats == RF_StatSuccess)
                {
                    // try to insert command one more time.
                    schStatus = dmmScheduleCmd(pCmdBg, pCmdNew, pPendQueue);
                }
            }
            else if (isQEmpty == (bool)true)
            {
                /* This may happen if there was a command in the queue when RF_scheduleCmd was called
                 * but the command was served/completed before the program checks the preemption
                 * dmmCheckPreemption(). So, the Q is empty. In this case, we should re-schedule.*/
                schStatus =dmmScheduleCmd(pCmdBg, pCmdNew, pPendQueue);
            }
        }

#endif
    }

#ifdef DMM_RX_RESUME
    /* check if we successfully inserted the command and there is a command being resumed*/
    if( (schStatus > RF_ScheduleStatusNone) && (rxResumeCmd.h == pCmdNew->pClient) && (rxResumeCmd.cmdHndl != RF_SCHEDULE_CMD_ERROR) )
    {
        RF_Cmd* pPendQIterator = (RF_Cmd*)List_head(pPendQueue);

        /* check if this is an Rx-Resume command and if so replace the handle */
        /* Walk the queue.*/
        while (pPendQIterator)
        {
            /* Check if there is a matching Rx-Resume command */
            if ((rxResumeCmd.pOp == pPendQIterator->pOp)&&(pPendQIterator->pCb == rxResumeCmdCb))
            {
                /* Replace cmdHandle with the original one */
                DMM_dbgLog1("scheduleCb: resumed c%x", rxResumeCmd.cmdHndl);
                pPendQIterator->ch = rxResumeCmd.cmdHndl;

                break;
            }
            else
            {
                pPendQIterator = (RF_Cmd*)List_next((List_Elem*)pPendQIterator);
            }
        }
    }
#endif //#ifdef DMM_RX_RESUME


    if( (preemptionCb) && (schStatus == RF_ScheduleStatusError) )
    {
        /* inform policy manager that a stack command has been preempted */
        preemptionCb(getClientStackRole(pCmdNew->pClient));
    }


#ifdef DMM_DEBUG_LOGGING
    if(pCmdNew->pClient->clientConfig.nID == RF_STACK_ID_154)
    {
        DMM_dbgLog5("ScheduleCmd: Stack:%x cmd:%x Act:%x TrigType:%x Delay:%x",
                    pCmdNew->pClient->clientConfig.nID, pCmdNew->pOp->commandNo, pCmdNew->activityInfo,pCmdNew->pOp->startTrigger.triggerType, pCmdNew->allowDelay);

        RF_Cmd* pTemp = (RF_Cmd*)List_head(pPendQueue);
        while(pTemp != NULL)
        {
            DMM_dbgLog5("PendQ: Stack:%x cmd:%x Act:%x TrigType:%x Delay:%x",
                        pTemp->pClient->clientConfig.nID, pTemp->pOp->commandNo, pTemp->activityInfo,pTemp->pOp->startTrigger.triggerType, pTemp->allowDelay);
            pTemp = (RF_Cmd*)List_next((List_Elem*)pTemp);
        }
    }
#endif

    return schStatus;
}

#if !CONFLICT_FLUSH_ALL
static RF_ScheduleStatus dmmSinglePreemptCheck(RF_Cmd* pCmdBg, RF_Cmd* pCmdNew, List_List* pPendQueue)
{
    RF_ScheduleStatus    scheduleResult = RF_ScheduleStatusError;
    DMM_ConflictStatus   conflictStatus;
    RF_Cmd* pRefCmd;
    RF_Cmd* pAfterRefCmd;
    uint8_t mode = RF_ABORT_PREEMPTION;

    /* Typecast the arguments to RF commands. */
    RF_Cmd* pHead = (RF_Cmd*)List_head(pPendQueue);

    /* special case, with BgCmd */
    conflictStatus = dmmCmdConflictCheck(pCmdBg, pCmdNew, pHead);
    switch(conflictStatus)
    {
        case DMM_ConflictWithPrev:
            /* just schedule at top, let conflict hook takes care of priority */
            List_putHead(pPendQueue, (List_Elem*)pCmdNew);
            return(RF_ScheduleStatusTop);
        case DMM_ConflictWithNext:
        case DMM_ConfictWithBoth:
            if(isNewCmdHigherPriority(pCmdNew, pHead))
            {
                /* remove the next command from the Pend Q, then schedule at top */
                RF_cancelCmd(pHead->pClient, pHead->ch, mode);
                List_putHead(pPendQueue, (List_Elem*)pCmdNew);
                return(RF_ScheduleStatusTop);
            }
            else
            {
                return(RF_ScheduleStatusError);
            }
        default:
            /* move down to Pend Q */
            break;
    }

    /* Load list head as the start point of the iterator. */
    pRefCmd = pHead;

    /* Walk the queue.*/
    while (pRefCmd)
    {
        pAfterRefCmd = (RF_Cmd*)List_next((List_Elem*)pRefCmd);
        conflictStatus = dmmCmdConflictCheck(pRefCmd, pCmdNew, pAfterRefCmd);

        switch(conflictStatus)
        {
            case DMM_ConflictWithPrev:
                if(isNewCmdHigherPriority(pCmdNew, pRefCmd))
                {
                    /* insert to PendQ first before removing pRefCmd */
                    if (pAfterRefCmd)
                    {
                        List_insert(pPendQueue, (List_Elem*)pCmdNew, List_next((List_Elem*)pAfterRefCmd));
                        scheduleResult = RF_ScheduleStatusMiddle;
                    }
                    else
                    {
                        List_put(pPendQueue, (List_Elem*)pCmdNew);
                        scheduleResult = RF_ScheduleStatusTail;
                    }
                    RF_cancelCmd(pRefCmd->pClient, pRefCmd->ch, mode);
                    return(scheduleResult);
                }
                else
                {
                    return(RF_ScheduleStatusError);
                }
            case DMM_ConflictWithNext:
                if(isNewCmdHigherPriority(pCmdNew, pAfterRefCmd))
                {
                    if (List_next((List_Elem*)pAfterRefCmd))
                    {
                        List_insert(pPendQueue, (List_Elem*)pCmdNew, List_next((List_Elem*)pAfterRefCmd));
                        scheduleResult = RF_ScheduleStatusMiddle;
                    }
                    else
                    {
                        List_put(pPendQueue, (List_Elem*)pCmdNew);
                        scheduleResult = RF_ScheduleStatusTail;
                    }
                    RF_cancelCmd(pAfterRefCmd->pClient, pAfterRefCmd->ch, mode);
                    return(scheduleResult);
                }
                else
                {
                    return(RF_ScheduleStatusError);
                }
            case DMM_ConfictWithBoth:
                if(isNewCmdHigherPriority(pCmdNew, pRefCmd))
                {
                    /*pAfterRefCmd must not NULL */
                    List_insert(pPendQueue, (List_Elem*)pCmdNew, (List_Elem*)pAfterRefCmd);
                    RF_cancelCmd(pRefCmd->pClient, pRefCmd->ch, mode);
                }
                if(isNewCmdHigherPriority(pCmdNew, pAfterRefCmd))
                {
                    /* no need to insert the new cmd here */
                    RF_cancelCmd(pAfterRefCmd->pClient, pAfterRefCmd->ch, mode);
                }
                return(RF_ScheduleStatusMiddle);
            case DMM_NoConflict:
            default:
                /* move down to Pend Q */
                pRefCmd = (RF_Cmd*)List_next((List_Elem*)pRefCmd);
            break;

        }
    }

    /* cannot find conflict*/
    return(RF_ScheduleStatusNone);
}
#endif

static RF_ScheduleStatus dmmScheduleCmd(RF_Cmd* pCmdBg, RF_Cmd* pCmdNew, List_List* pPendQueue)
{
    RF_ScheduleStatus scheduleResult = RF_ScheduleStatusError;

    /* Typecast the arguments to RF commands. */
    RF_Cmd* pHead = (RF_Cmd*)List_head(pPendQueue);

    if(true == dmmCmdInsertCheck(pCmdBg, pCmdNew, pHead))
    {
        /* Insert command at the beginning of the queue */
        List_putHead(pPendQueue, (List_Elem*)pCmdNew);
        scheduleResult = RF_ScheduleStatusTop;
    }
    else
    {
        /* Load list head as the start point of the iterator. */
        RF_Cmd* pRefCmd = pHead;

        /* Walk the queue.*/
        while (pRefCmd)
        {
            /* Check if we can insert the new command in between. */
            if (dmmCmdInsertCheck(pRefCmd, pCmdNew, (RF_Cmd*)List_next((List_Elem*)pRefCmd)))
            {
                /* Set the return value that the new command should be
                   inserted in between RefCmd and RefCmd->pNext. */
                if (List_next((List_Elem*)pRefCmd))
                {
                    /* Insert command before pInsertLocation->next. */
                    List_insert(pPendQueue, (List_Elem*)pCmdNew, List_next((List_Elem*)pRefCmd));
                    scheduleResult = RF_ScheduleStatusMiddle;
                }
                else
                {
                    /* Append command to the end of the queue (if nextCmd does not exist). */
                    List_put(pPendQueue, (List_Elem*)pCmdNew);
                    scheduleResult = RF_ScheduleStatusTail;
                }
                break;
            }
            else
            {
                pRefCmd = (RF_Cmd*)List_next((List_Elem*)pRefCmd);
            }
        }
    }

    if ((scheduleResult == RF_ScheduleStatusError) && (pCmdNew->pOp->startTrigger.triggerType == TRIG_NOW))
    {
        /* put Trig_Now at the end */
        List_put(pPendQueue, (List_Elem*)pCmdNew);
        scheduleResult = RF_ScheduleStatusTail;
    }
    /* Return with the scheduling method. */
    return(scheduleResult);
}


/* rf driver has this function. We can access clientConfig.nPhySwitchingDuration but we want to make sure whether physwitch is there or not */
static int32_t getSwitchingTimeInUs(RF_Cmd* prevCmd, RF_Cmd* nextCmd)
{
    int32_t switchingTime = 0;

    /*  If otherCmd and newCmd are from different client then there is a switching time
        related to moving between the two commands. */
    if (prevCmd->pClient != nextCmd->pClient)
    {
        switchingTime = (int32_t)nextCmd->pClient->clientConfig.nPhySwitchingDuration;
    }

    /* Return the switching time related to moving between the two clients. */
    return(switchingTime);
}


/* check cmd insert after prev cmd (general case) */
static bool cmdInsertCheckAfterPrevCmd(RF_Cmd* pPrevCmd, RF_Cmd* pNewCmd)
{
    uint32_t requiredDelay;
    uint32_t cmdTime;
    bool scheduleResult = true;

    if(pPrevCmd->pClient->clientConfig.nID == pNewCmd->pClient->clientConfig.nID)
    {
        // same client, new command should come after the previous one
        scheduleResult = true;
    }
    else if (pNewCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
    {
        if(pPrevCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            /* duration will be added later when 15.4 stack puts all related commands into a chain command */
            //cmdTime = (uint32_t)RF_convertRatTicksToUs(pPrevCmd->duration) + (uint32_t)getSwitchingTimeInUs(pPrevCmd, pNewCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
            cmdTime = (uint32_t)getSwitchingTimeInUs(pPrevCmd, pNewCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
            scheduleResult = isGapOK(pPrevCmd->startTime, pNewCmd->startTime,cmdTime);

            if(false == scheduleResult)
            {
                if(pNewCmd->allowDelay ==0)
                {
                    // no delay
                    scheduleResult = false;
                }
                else
                {
                    requiredDelay = getRequiredDelay(pPrevCmd->startTime, pNewCmd->startTime,cmdTime);
                    if(pNewCmd->allowDelay > requiredDelay)
                    {
                        // delay is enough
                        scheduleResult = true;
                        pNewCmd->allowDelay -= requiredDelay;
                        pNewCmd->startTime += requiredDelay;
                    }
                    else
                    {
                        // delay is not enough
                        scheduleResult = false;
                    }
                }
            }
        }
        else if(pPrevCmd->pOp->startTrigger.triggerType == TRIG_NOW)
        {
            scheduleResult = true;
        }
    }
    else if(pNewCmd->pOp->startTrigger.triggerType == TRIG_NOW)
    {
        if(pPrevCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            /* Trig_Now will be added later at the end of the Q or available slot */
            scheduleResult = false;
        }
        else if(pPrevCmd->pOp->startTrigger.triggerType == TRIG_NOW)
        {
            /* Trig_Now after Trig_Now */
            scheduleResult = true;
        }
    }
    return(scheduleResult);
}

/* check cmd insert after prev cmd (general case) */
static bool cmdInsertCheckBeforeNextCmd(RF_Cmd* pNewCmd, RF_Cmd* pNextCmd)
{
    uint32_t cmdTime;
    bool scheduleResult = true;

    if(pNewCmd->pClient->clientConfig.nID == pNextCmd->pClient->clientConfig.nID)
    {
         /* from the same client, the new command should not be placed before the previous one */
        scheduleResult = false;
    }
    else if (pNewCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
    {
        if(pNextCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            /* duration will be added later when 15.4 stack puts all related commands into a chain command */
            //cmdTime = (uint32_t)RF_convertRatTicksToUs(pNewCmd->duration) + (uint32_t)getSwitchingTimeInUs(pNewCmd, pNextCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
            cmdTime = (uint32_t)getSwitchingTimeInUs(pNewCmd, pNextCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
            scheduleResult = isGapOK(pNewCmd->startTime, pNextCmd->startTime,cmdTime);

            /* cannot apply delay for the conflict with the next command */
        }
        else if(pNextCmd->pOp->startTrigger.triggerType == TRIG_NOW)
        {
            scheduleResult = false;
        }
    }
    else if(pNewCmd->pOp->startTrigger.triggerType == TRIG_NOW)
    {
        if(pNextCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            uint32_t currentTime = RF_getCurrentTime();
            /* duration will be added later when 15.4 stack puts all related commands into a chain command */
            //cmdTime = (uint32_t)RF_convertRatTicksToUs(pNewCmd->duration) + (uint32_t)getSwitchingTimeInUs(pNewCmd, pNextCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
            cmdTime = (uint32_t)getSwitchingTimeInUs(pNewCmd, pNextCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
            scheduleResult = isGapOK(currentTime, pNextCmd->startTime,cmdTime);
            /* cannot apply delay for the conflict with the next command */
        }
        else if(pNextCmd->pOp->startTrigger.triggerType == TRIG_NOW)
        {
            /* Trig_Now cannot be placed in front of existing Trig_Now */
            scheduleResult = false;
        }
    }
    return(scheduleResult);
}




static bool isClientMatched(RF_Handle h, RF_Cmd* pCmd)
{
    if (pCmd && (pCmd->pClient == h))
    {
        return (bool)true;
    }
    else
    {
        return (bool)false;
    }
}

#if !CONFLICT_FLUSH_ALL
/* return = true => no conflict, false => conflict */
static bool conflictCheckWithNextCmd(RF_Cmd* pNewCmd, RF_Cmd* pNextCmd)
{
    bool scheduleResult = true;
    uint32_t cmdTime;

    if(pNextCmd->pClient != pNewCmd->pClient)
    {
        if (pNewCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            if(pNextCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
            {
                /* duration will be added later when 15.4 stack puts all related commands into a chain command */
                //cmdTime = (uint32_t)RF_convertRatTicksToUs(pNewCmd->duration) + (uint32_t)getSwitchingTimeInUs(pNewCmd, pNextCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
                cmdTime = (uint32_t)getSwitchingTimeInUs(pNewCmd, pNextCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
                scheduleResult = isGapOK(pNewCmd->startTime, pNextCmd->startTime,cmdTime);
            }
        }
    }

    return(scheduleResult);
}

/* return = true => no conflict, false => conflict */
static bool conflictCheckWithPrevCmd(RF_Cmd* pPrevCmd, RF_Cmd* pNewCmd)
{
    bool   scheduleResult = true;
    uint32_t cmdTime;

    if(pPrevCmd->pClient != pNewCmd->pClient)
    {
        if (pNewCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
        {
            if(pPrevCmd->pOp->startTrigger.triggerType == TRIG_ABSTIME)
            {
                /* duration will be added later when 15.4 stack puts all related commands into a chain command */
                //cmdTime = (uint32_t)RF_convertRatTicksToUs(pPrevCmd->duration) + (uint32_t)getSwitchingTimeInUs(pPrevCmd, pNewCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
                cmdTime = (uint32_t)getSwitchingTimeInUs(pPrevCmd, pNewCmd) + MIN_NEW_CMD_START_TIME_TO_CURRENT_CMD_US;
                scheduleResult = isGapOK(pPrevCmd->startTime, pNewCmd->startTime,cmdTime);
            }
            /* conflict with Trig Now cmd is not considered */
        }
        /* Trig Now command won't come here, so no need to check */
    }
    return(scheduleResult);
}

#endif

static bool isSameClient(RF_Cmd* pCmdNew, RF_Cmd* pCompareCmd)
{
    if(pCmdNew->pClient->clientConfig.nID == pCompareCmd->pClient->clientConfig.nID)
        return (true);
    else
        return(false);
}

static bool isNewCmdHigherPriority(RF_Cmd* pCmdNew, RF_Cmd* pCompareCmd)
{
    uint8_t compCmdPri = DMMPolicy_getGlobalPriority(pCompareCmd->activityInfo, pCompareCmd->pClient->clientConfig.nID);
    uint8_t newCmdPri = DMMPolicy_getGlobalPriority(pCmdNew->activityInfo, pCmdNew->pClient->clientConfig.nID);

    if (newCmdPri == compCmdPri)
    {
        compCmdPri = DMMPolicy_getDefaultPriority(pCompareCmd->pClient->clientConfig.nID);
        newCmdPri = DMMPolicy_getDefaultPriority(pCmdNew->pClient->clientConfig.nID);
    }

    return((newCmdPri > compCmdPri ? true : false));
}

static bool dmmCmdInsertCheck(RF_Cmd* prevCmd, RF_Cmd* newCmd, RF_Cmd* nextCmd)
{
    bool    insertOK = false;

    if(prevCmd)
    {
        if(true == cmdInsertCheckAfterPrevCmd(prevCmd,newCmd))
        {
            if(nextCmd)
            {
                insertOK = cmdInsertCheckBeforeNextCmd(newCmd,nextCmd);
            }
            else
            {
                // no nextCmd
                insertOK = true;
            }
        }
    }
    else
    {
        if(nextCmd)
        {
            insertOK = cmdInsertCheckBeforeNextCmd(newCmd,nextCmd);
        }
        else
        {
            // no prev nor next cmd
            insertOK = true;
        }

    }
    return(insertOK);
}

#if !CONFLICT_FLUSH_ALL
static DMM_ConflictStatus dmmCmdConflictCheck(RF_Cmd* prevCmd, RF_Cmd* newCmd, RF_Cmd* nextCmd)
{
    DMM_ConflictStatus   conflictStatus=DMM_NoConflict;
    bool insertOKPrev = true;
    bool insertOKNext = true;

    if(prevCmd)
    {
        insertOKPrev = conflictCheckWithPrevCmd(prevCmd,newCmd);
    }
    if(nextCmd)
    {
        insertOKNext = conflictCheckWithNextCmd(newCmd,nextCmd);
    }

    if((false == insertOKPrev)&&(false == insertOKNext))
    {
        conflictStatus = DMM_ConfictWithBoth;
    }
    else if ((false == insertOKPrev)&&(true == insertOKNext))
    {
        conflictStatus = DMM_ConflictWithPrev;
    }
    else if ((true == insertOKPrev)&&(false == insertOKNext))
    {
        conflictStatus = DMM_ConflictWithNext;
    }
    else
    {
        conflictStatus = DMM_NoConflict;
    }

    return(conflictStatus);
}
#endif

