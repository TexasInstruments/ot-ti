/******************************************************************************

 @file dmm_thread_activity.c

 @brief DMM TIOP Activity Tracking implementation

 Group: CMCU, LPC
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2021-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
s
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

/**
 * @file dmm_thread_activity.c
 * @brief DMM TIOP Activity Tracking implementation
 * 
 * This file contains the implementation for tracking Thread activity
 * in the DMM (Dynamic Multi-protocol Manager) context.
 */

#include "dmm_thread_activity.h"

/**
 * @brief Activity tracking variables for Thread TX.
 * 
 * These variables are updated within ISR and MAC Task context.
 */
volatile threadTxActivityData_t txActivityData = {0};

/**
 * @brief Activity tracking variables for Thread RX.
 * 
 * These variables are updated within ISR and MAC Task context.
 */
volatile threadRxActivityData_t rxActivityData = {0};

/**
 * @brief Sets the Thread TX activity type.
 * 
 * @param aInstance Pointer to the OpenThread instance.
 * @param isPolling Indicates if the activity is polling.
 */
void dmmSetThreadActivityTx(otInstance *aInstance, bool isPolling)
{
    threadActivity_t activity = THREAD_ACTIVITY_TX_DATA;

    /* Direct/Indirect/Poll */
    if (isPolling)
    {
        activity = THREAD_ACTIVITY_TX_POLL;
    }

    txActivityData.activity = activity;
}

/**
 * @brief Sets the Thread RX activity type.
 * 
 * @param aInstance Pointer to the OpenThread instance.
 * @param isPolling Indicates if the activity is polling.
 */
void dmmSetThreadActivityRx(otInstance *aInstance, bool isPolling)
{
    threadActivity_t activity = THREAD_ACTIVITY_RX_IDLE;
    bool isEnergyScanActive = false, isActiveScanActive = false;

#ifndef OPENTHREAD_RADIO 
    isEnergyScanActive = otLinkIsEnergyScanInProgress(aInstance);
    isActiveScanActive = otLinkIsActiveScanInProgress(aInstance);
#endif

    if (isEnergyScanActive || isActiveScanActive)
    {
        activity = THREAD_ACTIVITY_RX_SCAN;
    }
    else if (isPolling)
    {
        activity = THREAD_ACTIVITY_RX_POLL;
    }
    rxActivityData.activity = activity;
}

/**
 * @brief Updates tracking information for TX activity.
 * 
 * @param success Indicates if the TX operation was successful.
 */
void dmmSetActivityTrackingTx(bool success)
{
    /* Update tracking information for activity */
    switch(txActivityData.activity)
    {
        case THREAD_ACTIVITY_TX_DATA:
            txActivityData.numMissedDataTxFrames = success ? 0 : (txActivityData.numMissedDataTxFrames + 1);
            break;
        case THREAD_ACTIVITY_TX_POLL:
            txActivityData.numMissedPollDataTxFrames = success ? 0 : (txActivityData.numMissedPollDataTxFrames + 1);
            break;
        default:
            break;
    }
}

/**
 * @brief Updates tracking information for RX activity.
 * 
 * @param success Indicates if the RX operation was successful.
 */
void dmmSetActivityTrackingRx(bool success)
{
    /* Update tracking information for activity */
    switch(rxActivityData.activity)
    {
        case THREAD_ACTIVITY_RX_POLL:
            rxActivityData.numRxPollAbort = success ? 0 : (rxActivityData.numRxPollAbort + 1);
            break;
        case THREAD_ACTIVITY_RX_SCAN:
            rxActivityData.numRxScanAbort = success ? 0 : (rxActivityData.numRxScanAbort + 1);
            break;
        case THREAD_ACTIVITY_RX_IDLE:
        {
            /* Poll activity is only active for a short period following an incoming ACK. Reset abort count */
            if (success)
            {
                rxActivityData.numRxPollAbort = 0;
                rxActivityData.numRxIdleAbort = 0;
            }
            else
            {
                rxActivityData.numRxIdleAbort++;
            }
        }

            break;
        default:
            break;
    }
}

/**
 * @brief Gets the priority of the current RX activity.
 * 
 * @return The priority value for the RX activity.
 */
uint32_t dmmGetActivityPriorityRx(void)
{
    uint32_t priority;

    switch(rxActivityData.activity)
    {
        case THREAD_ACTIVITY_RX_POLL:
        {
            /* Default Priority for polled data is HIGH */
            if (rxActivityData.numRxPollAbort < THREAD_ACTIVITY_THRESHOLD_PRI_HIGH)
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_POLL, THREAD_ACTIVITY_PRI_HIGH_INDEX);
            }
            else
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_POLL, THREAD_ACTIVITY_PRI_URGENT_INDEX);
            }
        }
        break;

        case THREAD_ACTIVITY_RX_SCAN:
        {
            if (rxActivityData.numRxScanAbort < THREAD_ACTIVITY_THRESHOLD_PRI_NORMAL)
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_SCAN, THREAD_ACTIVITY_PRI_NORMAL_INDEX);
            }
            else if (rxActivityData.numRxScanAbort < THREAD_ACTIVITY_THRESHOLD_PRI_HIGH)
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_SCAN, THREAD_ACTIVITY_PRI_HIGH_INDEX);
            }
            else
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_SCAN, THREAD_ACTIVITY_PRI_URGENT_INDEX);
            }
        }
        break;
        case THREAD_ACTIVITY_RX_IDLE:
        {
           if (rxActivityData.numRxIdleAbort > THREAD_ACTIVITY_THRESHOLD_PRI_URGENT)
           {
               priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_IDLE, THREAD_ACTIVITY_PRI_URGENT_INDEX);
           }
           else if (rxActivityData.numRxIdleAbort > THREAD_ACTIVITY_THRESHOLD_PRI_HIGH)
           {
               priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_IDLE, THREAD_ACTIVITY_PRI_HIGH_INDEX);
           }
           else
           {
               priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_IDLE, THREAD_ACTIVITY_PRI_NORMAL_INDEX);
           }
        }
        break;
        default:
            priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_RX_IDLE, THREAD_ACTIVITY_PRI_NORMAL_INDEX);
        break;

    }

    return (priority);
}

/**
 * @brief Gets the priority of the current TX activity.
 * 
 * @return The priority value for the TX activity.
 */
uint32_t dmmGetActivityPriorityTx(void)
{
    uint32_t priority;

    switch(txActivityData.activity)
    {
        case THREAD_ACTIVITY_TX_DATA:
        {
            /* Default Priority for polled data is NORMAL */
            if (txActivityData.numMissedDataTxFrames < THREAD_ACTIVITY_THRESHOLD_PRI_NORMAL)
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_TX_DATA, THREAD_ACTIVITY_PRI_NORMAL_INDEX);
            }
            else if (txActivityData.numMissedDataTxFrames < THREAD_ACTIVITY_THRESHOLD_PRI_HIGH)
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_TX_DATA, THREAD_ACTIVITY_PRI_HIGH_INDEX);
            }
            else
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_TX_DATA, THREAD_ACTIVITY_PRI_URGENT_INDEX);
            }
        }
        break;

        case THREAD_ACTIVITY_TX_POLL:
        {
            /* Default Priority for polled data is HIGH */
            if (txActivityData.numMissedPollDataTxFrames < THREAD_ACTIVITY_THRESHOLD_PRI_HIGH)
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_TX_POLL, THREAD_ACTIVITY_PRI_HIGH_INDEX);
            }
            else
            {
                priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_TX_POLL, THREAD_ACTIVITY_PRI_URGENT_INDEX);
            }
        }
        break;

        default:
            priority = CALC_ACTIVITY_PRIORITY(THREAD_ACTIVITY_TX_DATA, THREAD_ACTIVITY_PRI_NORMAL_INDEX);
        break;

    }

    return (priority);
}