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
#include <region_settings.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <utils/code_utils.h>
#include <utils/link_metrics.h>
#include <utils/mac_frame.h>
#include <openthread/config.h>
#include <openthread/diag.h>
#include <openthread/link.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/entropy.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>

#include <ti/devices/DeviceFamily.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/ieee.h>
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_lrfddbell.h)

#include "radio.h"
#include "system.h"
#include <ti_drivers_config.h>
#include <ti_radio_config.h>

#include <ti/log/Log.h>
#ifdef ti_log_Log_ENABLE
#include "ti_log_config.h"
#endif

/* General Definitions */
#define PLATFORM_RADIO_RECEIVER_SENSITIVITY_DBM -90 /* dBm */

/* Words sent by the TX test command in modulated/unmodulated mode
 * 0xAAAA = 0b10101010101010101010
 * 0xFFFF = 0b11111111111111111111
 */
#define PLATFORM_RADIO_TX_TEST_MODULATED_WORD 0xAAAA
#define PLATFORM_RADIO_TX_TEST_UNMODULATED_WORD 0xFFFF

/* Receive Buffer options */
#define MAX_NUM_RX_PKTS (4) // Max number of packets to be held per Receive buffer (2)

/* Transmit Buffer options */
#define NUM_PAD 0
#define HDR_LEN 1
#define MAX_PKT_LEN 125
#define MAX_APPENDED_BYTES 9
#define NUM_CRC_BYTES 2
#define EDSCAN_REQ (1)
#define NO_EDSCAN_REQ (0)
/* Issue Tx/Rx commands with timing information only converted to RCL ticks */
#define USE_ABS_SCHED_TIMING (true)
/* Issue Tx/Rx command with timing information that will be normalized to otPlatRadioGetNow() time */
#define USE_REL_SCHED_TIMING (false)
/* Helper Macro for calulating index to bitfield in source matching table */
#define BIT_INDEX(INDEX) (1 << INDEX)
#define RCL_TICKS_PER_US (4)

/*!
\brief Converts a duration given in \a microseconds into radio timer (RCL_TICKS_PER_US) ticks.
*/
#define convertUsToRCLTicks(microseconds) \
    ((microseconds) * (RCL_TICKS_PER_US))
/*!
\brief Converts a duration given in \a milliseconds into radio timer (RCL_TICKS_PER_US) ticks.
*/
#define convertMsToRCLTicks(milliseconds) \
    ((milliseconds) * 1000 * (RCL_TICKS_PER_US))

/*!
\brief Converts a duration given in radio timer (RCL_TICKS_PER_US) \a ticks into microseconds.
*/
#define convertRCLTicksToUs(ticks) \
    ((ticks) / (RCL_TICKS_PER_US))

#define convertMsToUs(ms) (1000 * ms)
#define NO_RADIO_EVTS (0U)

/* Structures start */
struct rfPktAdditionalInfo
{
    uint8_t  status;
    uint8_t  lqi;
    uint8_t  rssi;
    uint32_t timestamp;
    uint8_t  endPad;
} __attribute__((__packed__));

/* Control flag to disable channel switching by the stack. This is used by
 * `rfCoreDiagChannelDisable` and `rfCoreDiagChannelEnable`.
 */
static bool sDisableChannel = false;

/**
 * Structure to store the maximum power of a given channel found from
 * characterization of the radio.
 */
struct tx_power_max
{
    uint8_t channel;  /**< Channel in IEEE Page 0 */
    int8_t  maxPower; /**< Maximum power for the Channel */
};
/* Structures End */
/* Globals start */
static RCL_Client rclClient;
static RCL_Handle sRclHandle;

static uint32_t rxBuffer1[RCL_MultiBuffer_len_u32((MAX_PKT_LEN + MAX_APPENDED_BYTES) * MAX_NUM_RX_PKTS)];
static uint32_t rxBuffer2[RCL_MultiBuffer_len_u32((MAX_PKT_LEN + MAX_APPENDED_BYTES) * MAX_NUM_RX_PKTS)];

static uint32_t rxActionBuf[RCL_Buffer_bytesToWords(sizeof(RCL_CmdIeee_RxAction)) +
                            RCL_Buffer_bytesToWords(sizeof(RCL_CmdIeee_PanConfig))];

#if OPENTHREAD_CONFIG_DIAG_ENABLE
/* Tx Test command setup */
RCL_CmdIeeeTxTest ieeeTxTestCmd;
#endif
/* Rx command setup */
static RCL_CmdIeee_RxAction *rxAction = (RCL_CmdIeee_RxAction *)&rxActionBuf;
static RCL_StatsIeee         ieeeRxTxStats;
static RCL_CommandStatus     rxStatus;
static RCL_CmdIeeeRxTx       ieeeRxTxCmd;
static RCL_StatsIeee         ieeeRxTxStats;

/* Tx command setup */
static RCL_CmdIeeeRxTx      ieeeTxCmd;
static RCL_CmdIeee_TxAction txAction;
static RCL_StatsIeee        ieeeTxStats;

/* True if OT Frame is pending transmission due to RX->TX Ack interruption*/
static bool sTransmitPending;

/* Non-Null if Active OT Frame present */
static otRadioFrame  sTransmitFrame;
static otRadioIeInfo sTransmitFrameIeInfo;
/* Transmit command RCL->OT error mapping */
static otError sTransmitError;

/* RCL Tx packet buffer */
#define MAX_TX_PAYLOAD_LEN (MAX_PKT_LEN + MAX_APPENDED_BYTES)
static uint32_t txPktBuffer[RCL_TxBuffer_len_u32(NUM_PAD, HDR_LEN, MAX_PKT_LEN + MAX_APPENDED_BYTES)];

/* Storage for Openthread Package generation */
static uint8_t sTransmitPsdu[MAX_TX_PAYLOAD_LEN];

/* Source Matching setup */
static uint32_t srcMatchTableBuffer[(offsetof(RCL_CmdIeee_SourceMatchingTableShort, shortEntry) / sizeof(uint32_t)) +
                                    PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM];
static RCL_CmdIeee_SourceMatchingTableShort *srcMatchTableShort =
    (RCL_CmdIeee_SourceMatchingTableShort *)&srcMatchTableBuffer;

/* This is the channel of the last successfully submitted Tx/Rx command */
static uint8_t                         sChannel;
static volatile platformRadio_phyState sState;
static volatile platformRadio_txState  sTxState;
/**
 * Value requested in dBm from the upper layers on the last call to
 * @ref otPlatRadioSetTransmitPower.
 */
static int8_t sReqTxPower = 0U;

/**
 * Array of back-off values necessary for passing FCC testing.
 */
static const struct tx_power_max cTxMaxPower[] = {
    /* IAR: Error[Pe1345]: an empty initializer is invalid for an array with unspecified bound
     * GCC: error: comparison of unsigned expression in '< 0' is always false [-Werror=type-limits]
     *
     * Adding an extra value with max theoretical channel and power numbers to
     * enable IAR and pedantic C warnings to build with this array. If the
     * device does not have regulatory back-offs this is a minor cost of
     * code and data.
     */
    {.channel = UINT8_MAX, .maxPower = UINT8_MAX},

};

static uint16_t sCurrentRegionCode = 0;

/* Transmit security keying material */
static uint8_t          sKeyId;
static uint8_t          sAckKeyId;
static otMacKeyMaterial sPrevKey;
static otMacKeyMaterial sCurrKey;
static otMacKeyMaterial sNextKey;
static uint32_t         sMacFrameCounter;
static uint32_t         sAckFrameCounter;

bool rclRxActive = false;
/* Globals End */
/* Function Prototypes */
static void              rclRxTxCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents);
static RCL_CommandStatus rclSendReceiveCmd(otInstance *aInstance,
                                    uint8_t     aChannel,
                                    uint32_t    aStart,
                                    uint32_t    aDuration,
                                    uint8_t     edScanReq,
                                    uint8_t absStartReq);

static RCL_CommandStatus rclSendTransmitCmd(otInstance *aInstance, otRadioFrame *aFrame, uint8_t endRxWhenDone, uint8_t absStartReq);
#if OPENTHREAD_CONFIG_DIAG_ENABLE
static RCL_CommandStatus rclSendTxTestCmd(bool aModulated);
#endif
static void              rclStopTransmitCmd(void);
static void              rclStopReceiveCmd(void);
static void    handleRxDataFinish(otInstance *aInstance, unsigned int aEvents, RCL_Buffer_DataEntry *currEntry);
static otError populateReceiveFrame(otRadioFrame *aFrame, RCL_Buffer_DataEntry *currEntry);
static void    processRxQueue(otInstance *aInstance, unsigned int aEvents);
static void platformRadioProcessReceiveDone(otInstance *aInstance, otRadioFrame *aReceiveFrame, otError aReceiveError);
static void platformRadioProcessTransmitDone(otInstance   *aInstance,
                                             otRadioFrame *aTransmitFrame,
                                             otRadioFrame *aAckFrame,
                                             otError       aTransmitError);
static uint64_t normalizeRCLTimestamp(uint32_t timestamp);
static uint32_t normalizeUsTimestamp(uint64_t timestamp);
/* Source Matching */
static void    rclToggleReceiveCmd(otInstance *aInstance);
static uint8_t rclFindShortSrcMatchIdx(uint16_t shortAddr);
static uint8_t rclFindEmptyShortSrcMatchIdx(void);

/* Function Prototypes End */

/* Stop Combined or Standalone Tx command on radio */
void rclStopTransmitCmd(void)
{
    /* Attempt to stop any Tx command (Either standalone or combined) */
    if (sTxState == platformRadio_txState_StandaloneActive)
    {
        /* Case 1 - Transmit was running without receive command */
        RCL_Command_stop(&ieeeTxCmd, RCL_StopType_Hard);
    }
    else if (sTxState == platformRadio_txState_CombinedActive)
    {
        /* Case 2 - Transmit was running with receive command */
        RCL_IEEE_Tx_stop(&ieeeRxTxCmd, RCL_StopType_Hard);
    }
}
/* Stop Rx command on radio */
void rclStopReceiveCmd(void)
{
    Log_printf(LogModule_Thread, Log_WARNING, "Warning, Calling Rx Stop");

    RCL_Command_stop(&ieeeRxTxCmd, RCL_StopType_Graceful);
}

/**
 * @brief Post a Radio Signal
 *
 * Some Radio event has occurred, wake the process loops.
 *
 * @param [in] evts Events to post
 */
static void radioSignal(unsigned int evts)
{
    if (evts != 0U)
    {
        platformRadioSignal(evts);
    }
}

/**
 * Callback for the TX OR Rx command chain.
 *
 * This callback is called on completion of the command string or failure of
 * an individual command in the string.
 *
 * @param [in] cmd RCL Command handle
 * @param [in] lrfEvents LRF Events posted during command execution
 * @param [in] rclEvents RCL Events posted during command execution
 */
void rclRxTxCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    unsigned int evts = NO_RADIO_EVTS;

    Log_printf(LogModule_Thread, Log_VERBOSE, "RF Callback: Callback entry point for command: %d", cmd->cmdId);

    if (cmd->cmdId == RCL_CMDID_IEEE_RX_TX)
    {
        /* Tx handling */
        if ((((RCL_CmdIeeeRxTx *)cmd)->txAction != NULL) && ((RCL_CmdIeeeRxTx *)cmd)->txAction->txEntry != NULL)
        {
            uint8_t txDone = false;
            Log_printf(LogModule_Thread, Log_VERBOSE, "RF Callback: Tx Action Status %d RCL Events: %d LRF Events: %d",
                       txAction.txStatus, rclEvents.value, lrfEvents.value);

            if ((rclEvents.cmdStepDone) && (((RCL_CmdIeeeRxTx *)cmd)->txAction->txStatus == RCL_CommandStatus_Finished))
            {
                sTransmitError = OT_ERROR_NONE;
                txDone         = true;
            }
            else if ((rclEvents.cmdStepDone &&
                      ((RCL_CmdIeeeRxTx *)cmd)->txAction->txStatus == RCL_CommandStatus_ChannelBusy) ||
                     (rclEvents.cmdStepDone &&
                         ((RCL_CmdIeeeRxTx *)cmd)->txAction->txStatus == RCL_CommandStatus_Error_TxBufferCorruption))
            {
                /* CCA Failure/buffer error */
                sTransmitError = OT_ERROR_CHANNEL_ACCESS_FAILURE;
                txDone         = true;
            }
            else if ((((RCL_CmdIeeeRxTx *)cmd)->txAction->txStatus == RCL_CommandStatus_NoSync) ||
                     (((RCL_CmdIeeeRxTx *)cmd)->txAction->txStatus == RCL_CommandStatus_RxErr))
            {
                /* ACK not received */
                sTransmitError = OT_ERROR_NO_ACK;
                txDone         = true;
            }
            else if (RCL_CommandStatus_isAnyStop(((RCL_CmdIeeeRxTx *)cmd)->txAction->txStatus))
            {
                /* Tx command aborted, general failure of the command string, notify processing loop.*/
                sTransmitError = OT_ERROR_ABORT;
                txDone         = true;
            }
            else
            {
                /* Handle Tx Preemption/unsupported errors */
                Log_printf(LogModule_Thread, Log_WARNING,
                           "RF Callback: Warning Unknown Tx Action Status %d RCL Events: %d LRF Events: %d",
                           txAction.txStatus, rclEvents.value, lrfEvents.value);
            }

            if (txDone)
            {
                sTxState = platformRadio_txState_Inactive;
                evts |= RF_EVENT_TX_DONE;
            }
        }

        Log_printf(LogModule_Thread, Log_VERBOSE, "RF Callback: Common Status %d RCL Events: %d LRF Events: %d",
                   ((RCL_CmdIeeeRxTx *)cmd)->common.status, rclEvents.value, lrfEvents.value);

        /* Rx handling */
        if (((RCL_CmdIeeeRxTx *)cmd)->rxAction != NULL)
        {
            if (lrfEvents.txAck)
            {
                Log_printf(LogModule_Thread, Log_VERBOSE, "Packet Rx, Ack Tx Success");
            }

            if (lrfEvents.rxBufFull)
            {
                Log_printf(LogModule_Thread, Log_VERBOSE, "RCL Callback: Rx Buffer Full");
                evts |= RF_EVENT_BUF_FULL;
            }

            if (rclEvents.rxEntryAvail)
            {
                Log_printf(LogModule_Thread, Log_VERBOSE, "RCL Callback: Rx Done");
                evts |= RF_EVENT_RX_DONE;
            }

            /* Combined Rx/Tx command was stopped */
            if (rclEvents.lastCmdDone || (RCL_CommandStatus_isAnyStop(((RCL_CmdIeeeRxTx *)cmd)->common.status)) ||
                (((RCL_CmdIeeeRxTx *)cmd)->common.status == RCL_CommandStatus_Error_RxFifo))
            {
                rclRxActive = false;

                Log_printf(LogModule_Thread, Log_ERROR, "RCL Callback: Rx Stop Requested/Rx failure occurred");

                if (((RCL_CmdIeeeRxTx *)cmd)->rxAction->disableSync)
                {
                    Log_printf(LogModule_Thread, Log_DEBUG, "RCL Callback: ED Scan Complete");

                    evts |= RF_EVENT_ED_SCAN_DONE;
                }
                evts |= RF_EVENT_RX_CMD_STOP;
            }
            /* General Error occurred for Rx */
            if (((RCL_CmdIeeeRxTx *)cmd)->common.status >= RCL_CommandStatus_Error)
            {
                rclRxActive = false;
                evts |= RF_EVENT_RX_CMD_ERROR;
            }
        }
    }

    /* tell radio processing loop what happened */
    radioSignal(evts);
}
#if OPENTHREAD_CONFIG_DIAG_ENABLE
/**
 * Callback for the TX Test coommand
 *
 * @param [in] cmd RCL Command handle
 * @param [in] lrfEvents LRF Events posted during command execution
 * @param [in] rclEvents RCL Events posted during command execution
 */
void rclTxTestCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    Log_printf(LogModule_Thread, Log_VERBOSE, "RF Callback: Tx Test Status %d RCL Events: %d LRF Events: %d",
                ((RCL_CmdIeeeTxTest *)cmd)->common.status, rclEvents.value, lrfEvents.value);

}
#endif
/**
 * Empties the rx queue, regardless of the current state of the entries.
 */
static void clearRxQueue(void)
{
    Log_printf(LogModule_Thread, Log_VERBOSE, "clearRxQueue");

    /* Clear entire multi buf */
    RCL_MultiBuffer_clear(RCL_MultiBuffer_head(&rxAction->rxBuffers));
}

/**
 * Scan through the RX queue, looking for completed entries.
 */
static void processRxQueue(otInstance *aInstance, unsigned int aEvents)
{
    RCL_Buffer_DataEntry *currEntry;           /* Locally received packet entry */
    RCL_MultiBuffer      *finishedMultiBuffer; /* Finished multi-buffer entry */
    List_List             finishedBuffers;     /* Singular completed buffer that was read */

    RCL_MultiBuffer *currBuffer;
    uint32_t         availableBytes;

    /* Prepare list of RX buffers that are done */
    List_clearList(&finishedBuffers);
    /* Search through all available packets */
    do
    {
        currEntry = RCL_MultiBuffer_RxEntry_get(&rxAction->rxBuffers, &finishedBuffers);
        if (currEntry != NULL)
        {
            handleRxDataFinish(aInstance, aEvents, currEntry);
        }

        currBuffer = RCL_MultiBuffer_head(&rxAction->rxBuffers);
        if (NULL != currBuffer)
        {
            availableBytes = RCL_MultiBuffer_findAvailableRxSpace(currBuffer);
            (void)availableBytes;
            Log_printf(LogModule_Thread, Log_VERBOSE, "processRxQueue, avail space: %d", availableBytes);
        }

        /* Make finished buffers available to RCL command */
        while ((finishedMultiBuffer = RCL_MultiBuffer_get(&finishedBuffers)) != NULL)
        {
            RCL_MultiBuffer_clear(finishedMultiBuffer);
            RCL_MultiBuffer_put(&rxAction->rxBuffers, finishedMultiBuffer);
        }
    } while ((!RCL_MultiBuffer_RxEntry_isEmpty(&rxAction->rxBuffers)));
}

static otError populateReceiveFrame(otRadioFrame *aFrame, RCL_Buffer_DataEntry *currEntry)
{
    uint8_t                     infoId    = currEntry->length - sizeof(struct rfPktAdditionalInfo) - 1;
    struct rfPktAdditionalInfo *extraInfo;
    extraInfo    = (struct rfPktAdditionalInfo *)&(currEntry->data[infoId]);
    bool    pend = false;
    uint8_t idx;

    /* Get SFD ts from the packet */
    aFrame->mInfo.mRxInfo.mTimestamp = normalizeRCLTimestamp(extraInfo->timestamp);

    aFrame->mLength             = currEntry->pad0 & 0x7F; /* length of rx frame (phy hdr) */
    aFrame->mPsdu               = currEntry->data;        /*  start of psdu */
    aFrame->mChannel            = sChannel;
    aFrame->mInfo.mRxInfo.mRssi = extraInfo->rssi;
    aFrame->mInfo.mRxInfo.mLqi  = extraInfo->lqi;

    /* Find source addressing mode for packet to determine pend bit state in ACK frame sent */
    otMacAddress aMacAddress;
    otError      srcError = otMacFrameGetSrcAddr(aFrame, &aMacAddress);

    if (OT_ERROR_NONE == srcError)
    {
        if (aMacAddress.mType == OT_MAC_ADDRESS_TYPE_SHORT)
        {
            idx = rclFindShortSrcMatchIdx(aMacAddress.mAddress.mShortAddress);
            if (PLATFORM_RADIO_SRC_MATCH_NONE != idx)
            {
                pend = srcMatchTableShort->framePending[idx / 16] & BIT_INDEX(idx & 0x0F);
            }
            Log_printf(LogModule_Thread, Log_VERBOSE, "populateReceiveFrame: MAC Address Short: %d ID: %d Pend: %d",
                       aMacAddress.mAddress.mShortAddress, idx, pend);
        }
        else if (aMacAddress.mType == OT_MAC_ADDRESS_TYPE_EXTENDED)
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "populateReceiveFrame: MAC Address Extended found");

            for (int i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
            {
                Log_printf(LogModule_Thread, Log_VERBOSE, "Addresss[%d]: %d", i, aMacAddress.mAddress.mExtAddress.m8[i]);
            }
            /* Extended address source matching not yet supported, set to defaultpend */
            pend = rxAction->panConfig[0].defaultPend;
        }
        else
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "populateReceiveFrame: No MAC Address");
        }
    }
    Log_printf(LogModule_Thread, Log_VERBOSE, "populateReceiveFrame: MAC source address Address Error: %d", srcError);

    aFrame->mInfo.mRxInfo.mAckedWithFramePending = pend;

    Log_printf(LogModule_Thread, Log_VERBOSE, "populateReceiveFrame: Packet Length: %d Frame pending: %d", currEntry->pad0,
               pend);

    return OT_ERROR_NONE;
}

/**
 * Clear Transmit data from RCL Command structure to prevent duplicate packets being sent if a combined Rx is
 * rescheduled.
 */
void clearTransmitState(void)
{
    txAction.txEntry     = NULL;
    ieeeRxTxCmd.txAction = NULL;
}
/**
 * An RX queue entry is in the finished state, process it.
 */
static void handleRxDataFinish(otInstance *aInstance, unsigned int aEvents, RCL_Buffer_DataEntry *currEntry)
{
    otError      error;
    otRadioFrame receiveFrame = {0};

    error = populateReceiveFrame(&receiveFrame, currEntry);
    if (OT_ERROR_NONE != error)
    {
        Log_printf(LogModule_Thread, Log_VERBOSE, "populateReceiveFrame: Error %d", error);

        /* Indicate a receive error to the upper layers */
        platformRadioProcessReceiveDone(aInstance, &receiveFrame, error);
        return;
    }

    /* Is this an ACK frame? */
    if (otMacFrameIsAck(&receiveFrame))
    {
        Log_printf(LogModule_Thread, Log_VERBOSE,
                   "populateReceiveFrame: ACK received State: %d Sequence Number Exp: %d Actual Seq Number: %d", sState,
                   otMacFrameGetSequence(&receiveFrame), otMacFrameGetSequence(&sTransmitFrame));

        if (((platformRadio_phyState_TransmitStandalone == sState) || (platformRadio_phyState_Transmit == sState)) &&
            otMacFrameIsAckRequested(&sTransmitFrame) &&
            otMacFrameGetSequence(&receiveFrame) == otMacFrameGetSequence(&sTransmitFrame))
        {
            /*  No previous Rx was running */
            if (platformRadio_phyState_TransmitStandalone == sState)
            {
                sState = platformRadio_phyState_Sleep;
            }
            else
            {
                sState = platformRadio_phyState_Receive;
            }

            platformRadioProcessTransmitDone(aInstance, &sTransmitFrame, &receiveFrame, error);
        }
        return;
    }

    if (otMacFrameIsAckRequested(&receiveFrame))
    {
#ifdef ENH_ACK_SUPPORT
        // ack requested
        if (0 != (aEvents & RF_EVENT_RX_ACK_DONE))
        {
            /* Enhanced Ack has been sent */
            if (otMacFrameIsVersion2015(&receiveFrame))
            {
                error = sTransmitError;
            }

            platformRadioProcessReceiveDone(aInstance, &receiveFrame, error);

            curEntry->status = DATA_ENTRY_PENDING;
        }
#endif
        platformRadioProcessReceiveDone(aInstance, &receiveFrame, OT_ERROR_NONE);
    }
    else
    {
        platformRadioProcessReceiveDone(aInstance, &receiveFrame, OT_ERROR_NONE);
    }
}

/**
 * @brief Initialize base RCL parameters for Tx and Rx operation
 */
void rclInitCmdParams(void)
{
    /* Initialize to defaults */
    ieeeRxTxStats = RCL_StatsIeee_DefaultRuntime();
    ieeeRxTxCmd   = RCL_CmdIeeeRxTx_DefaultRuntime();
    ieeeTxCmd     = RCL_CmdIeeeRxTx_DefaultRuntime();

    txAction  = RCL_CmdIeee_TxAction_DefaultRuntime();
    *rxAction = RCL_CmdIeee_RxAction_DefaultRuntime();

    txAction.txEntry             = NULL;
    txAction.ccaMode             = RCL_CmdIeee_CcaMode1Energy;
    txAction.ccaCorrThresh       = 2;
    txAction.ccaContentionWindow = 1;
    txAction.rssiLimit           = PLATFORM_RADIO_RECEIVER_SENSITIVITY_DBM;

    ieeeRxTxStats.config.activeUpdate = 1;

    rxAction->panConfig[0]                 = RCL_CmdIeee_PanConfig_DefaultRuntime();
    rxAction->panConfig[0].panCoord        = 1;
    rxAction->panConfig[0].localPanId      = 0x0;
    rxAction->panConfig[0].localShortAddr  = 0x00;
    rxAction->panConfig[0].autoAckMode     = RCL_CmdIeee_AutoAck_ImmAckAutoPendAll;
    rxAction->panConfig[0].defaultPend     = 1;
    rxAction->panConfig[0].maxFrameVersion = 1;
    rxAction->numPan                       = 1;

    /* Source Matching */
    rxAction->panConfig[0].sourceMatchingTableShort             = srcMatchTableShort;
    rxAction->panConfig[0].sourceMatchingTableShort->numEntries = PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM;

    /* Buffer handling */
    RCL_MultiBuffer *multiBuffer;

    multiBuffer = (RCL_MultiBuffer *)rxBuffer1;
    RCL_MultiBuffer_init(multiBuffer, sizeof(rxBuffer1));
    RCL_MultiBuffer_put(&rxAction->rxBuffers, multiBuffer);

    multiBuffer = (RCL_MultiBuffer *)rxBuffer2;
    RCL_MultiBuffer_init(multiBuffer, sizeof(rxBuffer2));
    RCL_MultiBuffer_put(&rxAction->rxBuffers, multiBuffer);

    ieeeRxTxCmd.common.runtime.callback              = rclRxTxCallback;
    ieeeRxTxCmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value | RCL_EventRxEntryAvail.value |
                                                       RCL_EventCmdStarted.value | RCL_EventTxBufferFinished.value |
                                                       RCL_EventCmdStepDone.value | RCL_EventGracefulStop.value | RCL_EventHardStop.value;
;

    ieeeRxTxCmd.common.runtime.lrfCallbackMask.value =
        LRF_EventRxBufFull.value | LRF_EventTxDone.value | LRF_EventTxAck.value;

    sChannel                = OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MIN;
    ieeeRxTxCmd.rfFrequency = RCL_CMD_IEEE_CHANNEL_FREQUENCY(sChannel);
    ieeeTxCmd.rfFrequency   = RCL_CMD_IEEE_CHANNEL_FREQUENCY(sChannel);
}

/**
 * @brief Setup Region information and initialize radio parameters
 */
void platformRadioInit(void)
{
    rclInitCmdParams();

    sCurrentRegionCode = CC_UINT16('W', 'W');

    sTransmitFrame.mPsdu                 = sTransmitPsdu;
    sTransmitFrame.mLength               = 0;
    sTransmitFrame.mInfo.mTxInfo.mIeInfo = &sTransmitFrameIeInfo;
    sTransmitPending                     = false;

    sState = platformRadio_phyState_Disabled;

    Log_printf(LogModule_Thread, Log_VERBOSE, "platformRadioInit complete");
}

/**
 * @brief Enable Radio operation
 */
otError otPlatRadioEnable(otInstance *aInstance)
{
    otError error = OT_ERROR_BUSY;
    (void)aInstance;

    if (sState == platformRadio_phyState_Sleep)
    {
        error = OT_ERROR_NONE;
    }
    else if (sState == platformRadio_phyState_Disabled)
    {
        RCL_init();
        rclInitCmdParams();

        sRclHandle = RCL_open(&rclClient, &LRF_config_ieee_802_15_4_0);

        otEXPECT_ACTION(sRclHandle != NULL, error = OT_ERROR_FAILED);
        sState = platformRadio_phyState_Sleep;

        error = OT_ERROR_NONE;
    }

exit:
    if (error == OT_ERROR_FAILED)
    {
        sState = platformRadio_phyState_Disabled;
    }

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioEnable: Error: %d", error);
    return error;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;
    return (sState != platformRadio_phyState_Disabled);
}

/**
 * @brief Disable the radio, if active, block until all commands have completed
 */
otError otPlatRadioDisable(otInstance *aInstance)
{
    otError error = OT_ERROR_BUSY;
    (void)aInstance;

    if (sState == platformRadio_phyState_Disabled)
    {
        error = OT_ERROR_NONE;
    }
    else if (sState == platformRadio_phyState_Sleep)
    {
        /* Wait for any pending commands to finish first */
        rclStopTransmitCmd();
        rclStopReceiveCmd();

        if (ieeeRxTxCmd.common.status > RCL_CommandStatus_Idle)
        {
            RCL_Command_pend(&ieeeRxTxCmd);
        }

        RCL_close(sRclHandle);
        sRclHandle = NULL;

        sState = platformRadio_phyState_Disabled;
        error  = OT_ERROR_NONE;
    }

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioDisable: Error: %d", error);

    return error;
}

/**
 * @brief Get the receive command's sensitivity when performing CCA.
 */
int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    (void)aInstance;

    return (txAction.rssiLimit);
}

/**
 * @brief Get the CCA RSSI threshold
 */
otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError err = OT_ERROR_NONE;

    if (aThreshold)
    {
        *aThreshold = txAction.rssiLimit;
    }
    else
    {
        err = OT_ERROR_INVALID_ARGS;
    }

    return (err);
}

/**
 * @brief Set the CCA RSSI threshold
 */
otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    txAction.rssiLimit = aThreshold;

    return (OT_ERROR_NONE);
}

/**
 * @brief Sends the energy detect scan command to the radio core
 *
 * Sends the Energy Detect scan command to the radio core. This scans the given
 * channel for activity.
 *
 * @param [in] aRfHandle The handle for the RF core client
 * @param [in] aChannel  The IEEE page 0 channel to scan
 * @param [in] aDuration Time in ms to scan
 *
 * @return RCL_CommandStatus of the running command returned by the command scheduler
 */
otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;
    RCL_CommandStatus edScanStatus;

    switch (sState)
    {
        case platformRadio_phyState_Receive:
        {
            sState = platformRadio_phyState_EdScan;
            /* abort receive */
            rclStopReceiveCmd();
            break;
        }

        case platformRadio_phyState_Sleep:
        {
            sState = platformRadio_phyState_EdScanStandalone;
            break;
        }
        default:
        {
            error = OT_ERROR_BUSY;
            break;
        }
    }
    /* rclSendReceiveCmd takes duration and start time in uS */
    edScanStatus = rclSendReceiveCmd(aInstance, aScanChannel, 0, convertMsToUs(aScanDuration), EDSCAN_REQ, USE_ABS_SCHED_TIMING);
    otEXPECT_ACTION((!RCL_CommandStatus_isAnyDescheduled(edScanStatus) && edScanStatus < RCL_CommandStatus_Error),
                    error = OT_ERROR_FAILED);

exit:
    if (OT_ERROR_NONE != error)
    {
        sState = (sState == platformRadio_phyState_EdScanStandalone) ? platformRadio_phyState_Sleep
                                                                     : platformRadio_phyState_Receive;
    }

    Log_printf(LogModule_Thread, Log_DEBUG, "otPlatRadioEnergyScan aScanChannel: %d aDuration: %d",
               aScanChannel, aScanDuration);
    Log_printf(LogModule_Thread, Log_DEBUG, "otPlatRadioEnergyScan New State: %d Status: %d", sState, edScanStatus);

    return error;
}

/**
 * @brief Sets the transmit.
 *
 * Sets the transmit power within the radio setup command or the override list.
 */
static otError rclSetTransmitPower(int8_t aPower)
{
    otError                retval = OT_ERROR_NONE;
    LRF_TxPowerTable_Index txPower;
    unsigned int           i;

    /* search for a matching backoff if there is one */
    for (i = 0; i < (sizeof(cTxMaxPower) / sizeof(cTxMaxPower[0])); i++)
    {
        if ((cTxMaxPower[i].channel == sChannel) && cTxMaxPower[i].maxPower < aPower)
        {
            /* drop aPower if it is above the channel's max power */
            aPower = cTxMaxPower[i].maxPower;
        }
    }

    txPower.dBm      = aPower;
    txPower.fraction = 0;

    /*  Standalone Tx command */
    ieeeTxCmd.txPower = txPower;
    /* Combined Rx Tx command */
    ieeeRxTxCmd.txPower = txPower;

exit:
    return retval;
}

/**
 * @brief Set global transmit power for future transmissions
 */
otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    (void)aInstance;
    otError error = OT_ERROR_NONE;

    /* Keep track of last set Tx power */
    sReqTxPower = aPower;

    rclSetTransmitPower(aPower);
    return error;
}

/**
 * @brief Get global transmit power
 */
otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;

    *aPower = sReqTxPower;
    return error;
}

/**
 * @brief Issue receive command to RCL
 */
otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    return otPlatRadioReceiveAt(aInstance, aChannel, 0, UINT32_MAX);
}

OT_TOOL_WEAK otError otPlatRadioReceiveAt(otInstance *aInstance, uint8_t aChannel, uint32_t aStart, uint32_t aDuration)
{
    (void)aInstance;
    otError           error    = OT_ERROR_BUSY;
    RCL_CommandStatus rxStatus = RCL_CommandStatus_Idle;
    /* By default, most states require Rx to be submitted */
    uint8_t rxRequired = true;

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioReceiveAt aChannel: %d aStart: %d aDuration: %d", aChannel,
               aStart, aDuration);

    if ((sState == platformRadio_phyState_TransmitStandalone) || (sState == platformRadio_phyState_Transmit))
    {
        /* Attempt to stop Tx command OR combined Rx-Tx command */
        rclStopTransmitCmd();
        Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioReceiveAt Stopping transmit");
    }

    if (sState == platformRadio_phyState_Receive || sState == platformRadio_phyState_RxTxAck)
    {

        if ((RCL_CommandStatus_isAnyStop((ieeeRxTxCmd).common.status))  && rclRxActive)
        {
            Log_printf(LogModule_Thread, Log_WARNING, "Warning: Rx stopped status but no callback scheduled.");
        }

        /* If the receive is inactive or the channel isn't disabled by the diag module always attempt to re-schedule regardless of channel selected. */
        if ((rclRxActive) && (!sDisableChannel))
        {
            /* Case 1 - Receive command already running, but on correct channel */
            if (sChannel == aChannel)
            {
                Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioReceiveAt Correct channel active");

                /* We are already running on the correct channel or the diag module
                * has disallowed switching channels.
                */
                rxRequired = false;
                error      = OT_ERROR_NONE;
            }
            else
            {
                Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioReceiveAt Incorrect channel active");

                /* Case 2 - Receive command already running, but on incorrect channel */
                /*  Need to cancel/re-submit Rx */
                rclStopReceiveCmd();
            }
        }

    }

    if (rxRequired)
    {
        /*
         * Allow the transmit power helper function to manage the characterized
         * max power.
         */
        rclSetTransmitPower(sReqTxPower);

        /* Send the command to the radio */
        rxStatus = rclSendReceiveCmd(aInstance, aChannel, aStart, aDuration, NO_EDSCAN_REQ, USE_REL_SCHED_TIMING);

        /* Update the tracking variables */
        sChannel = aChannel;
        sState   = platformRadio_phyState_Receive;
        error    = OT_ERROR_NONE;

        otEXPECT_ACTION((!RCL_CommandStatus_isAnyDescheduled(rxStatus) && rxStatus < RCL_CommandStatus_Error),
                        error = OT_ERROR_FAILED);
    }
    else
    {
        Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioReceiveAt Warning Rx NOT required");
    }
exit:
    Log_printf(LogModule_Thread, Log_DEBUG, "otPlatRadioReceiveAt schedule status: %d", rxStatus);

    if (OT_ERROR_NONE != error)
    {
        Log_printf(LogModule_Thread, Log_ERROR, "otPlatRadioReceiveAt failure, triggering event");

        /* Trigger Rx failure */
        unsigned int evts = 0U;
        evts |= RF_EVENT_RX_CMD_STOP;

        /* Tell radio processing loop what happened */
        radioSignal(evts);

        /* Return valid error so as to not assert Sub MAC */
        error = OT_ERROR_NONE;
    }

    return error;
}

/**
 * @brief Cancel all RCL commands, power state managed by RCL independently
 */
otError otPlatRadioSleep(otInstance *aInstance)
{
    otError error = OT_ERROR_BUSY;
    (void)aInstance;

    if (sState == platformRadio_phyState_Sleep)
    {
        error = OT_ERROR_NONE;
    }
    else
    {
        /* Stop any running Tx And or Rx command*/
        rclStopTransmitCmd();
        rclStopReceiveCmd();

        sState = platformRadio_phyState_Sleep;
        error  = OT_ERROR_NONE;
    }

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioSleep  Error: %d", error);

    return error;
}

/**
 * @brief Get pointer to TxAction buffer
 */
otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void)aInstance;
    return &sTransmitFrame;
}

void updateIeInfoTxFrame(otInstance *aInstance, otRadioFrame *aFrame)
{
    Log_printf(LogModule_Thread, Log_VERBOSE, "updateIeInfoTxFrame");

#ifdef NOTSUPPORTED
#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT && OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    // Seek the time sync offset and update the rendezvous time
    if (aFrame->mInfo.mTxInfo.mIeInfo->mTimeIeOffset != 0)
    {
        uint8_t *timeIe = aFrame->mPsdu + aFrame->mInfo.mTxInfo.mIeInfo->mTimeIeOffset;
        uint64_t time   = otPlatRadioGetNow(aInstance) + aFrame->mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset;

        *timeIe = aFrame->mInfo.mTxInfo.mIeInfo->mTimeSyncSeq;

        *(++timeIe) = (uint8_t)(time & 0xff);
        for (uint8_t i = 1; i < sizeof(uint64_t); i++)
        {
            time        = time >> 8;
            *(++timeIe) = (uint8_t)(time & 0xff);
        }
    }
#endif // OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT && OPENTHREAD_CONFIG_TIME_SYNC_ENABLE

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    // Update IE data in the 802.15.4 header with the newest CSL period / phase
    if (sCslPeriod > 0 && !aFrame->mInfo.mTxInfo.mIsHeaderUpdated)
    {
        otMacFrameSetCslIe(aFrame, (uint16_t)sCslPeriod, getCslPhase(aFrame));
    }
#endif // OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
#endif
}

/**
 * @brief Issue a receive command to the radio core
 *
 * @param [in] aInstance OpenThread instance structure, unused
 * @param [in] aChannel Specified channel to re-tune to (11-26)
 * @param [in] aStart Start time of receive
 * @param [in] aDuration End time of receive
 * @param [in] edScanReq True if performing non-sync based energy scan
 * @param [in] absStartReq True if aStart should be treted as an absolute time
 * in uS to issue the receive. Otherwise, aStart will be normalized to OtPlatRadioGetNow()
 * and Local radio time.
 *
 * @return RCL_CommandStatus of scheduled command
 */
static RCL_CommandStatus rclSendReceiveCmd(otInstance *aInstance,
                                    uint8_t     aChannel,
                                    uint32_t    aStart,
                                    uint32_t    aDuration,
                                    uint8_t     edScanReq,
                                    uint8_t absStartReq)
{
    if (!rclRxActive)
    {
        RCL_CommandStatus rxStatus;

        ieeeRxTxCmd.common.status = RCL_CommandStatus_Idle;

        /* True if the exact same Rx command should be re-submitted */
        if ((aChannel < 11) || (aChannel > 26))
        {
            /* Radio layer requests a receive on the previous valid channel */
            aChannel = sChannel;
        }

        rxAction->disableSync = edScanReq;

        if (!sDisableChannel)
        {
            /* If the diag module has not locked out changing the channel */
            ieeeRxTxCmd.rfFrequency = RCL_CMD_IEEE_CHANNEL_FREQUENCY(aChannel);
        }

        /* General Rx commands have no start nor end time */
        if (aStart == 0U && aDuration == UINT32_MAX)
        {
            ieeeRxTxCmd.common.scheduling          = RCL_Schedule_Now;
            ieeeRxTxCmd.common.timing.absStartTime = 0;
            ieeeRxTxCmd.common.timing.relHardStopTime = 0;
        }
        /* ED Scan only provides duration timing, duration set as relative end time */
        else if (aStart == 0U && aDuration != UINT32_MAX)
        {
            ieeeRxTxCmd.common.scheduling             = RCL_Schedule_AbsTime;
            ieeeRxTxCmd.common.timing.absStartTime    = RCL_Scheduler_getCurrentTime();
            ieeeRxTxCmd.common.timing.relHardStopTime = convertUsToRCLTicks(aDuration);
        }
        /* Only start time specified, no end */
        else if (aStart != 0U && aDuration == UINT32_MAX)
        {
            ieeeRxTxCmd.common.scheduling = RCL_Schedule_AbsTime;
            if (absStartReq)
            {
                ieeeRxTxCmd.common.timing.absStartTime = convertUsToRCLTicks(aStart);
            }
            else
            {
                ieeeRxTxCmd.common.timing.absStartTime = normalizeUsTimestamp(aStart);
            }
            ieeeRxTxCmd.common.timing.relHardStopTime = 0;
        }
        /*  
         * Start time and duration specified
         */
        else
        {
            ieeeRxTxCmd.common.scheduling = RCL_Schedule_AbsTime;
            if (absStartReq)
            {
                ieeeRxTxCmd.common.timing.absStartTime = convertUsToRCLTicks(aStart);
            }
            else
            {              
                ieeeRxTxCmd.common.timing.absStartTime = normalizeUsTimestamp(aStart);
            }
            ieeeRxTxCmd.common.timing.relHardStopTime = convertUsToRCLTicks(aDuration);
        }
          
        Log_printf(LogModule_Thread, Log_DEBUG, "rclSendReceiveCmd Timing Start Time: %d Stop Time: %d Current RCL Time: %d Current Radio Time: %d",
            ieeeRxTxCmd.common.timing.absStartTime, ieeeRxTxCmd.common.timing.relHardStopTime,
            RCL_Scheduler_getCurrentTime(), otPlatRadioGetNow(NULL));

        ieeeRxTxCmd.rxAction = rxAction;
        ieeeRxTxCmd.stats    = &ieeeRxTxStats;

        /* Allow for timing delay and maintian stop time */
        ieeeRxTxCmd.common.allowDelay = true;

        /* Clear Tx action of any stale Tx state */
        ieeeRxTxCmd.txAction = NULL;

        rxStatus = RCL_Command_submit(sRclHandle, &ieeeRxTxCmd);

        if (rxStatus < RCL_CommandStatus_Finished)
        {
            rclRxActive = true;
        }

        Log_printf(LogModule_Thread, Log_DEBUG, "rclSendReceiveCmd Status: %d Channel: %d EDScanReq: %d", rxStatus,
                   sChannel, edScanReq);
    }
    else
    {
        Log_printf(LogModule_Thread, Log_DEBUG, "rclSendReceiveCmd Rx Already active Channel: %d", sChannel);
    }

                
    return rxStatus;
}

/**
 * @brief Issue a transmit command to the radio core
 * Supported cases are as follows:
 *     Case 1) Immediate Tx without CCA
 *     Case 2) Delayed Tx without CCA
 *     Case 3) Delayed Tx with CCA
 *     Case 4) Immediate Tx with CCA
 * @param [in] aInstance OpenThread instance structure, unused
 * @param [in] aFrame OT Radio frame to transmit
 * @param [in] endRxWhenDone Signal that the radio should stop an existing receive when complete
 * @param [in] absStartReq True if aStart should be treted as an absolute time
 * in uS to issue the transmitm used for ACK frames. Otherwise, aStart will be normalized to OtPlatRadioGetNow()
 * and Local radio time, normally used for CSL frames.
 *
 * @return RCL_CommandStatus of scheduled command
 */
static RCL_CommandStatus rclSendTransmitCmd(otInstance *aInstance, otRadioFrame *aFrame, uint8_t endRxWhenDone, uint8_t absStartReq)
{
    uint8_t *pTxPktBuffer;
    uint8_t  payloadLen = aFrame->mLength - NUM_CRC_BYTES; /* OT MAC Automatically adds 2 byte CRC length */
    /* Frame timing in uS */
    uint32_t abstimeOffset = 0;

    /* True if Tx command requires Rx to be active */
    uint8_t           rxRequired = false;
    RCL_CommandStatus txStatus   = RCL_CommandStatus_Idle;

    Log_printf(LogModule_Thread, Log_VERBOSE, "rclSendTransmitCmd Incoming sTxState: %d Incoming TxAction Status: %d",
               sTxState, txAction.txStatus);

    /* Prepare Tx only command in case Rx is inactive and no ack expected */
    ieeeTxCmd.common.status              = RCL_CommandStatus_Idle;
    ieeeTxCmd.common.scheduling          = RCL_Schedule_Now;
    ieeeTxCmd.common.timing.absStartTime = abstimeOffset;
    ieeeTxCmd.common.allowDelay          = true;

    ieeeTxCmd.common.runtime.callback = rclRxTxCallback;
    ieeeTxCmd.common.runtime.rclCallbackMask.value =
        RCL_EventLastCmdDone.value | RCL_EventTxBufferFinished.value | RCL_EventCmdStepDone.value | RCL_EventGracefulStop.value | RCL_EventHardStop.value;
    ieeeTxCmd.common.runtime.lrfCallbackMask.value = LRF_EventTxDone.value;

    ieeeTxCmd.stats = &ieeeTxStats;
    /* Update statistics while command is running */
    ieeeTxStats.config.activeUpdate   = 1;
    ieeeRxTxStats.config.activeUpdate = 1;
    txAction.expectEnhAck             = false;
    txAction.expectImmAck             = false;

    /* Allow late scheduling due to RCL latency */
    txAction.allowTxDelay = true;

    if (0U == aFrame->mInfo.mTxInfo.mCsmaCaEnabled)
    {
        rxRequired = true;

        /* Setup CCA parameters */
        txAction.ccaScheduling       = RCL_Schedule_Now;
        txAction.absCcaStartTime     = 0;
        txAction.ccaMode             = RCL_CmdIeee_CcaMode1Energy;
        txAction.ccaCorrThresh       = 2;
        txAction.ccaContentionWindow = 1;
    }
    else
    {
        txAction.ccaMode = RCL_CmdIeee_NoCca;
    }

    if (otMacFrameIsAckRequested(aFrame))
    {
        rxRequired = true;

        if (otMacFrameIsVersion2015(aFrame))
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "Enhanced Ack expected for Tx");

            txAction.expectEnhAck = true;
            txAction.ackTimeout   = (288U + (32U * (1 + 1 + 4 + 127)));
        }
        else
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "Immediate Ack expected for Tx");

            txAction.expectImmAck = true;
            txAction.ackTimeout   = (288U + (32U * (1 + 1 + 4 + 5))) * 5;
        }
    }
 
    if (0U != aFrame->mInfo.mTxInfo.mTxDelay)
    {
        abstimeOffset = aFrame->mInfo.mTxInfo.mTxDelayBaseTime + aFrame->mInfo.mTxInfo.mTxDelay;
        
        /* Ack packets do not need their timing normalized */
        if (absStartReq)
        {
            abstimeOffset = convertUsToRCLTicks(abstimeOffset);
        }
        else
        {              
            abstimeOffset = normalizeUsTimestamp(abstimeOffset);
        }

        if (!rclRxActive)
        {
            /* Receive command is inactive */
            ieeeTxCmd.common.scheduling          = RCL_Schedule_AbsTime;
            ieeeTxCmd.common.timing.absStartTime = abstimeOffset;
        }
        else
        {
            /*
             * Receive is active, but an absolute time is requested.
             * Stop receive and re-schedule with correct timing parameters.
             */
            rclStopReceiveCmd();
        }
    }
    else
    {
        /* Immediate time scheduling */
        ieeeTxCmd.common.scheduling          = RCL_Schedule_Now;
        ieeeTxCmd.common.timing.absStartTime = 0;
    }

    Log_printf(LogModule_Thread, Log_DEBUG, "rclSendTransmitCmd Timing Start Time: %d Current RCL Time: %d Current Radio Time: %d",
         ieeeTxCmd.common.timing.absStartTime, RCL_Scheduler_getCurrentTime(), otPlatRadioGetNow(NULL));
    /*
     * Kick of Rx if inactive. Required for the following cases:
     * 1. Ack Request
     * 2. CCA
     * 3. Case 1/2 AND Rx stopped due to new timing requirement for Tx action
     */

    if ((rxRequired) && (!rclRxActive))
    {
        /* Receive API input for timing is in uS, convert back temporarily */
        txStatus = rclSendReceiveCmd(aInstance, aFrame->mChannel, convertRCLTicksToUs(abstimeOffset),
                                     UINT32_MAX /* No Duration, end when Transmit is complete */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
    }
    else
    {
        /* Set channel for standalone command */
        ieeeTxCmd.rfFrequency = RCL_CMD_IEEE_CHANNEL_FREQUENCY(aFrame->mChannel);
    }

    /* Setup Tx command payload */
    pTxPktBuffer = RCL_TxBuffer_init((RCL_Buffer_TxBuffer *)txPktBuffer, NUM_PAD, HDR_LEN, payloadLen);

    /* Insert length */
    pTxPktBuffer[0] = payloadLen + NUM_CRC_BYTES; /* Total length of OTA packet including CRC */

    /* Write payload */
    memcpy(&pTxPktBuffer[1], &aFrame->mPsdu[0], payloadLen);

    txAction.txEntry = (RCL_Buffer_DataEntry *)&(((RCL_Buffer_TxBuffer *)txPktBuffer)->length);

    Log_printf(LogModule_Thread, Log_VERBOSE, "Rx Required: %d Rx Active: %d", rxRequired, rclRxActive);

    /* Tx status is temporarily holding Rx state and must be IDLE/Active else Tx will fail
     * in cases where Rx is required.
     */
    if (txStatus < RCL_CommandStatus_Finished)
    {
        /*
         * Keep Receive active only if already on prior to this call.
         */
        txAction.endCmdWhenDone = endRxWhenDone;

        if (rclRxActive)
        {
            /* Submit Tx action on top of existing Rx command */
            txStatus = RCL_IEEE_Tx_submit(&ieeeRxTxCmd, &txAction);
            sTxState = platformRadio_txState_CombinedActive;
        }
        else
        {
            /* Standalone Tx, no Receive */
            ieeeTxCmd.txAction = &txAction;
            txStatus           = RCL_Command_submit(sRclHandle, &ieeeTxCmd);
            sTxState           = platformRadio_txState_StandaloneActive;
        }
    }

    Log_printf(LogModule_Thread, Log_INFO, "rclSendTransmitCmd Length: %d Status: %d sTxState: %d endRxWhenDone: %d",
               pTxPktBuffer[0], txStatus, sTxState, endRxWhenDone);

    return (txStatus);
}
#if OPENTHREAD_CONFIG_DIAG_ENABLE
/**
 * @brief Issue a Tx Test command to the radio core. Channel is that of the last submitted 
 * Receive command.
 *
 * @param [in] aInstance OpenThread instance structure, unused
 * @param [in] aModulated Either submit a unmodulated carrier wave or modulated signal
 *
 * @return RCL_CommandStatus of scheduled command
 */
static RCL_CommandStatus rclSendTxTestCmd(bool aModulated)
{
    RCL_CommandStatus txTestStatus;

    /* Initialize the command with default parameters */
    ieeeTxTestCmd = RCL_CmdIeeeTxTest_DefaultRuntime();

    /* Command configuration */
    ieeeTxTestCmd.common.scheduling = RCL_Schedule_Now;
    ieeeTxTestCmd.common.timing.relHardStopTime = 0; // No end time
    ieeeTxTestCmd.rfFrequency = RCL_CMD_IEEE_CHANNEL_FREQUENCY(sChannel);
    ieeeTxTestCmd.config.whitenMode = 0; // No whitening
    ieeeTxTestCmd.common.status = RCL_CommandStatus_Idle;

    ieeeTxTestCmd.common.runtime.callback = rclTxTestCallback;
    ieeeTxTestCmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

    if (aModulated)
    {
        ieeeTxTestCmd.config.sendCw = 0; // Send modulated signal
        ieeeTxTestCmd.txWord = PLATFORM_RADIO_TX_TEST_MODULATED_WORD;
    }
    else
    {
        ieeeTxTestCmd.config.sendCw = 1; // Send continuous wave
        ieeeTxTestCmd.txWord = PLATFORM_RADIO_TX_TEST_UNMODULATED_WORD;
    }

    txTestStatus =  RCL_Command_submit(sRclHandle, &ieeeTxTestCmd);
    return txTestStatus;
}
#endif
static uint32_t normalizeUsTimestamp(uint64_t timestamp)
{
    /* The RCL Timer runs at 4MHz while the RTC runs at 32kHz, when
     * interfacing between Host<->RCP the Host clock may add
     * considerable drift during its calculation of the next packet
     * to send for CSL communication. Use the current embedded
     * system time as a reference for the base amount of uS to
     *  delay with respect to the RCL_TICKS_PER_US.
     */
    uint64_t currentSysTime = otPlatRadioGetNow(NULL); /* uS */
    uint32_t currentRCLTime = RCL_Scheduler_getCurrentTime(); /* RCL Ticks */

    uint64_t delta = currentSysTime - timestamp; /* uS */

    return currentRCLTime - convertUsToRCLTicks(delta); /* RCL Ticks */
}

static uint64_t normalizeRCLTimestamp(uint32_t timestamp)
{
    /* The RCL timer runs at 4MHz and only returns 32 bits. This is not the format
     * that the calling sub_mac expects the timestamps. We construct the full
     * uint64_t based on the system timer even though most of this information
     * is lost. To get a full uint64_t timestamp we; get the current time from
     * both sources, calculate the delta between the RCL_TICKS_PER_US values, translate that
     * to uS, and subtract it from the current uS counter.
     *
     * The RCL timer and RTC are synchronized when the Radio core is powered on. This
     * should result in the same 0 across the system timer and the RCL_TICKS_PER_US
     * timestamps. It would be possible to use the RCL_TICKS_PER_US directly, however the
     * resolution is only 30-bits once converted to uS and may cause errors
     * with the other 32-bit values used elsewhere.
     */
    uint64_t currentSysTime = otPlatRadioGetNow(NULL); /* uS */
    uint32_t currentRCLTime = RCL_Scheduler_getCurrentTime(); /* RCL Ticks */
    uint32_t delta = currentRCLTime - timestamp; /* RCL Ticks (modulus uint32_t) */ 

    return currentSysTime - convertRCLTicksToUs(delta);
}

/* Encrypt frame */
static otError rclProcessTransmitSecurity(otRadioFrame *aFrame)
{
    otError      error = OT_ERROR_NONE;
    otExtAddress extAddr;
    uint8_t      keyId;

    otEXPECT(otMacFrameIsSecurityEnabled(aFrame) && otMacFrameIsKeyIdMode1(aFrame) &&
             !aFrame->mInfo.mTxInfo.mIsSecurityProcessed);

    if (otMacFrameIsAck(aFrame))
    {
        keyId = otMacFrameGetKeyId(aFrame);

        otEXPECT_ACTION(keyId != 0, error = OT_ERROR_FAILED);

        if (keyId == sKeyId - 1)
        {
            aFrame->mInfo.mTxInfo.mAesKey = &sPrevKey;
        }
        else if (keyId == sKeyId)
        {
            aFrame->mInfo.mTxInfo.mAesKey = &sCurrKey;
        }
        else if (keyId == sKeyId + 1)
        {
            aFrame->mInfo.mTxInfo.mAesKey = &sNextKey;
        }
        else
        {
            error = OT_ERROR_SECURITY;
            otEXPECT(false);
        }
    }
    else
    {
        keyId                         = sKeyId;
        aFrame->mInfo.mTxInfo.mAesKey = &sCurrKey;
    }

    if (!aFrame->mInfo.mTxInfo.mIsARetx)
    {
        if (otMacFrameIsAck(aFrame))
        {
            sAckKeyId        = keyId;
            sAckFrameCounter = sMacFrameCounter;
        }

        otMacFrameSetKeyId(aFrame, keyId);
        otMacFrameSetFrameCounter(aFrame, sMacFrameCounter++);
    }

    uint8_t *localAddr = (uint8_t *)&rxAction->panConfig[0].localExtAddr;
    for (size_t i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
    {
        /* reverse */
        extAddr.m8[i] = localAddr[OT_EXT_ADDRESS_SIZE - 1 - i];
    }

    otMacFrameProcessTransmitAesCcm(aFrame, &extAddr);

exit:
    Log_printf(LogModule_Thread, Log_ERROR, "rclProcessTransmitSecurity error: %d", error);

    return error;
}

/**
 * @brief Start a transmission at specified time with CCA/Ack expectations
 */
otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    otError           error = OT_ERROR_BUSY;
    RCL_CommandStatus txStatus;

    /* By default, stop Rx after Tx is complete */
    uint8_t endRxWhenDone = true;
    Log_printf(LogModule_Thread, Log_INFO, "otPlatRadioTransmit sState: %d", sState);

    if (sState == platformRadio_phyState_RxTxAck && !sTransmitPending)
    {
        /* indicate there is a pending frame after the ACK */
        otRadioFrame temp_frame = *aFrame; /* this is likely sTransmitFrame */
        sTransmitPending        = true;
        sTransmitFrame          = temp_frame;
        error                   = OT_ERROR_NONE;
    }
    else if ((sState != platformRadio_phyState_Disabled) && (sState != platformRadio_phyState_EdScan))
    {
        /* If currently in receive state, do not stop receive command after transmission */
        if (platformRadio_phyState_Receive == sState)
        {
            endRxWhenDone = false;
        }

        updateIeInfoTxFrame(aInstance, aFrame);

        otEXPECT_ACTION(OT_ERROR_NONE == rclProcessTransmitSecurity(aFrame), error = OT_ERROR_SECURITY);

        /* Tx power should be set to last OT assigned in case of multi-client operation */
        otPlatRadioSetTransmitPower(aInstance, sReqTxPower);

        txStatus = rclSendTransmitCmd(aInstance, aFrame, endRxWhenDone, USE_ABS_SCHED_TIMING);

        if (platformRadio_phyState_Sleep == sState)
        {
            sState = platformRadio_phyState_TransmitStandalone;
        }
        else
        {
            sState = platformRadio_phyState_Transmit;
        }

        /* Run-time failures handled in ISR */
        otEXPECT_ACTION((!RCL_CommandStatus_isAnyDescheduled(txStatus) && txStatus < RCL_CommandStatus_Error),
                        error = OT_ERROR_CHANNEL_ACCESS_FAILURE);

        sChannel = aFrame->mChannel;

        error = OT_ERROR_NONE;

        otPlatRadioTxStarted(aInstance, aFrame);
    }

exit:
    if (OT_ERROR_NONE != error)
    {
        Log_printf(LogModule_Thread, Log_ERROR, "otPlatRadioTransmit failure, triggering event");

        /* Command failed, either rejected or submit without Rx error */
        /* Trigger frame as started */
        otPlatRadioTxStarted(aInstance, aFrame);

        /* Trigger frame failure */
        unsigned int evts = 0U;
        evts |= RF_EVENT_TX_DONE;

        /* Tell radio processing loop what happened */
        radioSignal(evts);

        /* Cache error and return positive status so as to not assert MAC */
        sTransmitError = error;
        error          = OT_ERROR_NONE;
    }

    return error;
}
/**
 * @brief Retrieve last receieved RSSI from RCL
 */
int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void)aInstance;

    return RCL_readRssi();
}

/**
 * @brief Get supported radio capabilities, For RCL the following is supported:
 * - Ack timeout for transmissions - OT_RADIO_CAPS_ACK_TIMEOUT
 * - Scheduling transmission at specified time - OT_RADIO_CAPS_TRANSMIT_TIMING
 * - Transmit frame security - OT_RADIO_CAPS_TRANSMIT_SEC
 */
otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void)aInstance;
    return
        /* DISABLED: Enhanced Protocol Features
         * Changes to enable IEEE 802.15.4-2015 Enh-Ack make using the built in
         * protocol features of the RF Core difficult. This may be enabled in a
         * future release.
         *  OT_RADIO_CAPS_TRANSMIT_RETRIES |
         *  OT_RADIO_CAPS_CSMA_BACKOFF     |
         */
        /* DISABLED: Hardware Limitation
         * The RCL should have an RX command to TX. Let
         * the sub_mac handle this instead of the radio.c.
         *  OT_RADIO_CAPS_SLEEP_TO_TX      |
         */
        /* DISABLED: RF State Machine
         * The radio.c state machine cannot currently handle returning to sleep
         * state after a completed RX operation. Additional logic may be added
         * to handle this in future releases.
         *  OT_RADIO_CAPS_RECEIVE_TIMING   |
         */
        /* ENABLED */
        OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_TRANSMIT_TIMING |
        OT_RADIO_CAPS_TRANSMIT_SEC | OT_RADIO_CAPS_ENERGY_SCAN | OT_RADIO_CAPS_NONE;
}

/**
 * @brief Walks the short address source match list to find an address
 *
 * @param [in] aAddress the short address to search for
 *
 * @return the index where the address was found
 * @retval PLATFORM_RADIO_SRC_MATCH_NONE the address was not found
 */
static uint8_t rclFindShortSrcMatchIdx(uint16_t shortAddr)
{
    uint8_t i;
    /* Scan through entry list */
    for (i = 0; i < RCL_CMD_IEEE_SOURCE_MATCH_TABLE_SHORT_MAX_LEN; i++)
    {
        if (srcMatchTableShort->shortEntry[i].shortAddr == shortAddr)
        {
            break;
        }
    }

    /* Entry not found */
    if (i >= RCL_CMD_IEEE_SOURCE_MATCH_TABLE_SHORT_MAX_LEN)
    {
        i = PLATFORM_RADIO_SRC_MATCH_NONE;
    }

    Log_printf(LogModule_Thread, Log_VERBOSE, "rclFindShortSrcMatchIdx Searching short addr: %d Id: %d", shortAddr, i);

    return i;
}

/**
 * @brief Walks the short address source match list to find an empty slot
 *
 * @return the index of an unused address slot
 * @retval PLATFORM_RADIO_SRC_MATCH_NONE no unused slots available
 */
static uint8_t rclFindEmptyShortSrcMatchIdx(void)
{
    uint8_t i;

    /* Scan through entry list */
    for (i = 0; i < PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM; i++)
    {
        if ((srcMatchTableShort->entryEnable[i / 16] & BIT_INDEX(i & 0x0F)) == 0)
        {
            break;
        }
    }

    /* List full, no empty entries */
    if (i >= PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM)
    {
        i = PLATFORM_RADIO_SRC_MATCH_NONE;
    }

    Log_printf(LogModule_Thread, Log_VERBOSE, "rclFindEmptyShortSrcMatchIdx Id: %d", i);

    return i;
}

/**
 * @brief Cancels any running Tx/Rx command in order to trigger a re-submission in the
 * event processing loop
 */
static void rclToggleReceiveCmd(otInstance *aInstance)
{
    if (((sState == platformRadio_phyState_Receive) || (sState == platformRadio_phyState_TransmitStandalone)) &&
        (rclRxActive))
    {
        Log_printf(LogModule_Thread, Log_VERBOSE, "rclToggleReceiveCmd sState: %d", sState);

        /* Stop any running Tx And or Rx command*/
        rclStopTransmitCmd();
        rclStopReceiveCmd();
    }
}

/**
 * @brief Enables checking of the source matching table for auto-ack
 * @param [in] aInstance Unused
 * @param [in] aEnable Enable or disable source matching HW functionality
 */
void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioEnableSrcMatch Enable: %d", aEnable);

    rclToggleReceiveCmd(aInstance);

    if (aEnable)
    {
        rxAction->panConfig[0].autoAckMode = RCL_CmdIeee_AutoAck_ImmAckAutoPendAll;
    }
    else
    {
        otPlatRadioClearSrcMatchShortEntries(aInstance);
        rxAction->panConfig[0].autoAckMode = RCL_CmdIeee_AutoAck_ImmAckNoAutoPend;
    }

    return;
}

/**
 * @brief Add single short address entry to source matching table, if space permits
 * @param [in] aInstance Unused
 * @param [in] aShortAddress The short address to search for
 * @return OT_ERROR_NO_BUFS if no space available in source matching table, OT_ERROR_NONE otherwise
 */
otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    otError error = OT_ERROR_NONE;
    uint8_t idx;
    (void)aInstance;

    idx = rclFindShortSrcMatchIdx(aShortAddress);
    if (idx == PLATFORM_RADIO_SRC_MATCH_NONE)
    {
        /* the entry does not exist already, add it */
        otEXPECT_ACTION((idx = rclFindEmptyShortSrcMatchIdx()) != PLATFORM_RADIO_SRC_MATCH_NONE,
                        error = OT_ERROR_NO_BUFS);

        srcMatchTableShort->shortEntry[idx].panId     = rxAction->panConfig[0].localPanId;
        srcMatchTableShort->shortEntry[idx].shortAddr = aShortAddress;
    }

    if (idx != PLATFORM_RADIO_SRC_MATCH_NONE)
    {
        /* Index into source matching table [0-3] and set bit for entry based on source matching index */
        srcMatchTableShort->entryEnable[idx / 16] |= BIT_INDEX(idx & 0x0F);
        /* Frame pending true by default */
        srcMatchTableShort->framePending[idx / 16] |= BIT_INDEX(idx & 0x0F);

        /* Attempt to modify running or backgrounded rx command, TBD Unsupported by RCL */
        rclToggleReceiveCmd(aInstance);
    }

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioAddSrcMatchShortEntry Short Address:%d Entry ID: %d",
               aShortAddress, idx);

exit:
    return error;
}

/**
 * @brief Walks the short address source match list and
 *        clears the slot corresponding to the input short address
 * @param [in] aInstance Unused
 * @param [in] aShortAddress The short address to search for
 * @return OT_ERROR_NO_BUFS if not space available in source matching table, OT_ERROR_NONE otherwise
 */
otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    otError error = OT_ERROR_NO_BUFS;
    uint8_t idx;
    (void)aInstance;

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioClearSrcMatchShortEntry");

    /* Find id, if any, of entry */
    idx = rclFindShortSrcMatchIdx(aShortAddress);

    if (idx != PLATFORM_RADIO_SRC_MATCH_NONE)
    {
        /* Clear enable/pend bits */
        /* Entry must be kept enabled with ONLY pend bit cleared to prevent default pend for activating */
        // srcMatchTableShort->entryEnable[idx / 16] &= ~BIT_INDEX(idx & 0x0F);
        srcMatchTableShort->framePending[idx / 16] &= ~BIT_INDEX(idx & 0x0F);

        /* Toggle Rx if active */
        rclToggleReceiveCmd(aInstance);
        error = OT_ERROR_NONE;
    }

    return error;
}

/**
 * @brief Walks the short address source match list and
 *        clears the slot corresponding to the input short address
 * @param [in] aInstance Unused
 */
void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;

    uint8_t idx;
    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioClearSrcMatchShortEntries");

    /* Initialize entries */
    for (idx = 0; idx < PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM_WORDS; idx++)
    {
        srcMatchTableShort->entryEnable[idx]  = 0;
        srcMatchTableShort->framePending[idx] = 0;
    }

    /* Toggle Rx if active */
    rclToggleReceiveCmd(aInstance);

    return;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    (void)aInstance;

    otError error = OT_ERROR_NONE;

    return error;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    (void)aInstance;
    otError error = OT_ERROR_NONE;

    return error;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;

    return;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void)aInstance;
    return (rxAction->numPan == 0);
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    /* By default updates to RX action require receiver to be restarted if active */
    uint8_t rxRequired = true;

    if (aEnable)
    {
        /* Disable Frame filtering */
        rxAction->numPan = 0;
    }
    else
    {
        rxAction->numPan = 1;
    }

    /* Check if Rx must be restarted for changes to take effect */
    if (platformRadio_phyState_Sleep == sState || platformRadio_phyState_Disabled == sState)
    {
        rxRequired = false;
    }
    else
    {
        /* Stop any running Tx And or Rx command*/
        rclStopTransmitCmd();
        rclStopReceiveCmd();
    }

    if (rxRequired)
    {
        /* Start Rx immediately, pull previous timing information from existing command */
        rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
    }

    return;
}

void otPlatRadioSetMacKey(otInstance             *aInstance,
                          uint8_t                 aKeyIdMode,
                          uint8_t                 aKeyId,
                          const otMacKeyMaterial *aPrevKey,
                          const otMacKeyMaterial *aCurrKey,
                          const otMacKeyMaterial *aNextKey,
                          otRadioKeyType          aKeyType)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aKeyIdMode);

    otEXPECT(aKeyType == OT_KEY_TYPE_LITERAL_KEY);
    otEXPECT(aPrevKey != NULL && aCurrKey != NULL && aNextKey != NULL);

    sKeyId   = aKeyId;
    sPrevKey = *aPrevKey;
    sCurrKey = *aCurrKey;
    sNextKey = *aNextKey;
exit:
    return;
}

void otPlatRadioSetMacFrameCounter(otInstance *aInstance, uint32_t aMacFrameCounter)
{
    OT_UNUSED_VARIABLE(aInstance);

    sMacFrameCounter = aMacFrameCounter;
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    switch (sState)
    {
    case platformRadio_phyState_Disabled:
        return OT_RADIO_STATE_DISABLED;

    case platformRadio_phyState_Sleep:
        return OT_RADIO_STATE_SLEEP;

    case platformRadio_phyState_Receive:
    case platformRadio_phyState_RxTxAck:
    case platformRadio_phyState_EdScanStandalone:
    case platformRadio_phyState_EdScan:
        return OT_RADIO_STATE_RECEIVE;

    case platformRadio_phyState_Transmit:
    case platformRadio_phyState_TransmitStandalone:
        return OT_RADIO_STATE_TRANSMIT;

    default:
        return OT_RADIO_STATE_INVALID;
    }
}

uint64_t otPlatRadioGetNow(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return otPlatTimeGet();
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    (void)aInstance;
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanid)
{
    (void)aInstance;
    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioSetPanId PANID: %x", aPanid);

    if (((sState == platformRadio_phyState_Receive) || (sState == platformRadio_phyState_TransmitStandalone)) &&
        (rclRxActive))
    {
        /* Stop any running Tx And or Rx command*/
        rclStopTransmitCmd();
        rclStopReceiveCmd();

        rxAction->panConfig[0].localPanId = aPanid;

        /* Start Rx immediately, pull previous timing information from existing command */
        rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
    }
    else
    {
        rxAction->panConfig[0].localPanId = aPanid;
    }

    return;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    (void)aInstance;

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioSetExtendedAddress");
    Log_printf(LogModule_Thread, Log_VERBOSE, "%x %x %x %x", aAddress->m8[0], aAddress->m8[1], aAddress->m8[2], aAddress->m8[3]);
    Log_printf(LogModule_Thread, Log_VERBOSE, "%x %x %x %x", aAddress->m8[4], aAddress->m8[5], aAddress->m8[6], aAddress->m8[7]);

    if (((sState == platformRadio_phyState_Receive) || (sState == platformRadio_phyState_TransmitStandalone)) &&
        (rclRxActive))
    {
        /* Stop any running Tx And or Rx command*/
        rclStopTransmitCmd();
        rclStopReceiveCmd();

        memcpy((void *)&rxAction->panConfig[0].localExtAddr, aAddress, sizeof(otExtAddress));

        /* Start Rx immediately, pull previous timing information from existing command */
        rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
    }
    else
    {
        memcpy((void *)&rxAction->panConfig[0].localExtAddr, aAddress, sizeof(otExtAddress));
    }

    return;
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    (void)aInstance;

    Log_printf(LogModule_Thread, Log_VERBOSE, "otPlatRadioSetShortAddress Address: %d", aAddress);

    if (((sState == platformRadio_phyState_Receive) || (sState == platformRadio_phyState_TransmitStandalone)) &&
        (rclRxActive))
    {
        /* Stop any running Tx And or Rx command*/
        rclStopTransmitCmd();
        rclStopReceiveCmd();

        rxAction->panConfig[0].localShortAddr = aAddress;

        /* Start Rx immediately, pull previous timing information from existing command */
        rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
    }
    else
    {
        rxAction->panConfig[0].localShortAddr = aAddress;
    }

    return;
}

void otPlatRadioUpdateCslSampleTime(otInstance *aInstance, uint32_t aCslSampleTime)
{
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioEnableCsl(otInstance         *aInstance,
                             uint32_t            aCslPeriod,
                             otShortAddress      aShortAddr,
                             const otExtAddress *aExtAddr)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddr);
    OT_UNUSED_VARIABLE(aExtAddr);

    return OT_ERROR_NONE;
}

uint8_t otPlatRadioGetCslAccuracy(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return 0;
}

uint8_t otPlatRadioGetCslUncertainty(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return 0;
}

otError otPlatRadioConfigureEnhAckProbing(otInstance          *aInstance,
                                          otLinkMetrics        aLinkMetrics,
                                          const otShortAddress aShortAddress,
                                          const otExtAddress  *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    return otLinkMetricsConfigureEnhAckProbing(aShortAddress, aExtAddress, aLinkMetrics);
}

#if OPENTHREAD_CONFIG_DIAG_ENABLE
void rfCoreDiagChannelDisable(uint8_t aChannel)
{
    otPlatRadioReceive(NULL, aChannel);
    sDisableChannel = true;
}

void rfCoreDiagChannelEnable(uint8_t aChannel)
{
    sDisableChannel = false;
    otPlatRadioReceive(NULL, aChannel);
}
otError otPlatDiagRadioToneStart(otInstance *aInstance, bool aModulated)
{
    (void)aInstance;
    otError retval = OT_ERROR_NONE;
    RCL_CommandStatus txTestStatus;

    if (platformRadio_phyState_Receive == sState)
    {
        rclStopReceiveCmd();
    }

    sState = platformRadio_phyState_TransmitStandalone;

    txTestStatus = rclSendTxTestCmd(aModulated);

    Log_printf(LogModule_Thread, Log_DEBUG, "otPlatDiagRadioToneStart Status: %d Modulated: %d", txTestStatus, aModulated);

    otEXPECT_ACTION((!RCL_CommandStatus_isAnyDescheduled(txTestStatus) && txTestStatus < RCL_CommandStatus_Error),
        retval = OT_ERROR_FAILED);
exit:
    return retval;

}

otError otPlatDiagRadioToneStop(otInstance *aInstance)
{
    (void)aInstance;
    otError retval = OT_ERROR_NONE;

    sState = platformRadio_phyState_Sleep;

    rclStopReceiveCmd();
    RCL_Command_stop(&ieeeTxTestCmd, RCL_StopType_Hard);

    Log_printf(LogModule_Thread, Log_WARNING, "otPlatDiagRadioToneStop");

exit:
    return retval;
}
#endif

#if OPENTHREAD_CONFIG_RADIO_NO_WEAK_DEFINITIONS
/* These functions are defined here to avoid undefined symbol errors in the
 * linker. OpenThread usually defines these as "weak" symbols, but we are
 * unable to use that when the functions are compiled into libraries. The
 * compiler strips off those hints when the code is put into an archive.
 */

uint32_t otPlatRadioGetSupportedChannelMask(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MASK;
}

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *aInstance)
{
    return otPlatRadioGetSupportedChannelMask(aInstance);
}

const char *otPlatRadioGetVersionString(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return otGetVersionString();
}

uint32_t otPlatRadioGetBusSpeed(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return 0;
}

otError otPlatRadioGetFemLnaGain(otInstance *aInstance, int8_t *aGain)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aGain);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetFemLnaGain(otInstance *aInstance, int8_t aGain)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aGain);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetChannelMaxTransmitPower(otInstance *aInstance, uint8_t aChannel, int8_t aMaxPower)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aChannel);
    OT_UNUSED_VARIABLE(aMaxPower);

    return OT_ERROR_NOT_IMPLEMENTED;
}

static otPlat_radioRegion_t regionCodeToPowerTableIndex(uint16_t region_code)
{
    otPlat_radioRegion_t region_index = OT_HAL_REGION_MAX;
    switch (region_code)
    {
    case CC_UINT16('U', 'S'):
        region_index = OT_HAL_REGION_UNITED_STATES;
        break;
    case CC_UINT16('J', 'P'):
        region_index = OT_HAL_REGION_JAPAN;
        break;
    case CC_UINT16('I', 'N'):
        region_index = OT_HAL_REGION_INDIA;
        break;
    case CC_UINT16('C', 'A'):
        region_index = OT_HAL_REGION_CANADA;
        break;
    case CC_UINT16('A', 'U'):
    case CC_UINT16('N', 'Z'):
        region_index = OT_HAL_REGION_AU_NZ;
        break;
    case CC_UINT16('B', 'R'):
        region_index = OT_HAL_REGION_BRAZIL;
        break;
    case CC_UINT16('M', 'X'):
        region_index = OT_HAL_REGION_MEXICO;
        break;
    case CC_UINT16('G', 'B'):
    case CC_UINT16('D', 'E'):
    case CC_UINT16('F', 'R'):
    case CC_UINT16('I', 'T'):
    case CC_UINT16('E', 'S'):
        region_index = OT_HAL_REGION_EUROPE;
        break;
    default:
        region_index = OT_HAL_REGION_WORLD_WIDE;
        break;
    }
    return region_index;
}
otError otPlatRadioSetRegion(otInstance *aInstance, uint16_t aRegionCode)
{
    otError retval = OT_ERROR_NONE;

    return retval;
}

otError otPlatRadioGetRegion(otInstance *aInstance, uint16_t *aRegionCode)
{
    OT_UNUSED_VARIABLE(aInstance);
    otError retval = OT_ERROR_NONE;

    return retval;
}

#endif /* OPENTHREAD_CONFIG_RADIO_NO_WEAK_DEFINITIONS */

static void platformRadioProcessTransmitDone(otInstance   *aInstance,
                                             otRadioFrame *aTransmitFrame,
                                             otRadioFrame *aAckFrame,
                                             otError       aTransmitError)
{
    Log_printf(LogModule_Thread, Log_INFO, "platformRadioProcessTransmitDone");

    clearTransmitState();

#if OPENTHREAD_CONFIG_DIAG_ENABLE
    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioTransmitDone(aInstance, aTransmitFrame, aTransmitError);
    }
    else
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */
    {
        otPlatRadioTxDone(aInstance, aTransmitFrame, aAckFrame, aTransmitError);
    }
}

static void platformRadioProcessReceiveDone(otInstance *aInstance, otRadioFrame *aReceiveFrame, otError aReceiveError)
{
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioReceiveDone(aInstance, aReceiveFrame, aReceiveError);
    }
    else
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */
    {
        otPlatRadioReceiveDone(aInstance, aReceiveFrame, aReceiveError);
        Log_printf(LogModule_Thread, Log_INFO, "platformRadioProcessReceiveDone Error: %d", aReceiveError);
    }
}

/**
 * Handle events in the TX state.
 */
static void handleTxState(otInstance *aInstance, unsigned int events)
{
    /* Save error on the stack and clear global variable */
    otError error = sTransmitError;

    if (OT_ERROR_NONE != error || 0U == (sTransmitFrame.mPsdu[0] & IEEE802154_ACK_REQUEST))
    {
        sTransmitError = OT_ERROR_NONE;

        if (sState == platformRadio_phyState_TransmitStandalone)
        {
            sState = platformRadio_phyState_Sleep;
        }
        else
        {
            sState = platformRadio_phyState_Receive;
        }
        Log_printf(LogModule_Thread, Log_INFO, "handleTxState Transmit Complete New sState: %d", sState);

        platformRadioProcessTransmitDone(aInstance, &sTransmitFrame, NULL, error);
    }
    else
    {
        /* Acks are handled in the receive handler */
        Log_printf(LogModule_Thread, Log_VERBOSE, "handleTxState Waiting for Ack");
    }
}

void platformRadioProcess(otInstance *aInstance, uintptr_t arg)
{
    Log_printf(LogModule_Thread, Log_INFO, "platformRadioProcess Events: %d", arg);

    /* Handle the events based on the radio state */
    switch (sState)
    {
    case platformRadio_phyState_Sleep:
    {
        Log_printf(LogModule_Thread, Log_VERBOSE, "State; platformRadio_phyState_Sleep");

        if (arg & (RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE))
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "Clearing Rx Queue RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE");

            /* Unfortunately, the frame must be discarded or we risk
             * asserting the MAC.
             */
            clearRxQueue();
        }
        break;
    }

    case platformRadio_phyState_TransmitStandalone:
    case platformRadio_phyState_Transmit:
    {
        if (platformRadio_phyState_TransmitStandalone == sState)
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "State; platformRadio_phyState_TransmitStandalone");
        }
        else
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "State; platformRadio_phyState_Transmit");
        }

        /* The TX command string has finished */
        if (arg & RF_EVENT_TX_DONE)
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "RF_EVENT_TX_DONE");

            handleTxState(aInstance, arg);
        }
        /* Clear the receive buffer if the radio can't find space to put RX frames */
        else if (arg & RF_EVENT_BUF_FULL)
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "RF_EVENT_BUF_FULL");

            clearRxQueue();
            sTransmitError = OT_ERROR_ABORT;
            handleTxState(aInstance, RF_EVENT_TX_DONE);
        }

        /* Handle new received frame */
        if (arg & (RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE))
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE");

            processRxQueue(aInstance, arg);
        }

        /* Re-start the RX command since we are still in the state. */
        if ((platformRadio_phyState_Transmit == sState) && (arg & RF_EVENT_RX_CMD_STOP))
        {
            Log_printf(LogModule_Thread, Log_DEBUG, "RF_EVENT_RX_CMD_STOP");

            /* Start Rx immediately, pull previous timing information from existing command */
            rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
        }
        break;
    }
#ifdef NOTYETSUPPORTED
    case platformRadio_phyState_RxTxAck:
    {
        Log_printf(LogModule_Thread, Log_VERBOSE, "platformRadio_phyState_RxTxAck");

        /* The TX command string has finished */
        if (arg & RF_EVENT_TX_DONE)
        {
            arg |= RF_EVENT_RX_ACK_DONE;
            sState = platformRadio_phyState_Receive;
            processRxQueue(aInstance, arg);

            if (sTransmitPending)
            {
                sTransmitPending = false;
                otPlatRadioTransmit(aInstance, &sTransmitFrame);
            }
        }
        else if (arg & RF_EVENT_RX_DONE)
        {
            processRxQueue(aInstance, arg);
        }

        /* Re-start the RX command since we are still in the state. */
        if (arg & RF_EVENT_RX_CMD_STOP)
        {
            rfCoreSendReceiveCmd(sRfHandle);
            sState = platformRadio_phyState_Receive;
        }
        break;
    }
#endif

    case platformRadio_phyState_EdScan:
    {
        Log_printf(LogModule_Thread, Log_DEBUG, "platformRadio_phyState_EdScan");

        if (arg & RF_EVENT_ED_SCAN_DONE)
        {
            /* Return to previous state */
            sState = (sState == platformRadio_phyState_EdScanStandalone) ? platformRadio_phyState_Sleep
                                                                         : platformRadio_phyState_Receive;

            otPlatRadioEnergyScanDone(aInstance, ieeeRxTxStats.maxRssi);
        }

        if (arg & RF_EVENT_RX_CMD_ERROR)
        {
            otPlatRadioEnergyScanDone(aInstance, PLATFORM_RADIO_INVALID_RSSI);
        }

        if ((arg & RF_EVENT_RX_CMD_ERROR) || (arg & RF_EVENT_ED_SCAN_DONE))
        {
            /* Re-start the RX command since we are still in the state. */
            if (platformRadio_phyState_Receive == sState)
            {
                Log_printf(LogModule_Thread, Log_DEBUG, "ED Scan Restart Rx");

                /* Start Rx immediately, pull previous timing information from existing command */
                rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
            }
        }

        break;
    }
    case platformRadio_phyState_Receive:
    {
        Log_printf(LogModule_Thread, Log_DEBUG, "State; platformRadio_phyState_Receive");

        /* Handle new received frame */
        if (arg & (RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE))
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE");

            processRxQueue(aInstance, arg);
        }

        /* Clear the receive buffer if the radio can't find space to put RX frames */
        if (arg & RF_EVENT_BUF_FULL)
        {
            Log_printf(LogModule_Thread, Log_VERBOSE, "RF_EVENT_BUF_FULL");

            clearRxQueue();
        }

        /* Re-start the RX command since we are still in the state. */
        if (arg & RF_EVENT_RX_CMD_STOP)
        {
            Log_printf(LogModule_Thread, Log_DEBUG, "RF_EVENT_RX_CMD_STOP");

            /* Start Rx immediately, pull previous timing information from existing command */
            rclSendReceiveCmd(aInstance, 0, 0, UINT32_MAX /* Infinite Duration */, NO_EDSCAN_REQ, USE_ABS_SCHED_TIMING);
        }
        break;
    }
    case platformRadio_phyState_Disabled:
        Log_printf(LogModule_Thread, Log_VERBOSE, "State; platformRadio_phyState_Disabled");
        break;
    default:
        Log_printf(LogModule_Thread, Log_ERROR, "State; Default Unknown");
        break;
    }
}
