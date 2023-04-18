/*
 *  Copyright (c) 2017, Texas Instruments Incorporated
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

#include <openthread/config.h>

#include "radio.h"

#if defined(USE_DMM)
#include <dmm/dmm_rfmap.h>
#include <dmm_thread_activity.h>
#elif defined(TIOP_RADIO_USE_CSF)
#include <cfs_rfmap.h>
#else
#include <ti/drivers/rf/RF.h>
#endif

#include <stddef.h>

#include <utils/code_utils.h>
#include <utils/link_metrics.h>
#include <utils/mac_frame.h>
#include <openthread/diag.h>
#include <openthread/link.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/entropy.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>

// clang-format off
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rfc.h)
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_ieee_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_ieee_mailbox.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(inc/hw_ccfg.h)
#include DeviceFamily_constructPath(inc/hw_fcfg1.h)
// clang-format on

#include <ti_radio_config.h>

#include "system.h"
#include "ti_drivers_config.h"
#include "ti_radio_config.h"

#define PLAT_RADIO_SOFTWARE_IMM_ACK 1
#ifndef PLAT_RADIO_SOFTWARE_IMM_ACK
#define PLAT_RADIO_SOFTWARE_IMM_ACK 0
#endif

/* The sync word used by the radio TX test command */
#define PLATFORM_RADIO_TX_TEST_SYNC_WORD 0x71764129

/* Words sent by the TX test command in modulated/unmodulated mode
 * 0xAAAA = 0b10101010101010101010
 * 0xFFFF = 0b11111111111111111111
 */
#define PLATFORM_RADIO_TX_TEST_MODULATED_WORD 0xAAAA
#define PLATFORM_RADIO_TX_TEST_UNMODULATED_WORD 0xFFFF

/* state of the RF interface */
static volatile platformRadio_phyState sState;

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

/*
 * Radio command structures that run on the CM0.
 */
#if defined(SUPPORT_HIGH_PA) || defined(LP_CC2653P10) || defined(LP_EM_CC1354P10_1) || defined(LP_EM_CC1354P10_6)
static volatile rfc_CMD_RADIO_SETUP_PA_t sRadioSetupCmd;
#else
static volatile rfc_CMD_RADIO_SETUP_t sRadioSetupCmd;
#endif

static volatile rfc_CMD_IEEE_MOD_FILT_t      sModifyReceiveFilterCmd;
static volatile rfc_CMD_IEEE_MOD_SRC_MATCH_t sModifyReceiveSrcMatchCmd;

static volatile rfc_CMD_IEEE_ED_SCAN_t sEdScanCmd;

static volatile rfc_CMD_TX_TEST_t sTxTestCmd;

static volatile rfc_CMD_IEEE_RX_t sReceiveCmd;

static volatile rfc_CMD_IEEE_TX_t sTransmitCmd;

static volatile ext_src_match_data_t   sSrcMatchExtData;
static volatile short_src_match_data_t sSrcMatchShortData;

/* struct containing radio stats */
static volatile rfc_ieeeRxOutput_t sRfStats;

/*
 * Four receive buffers entries with room for 1 max IEEE802.15.4 frame in each
 *
 * These will be setup in a circular buffer configuration by /ref sRxDataQueue.
 */
static __attribute__((aligned(4))) uint8_t sRxBuf0[RX_BUF_SIZE];
static __attribute__((aligned(4))) uint8_t sRxBuf1[RX_BUF_SIZE];
static __attribute__((aligned(4))) uint8_t sRxBuf2[RX_BUF_SIZE];
static __attribute__((aligned(4))) uint8_t sRxBuf3[RX_BUF_SIZE];

/*
 * The RX Data Queue used by @ref sReceiveCmd.
 */
static __attribute__((aligned(4))) dataQueue_t sRxDataQueue = {0};

/* openthread data primitives */
static otError       sTransmitError;
static otRadioFrame  sTransmitFrame;
static otRadioIeInfo sTransmitFrameIeInfo;
static otRadioFrame  sAckFrame;
static otRadioIeInfo sAckFrameIeInfo;

static __attribute__((aligned(4))) uint8_t sTransmitPsdu[OT_RADIO_FRAME_MAX_SIZE];
static __attribute__((aligned(4))) uint8_t sAckPsdu[OT_RADIO_FRAME_MAX_SIZE];

static RF_Object sRfObject;

static RF_Handle sRfHandle;

static RF_CmdHandle sReceiveCmdHandle = (RF_CmdHandle)RF_ALLOC_ERROR;
static RF_CmdHandle sTransmitCmdHandle;
static RF_CmdHandle sTransmitAckCmdHandle;
static RF_CmdHandle sTxTestCmdHandle;

/**
 * Value requested in dBm from the upper layers on the last call to
 * @ref otPlatRadioSetTransmitPower.
 */
static int8_t sReqTxPower = 0U;

/**
 * Array of back-off values necessary for passing FCC testing.
 */
static const struct tx_power_max cTxMaxPower[] = {
#if defined(LP_CC2652RSIP)
    {.channel = 26, .maxPower = 2}, /* back-off for FCC/CAN band edge, not needed in ETSI */

#elif defined(LAUNCHXL_CC1352P_2)
    {.channel = 26, .maxPower = 15}, /* back-off for 25 deg C, 3.3V */
    {.channel = 25, .maxPower = 19}, /* back-off for 25 deg C, 3.3V */

#else
    /* IAR: Error[Pe1345]: an empty initializer is invalid for an array with unspecified bound
     * GCC: error: comparison of unsigned expression in '< 0' is always false [-Werror=type-limits]
     *
     * Adding an extra value with max theoretical channel and power numbers to
     * enable IAR and pedantic C warnings to build with this array. If the
     * device does not have regulatory back-offs this is a minor cost of
     * code and data.
     */
    {.channel = UINT8_MAX, .maxPower = UINT8_MAX},

#endif
};

/* Status of the coex priority signal line. This is passed to the RF Driver
 * with the scheduling parameters for each command. The value of this will only
 * be important in 2-wire or greater CoEx configurations.
 */
static RF_PriorityCoex sPriorityCoex = RF_PriorityCoexDefault;

/* Status of the coex request signal for RX operations. This is passed to the
 * RF Driver with the scheduling parameters for each command. The value of this
 * will only be important in 3-wire or greater CoEx configurations.
 */
static RF_RequestCoex sRequestCoex = RF_RequestCoexDefault;

/* Transmit security keying material */
static uint8_t          sKeyId;
static uint8_t          sAckKeyId;
static otMacKeyMaterial sPrevKey;
static otMacKeyMaterial sCurrKey;
static otMacKeyMaterial sNextKey;
static uint32_t         sMacFrameCounter;
static uint32_t         sAckFrameCounter;

// PPM of the 32KHz clock source, will vary based on the design's LF source
#define XTAL_UNCERTAINTY 10U
// Uncertainty of scheduling a CSL transmission, in ±10 us units. This will
// vary based on the LF clock source used for the system's alarm module. The RF
// driver will use a combination of RAT/HF/LF clock to schedule commands.
#define CSL_TX_UNCERTAINTY 5U

static uint32_t sCslPeriod;
static uint32_t sCslSampleTime;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
static uint16_t getCslPhase(otRadioFrame *aFrame)
{
    uint32_t frameTime, cslPeriodInUs, delta;

    if (aFrame->mInfo.mTxInfo.mTxDelayBaseTime == 0U && aFrame->mInfo.mTxInfo.mTxDelay == 0U)
    {
        // best guess
        frameTime = otPlatRadioGetNow(NULL);
    }
    else
    {
        frameTime = aFrame->mInfo.mTxInfo.mTxDelayBaseTime + aFrame->mInfo.mTxInfo.mTxDelay;
    }
    cslPeriodInUs = sCslPeriod * OT_US_PER_TEN_SYMBOLS;
    delta = ((sCslSampleTime % cslPeriodInUs) - (frameTime % cslPeriodInUs) + cslPeriodInUs) % cslPeriodInUs;

    return (uint16_t)(delta / OT_US_PER_TEN_SYMBOLS);
}
#endif /* OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE */

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
 * @brief initialize the RX/TX buffers
 *
 * Zeros out the receive and transmit buffers and sets up the data structures
 * of the receive queue.
 */
static void rfCoreInitBufs(void)
{
    rfc_dataEntry_t *entry;
    memset(sRxBuf0, 0x00, sizeof(sRxBuf0));
    memset(sRxBuf1, 0x00, sizeof(sRxBuf1));
    memset(sRxBuf2, 0x00, sizeof(sRxBuf2));
    memset(sRxBuf3, 0x00, sizeof(sRxBuf3));

    entry               = (rfc_dataEntry_t *)sRxBuf0;
    entry->pNextEntry   = sRxBuf1;
    entry->config.lenSz = DATA_ENTRY_LENSZ_BYTE;
    entry->length       = sizeof(sRxBuf0) - sizeof(rfc_dataEntry_t);

    entry               = (rfc_dataEntry_t *)sRxBuf1;
    entry->pNextEntry   = sRxBuf2;
    entry->config.lenSz = DATA_ENTRY_LENSZ_BYTE;
    entry->length       = sizeof(sRxBuf1) - sizeof(rfc_dataEntry_t);

    entry               = (rfc_dataEntry_t *)sRxBuf2;
    entry->pNextEntry   = sRxBuf3;
    entry->config.lenSz = DATA_ENTRY_LENSZ_BYTE;
    entry->length       = sizeof(sRxBuf2) - sizeof(rfc_dataEntry_t);

    entry               = (rfc_dataEntry_t *)sRxBuf3;
    entry->pNextEntry   = sRxBuf0;
    entry->config.lenSz = DATA_ENTRY_LENSZ_BYTE;
    entry->length       = sizeof(sRxBuf3) - sizeof(rfc_dataEntry_t);

    sRxDataQueue.pCurrEntry = sRxBuf0;
    sRxDataQueue.pLastEntry = NULL;

    sTransmitFrame.mPsdu   = sTransmitPsdu;
    sTransmitFrame.mLength = 0;
    sTransmitFrame.mInfo.mTxInfo.mIeInfo = &sTransmitFrameIeInfo;

    sAckFrame.mPsdu   = sAckPsdu;
    sAckFrame.mLength = 0;
    sAckFrame.mInfo.mTxInfo.mIeInfo = &sAckFrameIeInfo;
}
/**
 * @brief initializes the setup command structure
 *
 * The sRadioSetupCmd struct is used by the RF driver to bring the
 */
void rfCoreInitSetupCmd(void)
{
    /* initialize radio setup command */
    sRadioSetupCmd = RF_cmdIeeeRadioSetup;

    sRadioSetupCmd.startTrigger.pastTrig = 1; // XXX: workaround for RF scheduler
}

/**
 * @brief initialize the RX command structure
 *
 * Sets the default values for the receive command structure.
 */
static void rfCoreInitReceiveParams(void)
{
    sReceiveCmd = RF_cmdIeeeRx;

    sReceiveCmd.pRxQ            = &sRxDataQueue;
    sReceiveCmd.pOutput         = (rfc_ieeeRxOutput_t *)&sRfStats;
    sReceiveCmd.numShortEntries = PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM;
    sReceiveCmd.pShortEntryList = (void *)&sSrcMatchShortData;
    sReceiveCmd.numExtEntries   = PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM;
    sReceiveCmd.pExtEntryList   = (uint32_t *)&sSrcMatchExtData;
    sReceiveCmd.channel         = OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MIN;
    sReceiveCmd.ccaRssiThr      = -90;

    sReceiveCmd.startTrigger.pastTrig = 1; // XXX: workaround for RF scheduler
    sReceiveCmd.condition.rule        = COND_NEVER;

    sReceiveCmd.rxConfig.bAutoFlushCrc = 1;

    sReceiveCmd.rxConfig.bAutoFlushIgn    = 1;
    sReceiveCmd.rxConfig.bIncludePhyHdr   = 1;
    sReceiveCmd.rxConfig.bAppendCorrCrc   = 1;
    sReceiveCmd.rxConfig.bAppendRssi      = 1;
    sReceiveCmd.rxConfig.bAppendSrcInd    = 1;
    sReceiveCmd.rxConfig.bAppendTimestamp = 1;

    sReceiveCmd.frameFiltOpt.frameFiltEn      = 1;
    sReceiveCmd.frameFiltOpt.frameFiltStop    = 1;
    sReceiveCmd.frameFiltOpt.autoAckEn        = 0;
    sReceiveCmd.frameFiltOpt.bStrictLenFilter = 0;
#if defined(TIOP_RADIO_USE_CSF)
    sReceiveCmd.dualPanFiltOpt.frameFilterDoneEn0 = 1;
    sReceiveCmd.dualPanFiltOpt.bAllowEnhAck0 = 1;
#endif
    sReceiveCmd.ccaOpt.ccaEnEnergy = 1;
    sReceiveCmd.ccaOpt.ccaEnCorr   = 1;
    sReceiveCmd.ccaOpt.ccaEnSync   = 1;
    sReceiveCmd.ccaOpt.ccaSyncOp   = 1;
    sReceiveCmd.ccaOpt.ccaCorrThr  = 1;
}

/**
 * @brief Get the receive command's sensitivity.
 */
int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    (void)aInstance;
    return sReceiveCmd.ccaRssiThr;
}

/**
 * @brief sends the direct abort command to the radio core
 *
 * @param [in] aRfHandle    The handle for the RF core client
 * @param [in] aRfCmdHandle The command handle to be aborted
 *
 * @return the value from the command status register
 * @retval RF_StatSuccess the command completed correctly
 */
static RF_Stat rfCoreExecuteAbortCmd(RF_Handle aRfHandle, RF_CmdHandle aRfCmdHandle)
{
    return RF_cancelCmd(aRfHandle, aRfCmdHandle, RF_DRIVER_ABORT);
}

/**
 * @brief enable/disable filtering
 *
 * Uses the radio core to alter the current running RX command filtering
 * options. This ensures there is no access fault between the CM3 and CM0 for
 * the RX command.
 *
 * This function leaves the type of frames to be filtered the same as the
 * receive command.
 *
 * @note An IEEE RX command *must* be running while this command executes.
 *
 * @param [in] aRfHandle The handle for the RF core client
 * @param [in] aEnable   TRUE: enable frame filtering
 *                       FALSE: disable frame filtering
 *
 * @return the value from the command status register
 * @retval RF_StatCmdDoneSuccess the command completed correctly
 */
static RF_Stat rfCoreModifyRxFrameFilter(RF_Handle aRfHandle, bool aEnable)
{
    /* memset skipped because sModifyReceiveFilterCmd has only 3 members */
    sModifyReceiveFilterCmd.commandNo = CMD_IEEE_MOD_FILT;
    /* copy current frame filtering and frame types from running RX command */
    memcpy((void *)&sModifyReceiveFilterCmd.newFrameFiltOpt, (void *)&sReceiveCmd.frameFiltOpt,
           sizeof(sModifyReceiveFilterCmd.newFrameFiltOpt));
    memcpy((void *)&sModifyReceiveFilterCmd.newFrameTypes, (void *)&sReceiveCmd.frameTypes,
           sizeof(sModifyReceiveFilterCmd.newFrameTypes));

    sModifyReceiveFilterCmd.newFrameFiltOpt.frameFiltEn = aEnable ? 1 : 0;

    return RF_runImmediateCmd(aRfHandle, (uint32_t *)&sModifyReceiveFilterCmd);
}

/**
 * @brief enable/disable autoPend
 *
 * Uses the radio core to alter the current running RX command filtering
 * options. This ensures there is no access fault between the CM3 and CM0 for
 * the RX command.
 *
 * This function leaves the type of frames to be filtered the same as the
 * receive command.
 *
 * @note An IEEE RX command *must* be running while this command executes.
 *
 * @param [in] aRfHandle The handle for the RF core client
 * @param [in] aEnable TRUE: enable autoPend, FALSE: disable autoPend
 *
 * @return the value from the command status register
 * @retval RF_StatCmdDoneSuccess the command completed correctly
 */
static RF_Stat rfCoreModifyRxAutoPend(RF_Handle aRfHandle, bool aEnable)
{
    /* memset skipped because sModifyReceiveFilterCmd has only 3 members */
    sModifyReceiveFilterCmd.commandNo = CMD_IEEE_MOD_FILT;
    /* copy current frame filtering and frame types from running RX command */
    memcpy((void *)&sModifyReceiveFilterCmd.newFrameFiltOpt, (void *)&sReceiveCmd.frameFiltOpt,
           sizeof(sModifyReceiveFilterCmd.newFrameFiltOpt));
    memcpy((void *)&sModifyReceiveFilterCmd.newFrameTypes, (void *)&sReceiveCmd.frameTypes,
           sizeof(sModifyReceiveFilterCmd.newFrameTypes));

    sModifyReceiveFilterCmd.newFrameFiltOpt.autoPendEn = aEnable ? 1 : 0;

    return RF_runImmediateCmd(aRfHandle, (uint32_t *)&sModifyReceiveFilterCmd);
}

/**
 * @brief sends the immediate modify source matching command to the radio core
 *
 * Uses the radio core to alter the current source matching parameters used by
 * the running RX command. This ensures there is no access fault between the
 * CM3 and CM0, and ensures that the RX command has cohesive view of the data.
 * The CM3 may make alterations to the source matching entries if the entry is
 * marked as disabled.
 *
 * @note An IEEE RX command *must* be running while this command executes.
 *
 * @param [in] aRfHandle The handle for the RF core client
 * @param [in] aEntryNo  The index of the entry to alter
 * @param [in] aType     TRUE: the entry is a short address
 *                       FALSE: the entry is an extended address
 * @param [in] aEnable   Whether the given entry is to be enabled or disabled
 *
 * @return the value from the command status register
 * @retval RF_StatCmdDoneSuccess the command completed correctly
 */
static RF_Stat rfCoreModifySourceMatchEntry(RF_Handle             aRfHandle,
                                            uint8_t               aEntryNo,
                                            platformRadio_address aType,
                                            bool                  aEnable)
{
    /* memset kept to save 60 bytes of text space, gcc can't optimize the
     * following bitfield operation if it doesn't know the fields are zero
     * already.
     */
    memset((void *)&sModifyReceiveSrcMatchCmd, 0, sizeof(sModifyReceiveSrcMatchCmd));

    sModifyReceiveSrcMatchCmd.commandNo = CMD_IEEE_MOD_SRC_MATCH;

    /* we only use source matching for pending data bit, so enabling and
     * pending are the same to us.
     */
    if (aEnable)
    {
        sModifyReceiveSrcMatchCmd.options.bEnable = 1;
        sModifyReceiveSrcMatchCmd.options.srcPend = 1;
    }
    else
    {
        sModifyReceiveSrcMatchCmd.options.bEnable = 0;
        sModifyReceiveSrcMatchCmd.options.srcPend = 0;
    }

    sModifyReceiveSrcMatchCmd.options.entryType = aType;
    sModifyReceiveSrcMatchCmd.entryNo           = aEntryNo;

    return RF_runImmediateCmd(aRfHandle, (uint32_t *)&sModifyReceiveSrcMatchCmd);
}

/**
 * @brief walks the short address source match list to find an address
 *
 * @param [in] aAddress the short address to search for
 *
 * @return the index where the address was found
 * @retval PLATFORM_RADIO_SRC_MATCH_NONE the address was not found
 */
static uint8_t rfCoreFindShortSrcMatchIdx(const uint16_t aAddress)
{
    uint8_t i;

    for (i = 0; i < PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM; i++)
    {
        if (sSrcMatchShortData.shortAddrEnt[i].shortAddr == aAddress)
        {
            return i;
        }
    }

    return PLATFORM_RADIO_SRC_MATCH_NONE;
}

/**
 * @brief walks the short address source match list to find an empty slot
 *
 * @return the index of an unused address slot
 * @retval PLATFORM_RADIO_SRC_MATCH_NONE no unused slots available
 */
static uint8_t rfCoreFindEmptyShortSrcMatchIdx(void)
{
    uint8_t i;

    for (i = 0; i < PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM; i++)
    {
        if ((sSrcMatchShortData.srcMatchEn[i / 32] & (1 << (i % 32))) == 0u)
        {
            return i;
        }
    }

    return PLATFORM_RADIO_SRC_MATCH_NONE;
}

/**
 * @brief walks the ext address source match list to find an address
 *
 * @param [in] aAddress the ext address to search for
 *
 * @return the index where the address was found
 * @retval PLATFORM_RADIO_SRC_MATCH_NONE the address was not found
 */
static uint8_t rfCoreFindExtSrcMatchIdx(const uint64_t aAddress)
{
    uint8_t i;

    for (i = 0; i < PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM; i++)
    {
        if (sSrcMatchExtData.extAddrEnt[i] == aAddress)
        {
            return i;
        }
    }

    return PLATFORM_RADIO_SRC_MATCH_NONE;
}

/**
 * @brief walks the ext address source match list to find an empty slot
 *
 * @return the index of an unused address slot
 * @retval PLATFORM_RADIO_SRC_MATCH_NONE no unused slots available
 */
static uint8_t rfCoreFindEmptyExtSrcMatchIdx(void)
{
    uint8_t i;

    for (i = 0; i < PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM; i++)
    {
        if ((sSrcMatchExtData.srcMatchEn[i / 32] & (1 << (i % 32))) == 0u)
        {
            return i;
        }
    }

    return PLATFORM_RADIO_SRC_MATCH_NONE;
}

/**
 * Callback for the TX command chain.
 *
 * This callback is called on completion of the command string or failure of
 * an individual command in the string.
 *
 * @param [in] aRfHandle    Handle of the RF object
 * @param [in] aRfCmdHandle Handle of the command chain that finished
 * @param [in] aRfEventMask Events that happened to trigger this callback
 */
static void rfCoreTxCallback(RF_Handle aRfHandle, RF_CmdHandle aRfCmdHandle, RF_EventMask aRfEventMask)
{
    unsigned int evts = 0U;

    if (aRfEventMask & (RF_EventCmdPreempted))
    {
#ifdef USE_DMM
        dmmSetActivityTrackingTx(false);
#endif
        evts |= RF_EVENT_TX_CMD_PREEMPTED;
    }
    else if (aRfEventMask & (RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdStopped))
    {
        /* General failure of the command string, notify processing loop.
         * Likely the TX was aborted to return to RX for some reason.
         */
        sTransmitError = OT_ERROR_ABORT;
        evts |= RF_EVENT_TX_DONE;
    }
    else if (aRfEventMask & (RF_EventLastFGCmdDone | RF_EventLastCmdDone))
    {
        if (sTransmitCmd.status == IEEE_DONE_OK)
        {
            /* The CSMA-CA command completed successfully and the transmit
             * command completed successfully, successful transmission */
            sTransmitError = OT_ERROR_NONE;
        }
        else
        {
            sTransmitError = OT_ERROR_CHANNEL_ACCESS_FAILURE;
        }

        evts |= RF_EVENT_TX_DONE;

#ifdef USE_DMM
        if (sTransmitError == OT_ERROR_NONE)
        {
            dmmSetActivityTrackingTx(true);
        }
        else
        {
            dmmSetActivityTrackingTx(false);
        }
#endif
    }

    /* tell radio processing loop what happened */
    radioSignal(evts);
}

/**
 * @brief sends the tx command to the radio core
 *
 * Sends the packet to the radio core to be sent asynchronously.
 *
 * @param [in] aRfHandle The handle for the RF core client
 * @param [in] aPsdu     A pointer to the data to be sent
 * @note this *must* be 4 byte aligned and not include the FCS, that is
 * calculated in hardware.
 * @param [in] aLen      The length in bytes of data pointed to by psdu.
 *
 * @return handle of the running command returned by the command scheduler
 */
static RF_CmdHandle rfCoreSendTransmitCmd(otInstance *aInstance, RF_Handle aRfHandle, otRadioFrame *aFrame)
{
    RF_ScheduleCmdParams rfScheduleCmdParams;

    RF_ScheduleCmdParams_init(&rfScheduleCmdParams);
    sTransmitCmd = RF_cmdIeeeTx;

    // remove 2 octets for CRC, generated by hardware
    sTransmitCmd.payloadLen = aFrame->mLength - 2;
    sTransmitCmd.pPayload   = aFrame->mPsdu;

    if (0U != aFrame->mInfo.mTxInfo.mTxDelay)
    {
// Number of RAT ticks from the command beginning to symbols being sent over the air
#define RF_CORE_TX_CMD_LATENCY (320) // 80 uS: TX synth lock guard time
        sTransmitCmd.startTime =
            RF_convertUsToRatTicks(aFrame->mInfo.mTxInfo.mTxDelayBaseTime + aFrame->mInfo.mTxInfo.mTxDelay) -
            RF_CORE_TX_CMD_LATENCY;
        sTransmitCmd.startTrigger.triggerType = TRIG_ABSTIME;
        sTransmitCmd.startTrigger.pastTrig    = 0;
        rfScheduleCmdParams.startTime         = sTransmitCmd.startTime;
        rfScheduleCmdParams.startType         = RF_StartAbs;
        // rfScheduleCmdParams.allowDelay        = RF_AllowDelayNone;
        rfScheduleCmdParams.allowDelay = RF_AllowDelayAny; // lie to get our command scheduled
    }
    else
    {
        sTransmitCmd.startTime                = 0U;
        sTransmitCmd.startTrigger.triggerType = TRIG_NOW;
        sTransmitCmd.startTrigger.pastTrig    = 1; // XXX: workaround for RF scheduler
        rfScheduleCmdParams.startTime         = 0;
        rfScheduleCmdParams.startType         = RF_StartNotSpecified;
        rfScheduleCmdParams.allowDelay        = RF_AllowDelayAny;
    }

    sTransmitCmd.condition.rule = COND_NEVER;

#ifdef USE_DMM
    dmmSetThreadActivityTx(aInstance, (aFrame->mPsdu[0] & IEEE802154_FRAME_PENDING));
    rfScheduleCmdParams.activityInfo = dmmGetActivityPriorityTx();
#endif
    rfScheduleCmdParams.coexPriority = sPriorityCoex;
    rfScheduleCmdParams.coexRequest  = sRequestCoex;

    /* no error has occurred (yet) */
    sTransmitError = OT_ERROR_NONE;

    return RF_scheduleCmd(aRfHandle, (RF_Op *)&sTransmitCmd, &rfScheduleCmdParams, rfCoreTxCallback,
                          RF_EventLastFGCmdDone);
}

/* Forward definition for RX callback */
static void rfCoreSendReceiveCmd(RF_Handle aRfHandle);

/**
 * Callback for the receive command.
 *
 *
 *
 * @param [in] aRfHandle    Handle of the RF object
 * @param [in] aRfCmdHandle Handle of the command chain that finished
 * @param [in] aRfEventMask Events that happened to trigger this callback
 */
static void rfCoreRxCallback(RF_Handle aRfHandle, RF_CmdHandle aRfCmdHandle, RF_EventMask aRfEventMask)
{
    unsigned int evts = 0U;

    if (aRfEventMask & RF_EventRxBufFull)
    {
        evts |= RF_EVENT_BUF_FULL;
    }

    if (aRfEventMask & RF_EventRxCtrl)
    {
        evts |= RF_EVENT_RX_FRM_FILT;
    }

    if (aRfEventMask & RF_EventTXAck)
    {
        /* A packet was received, that packet required an ACK and the
         * ACK has been transmitted
         */
        evts |= RF_EVENT_RX_ACK_DONE;
    }

    if (aRfEventMask & RF_EventRxEntryDone)
    {
        /* A packet was received the packet MAY require an ACK Or the
         * packet might not (ie: a broadcast)
         */
        evts |= RF_EVENT_RX_DONE;
    }

    else if (aRfEventMask & ((RF_EventLastCmdDone) | (RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdStopped)))
    {
        /* Clean up command handle in terminating event */
        sReceiveCmdHandle = (RF_CmdHandle)RF_ALLOC_ERROR;

        /* The RX command is stopped. This may be due to a change in the RX
         * command or the RF driver might have stopped the command. Abort
         * conditions are delivered separately from regular last command done
         * events but are handled the same way.
         */
        evts |= RF_EVENT_RX_CMD_STOP;
    }

    if (aRfEventMask & RF_EventCmdPreempted)
    {
        /* Clean up command handle in terminating event */
        sReceiveCmdHandle = (RF_CmdHandle)RF_ALLOC_ERROR;
#ifdef USE_DMM
        dmmSetActivityTrackingRx(false);
#endif
        /*
         * Re-submission of the RX command is done in the RX callback to try
         * and avoid the TX command being submitted without a background RX
         * command.
         */

        rfCoreSendReceiveCmd(sRfHandle);
    }
    else
    {
#ifdef USE_DMM
        dmmSetActivityTrackingRx(true);
#endif
    }

    /* tell radio processing loop what happened */
    radioSignal(evts);
}

/**
 * @brief sends the rx command to the radio core
 *
 * Sends the pre-built receive command to the radio core. This sets up the
 * radio to receive packets according to the settings in the global rx command.
 *
 * @note This function does not alter any of the parameters of the rx command.
 * It is only concerned with sending the command to the radio core. See @ref
 * otPlatRadioSetPanId for an example of how the rx settings are set changed.
 *
 * @param [in] aRfHandle The handle for the RF core client
 *
 * @return handle of the running command returned by the command scheduler
 */
static void rfCoreSendReceiveCmd(RF_Handle aRfHandle)
{
    RF_ScheduleCmdParams rfScheduleCmdParams;

    if (sReceiveCmdHandle < 0)
    {
        RF_ScheduleCmdParams_init(&rfScheduleCmdParams);
#ifdef USE_DMM
        dmmSetThreadActivityRx(OtInstance_get(), false);

        rfScheduleCmdParams.activityInfo = dmmGetActivityPriorityRx();
#endif
        rfScheduleCmdParams.coexPriority = sPriorityCoex;
        rfScheduleCmdParams.coexRequest  = sRequestCoex;

        sReceiveCmd.status = IDLE;

        sReceiveCmdHandle = RF_scheduleCmd(
            aRfHandle, (RF_Op *)&sReceiveCmd, &rfScheduleCmdParams, rfCoreRxCallback,
            (RF_EventLastCmdDone | RF_EventRxEntryDone | RF_EventTXAck | RF_EventRxBufFull | RF_EventRxCtrl));
    }
}

/**
 * @brief Sets the transmit.
 *
 * Sets the transmit power within the radio setup command or the override list.
 */
static otError rfCoreSetTransmitPower(int8_t aPower)
{
    otError               retval = OT_ERROR_NONE;
    RF_TxPowerTable_Value oldValue;
    RF_TxPowerTable_Value newValue;
    unsigned int          i;

    /* search for a matching backoff if there is one */
    for (i = 0; i < (sizeof(cTxMaxPower) / sizeof(cTxMaxPower[0])); i++)
    {
        if (cTxMaxPower[i].channel == sReceiveCmd.channel && cTxMaxPower[i].maxPower < aPower)
        {
            /* drop aPower if it is above the channel's max power */
            aPower = cTxMaxPower[i].maxPower;
        }
    }

    /* find the tx power configuration */
    newValue = RF_TxPowerTable_findValue(txPowerTable, aPower);
    oldValue = RF_getTxPower(sRfHandle);
    otEXPECT_ACTION(RF_TxPowerTable_INVALID_VALUE != newValue.rawValue, retval = OT_ERROR_INVALID_ARGS);

    /* set the tx power configuration */
    if (platformRadio_phyState_Sleep == sState || platformRadio_phyState_Disabled == sState ||
        newValue.paType == oldValue.paType)
    {
        otEXPECT_ACTION(RF_StatSuccess == RF_setTxPower(sRfHandle, newValue), retval = OT_ERROR_FAILED);
    }
    else
    {
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);

        otEXPECT_ACTION(RF_StatSuccess == RF_setTxPower(sRfHandle, newValue), retval = OT_ERROR_FAILED);

        rfCoreSendReceiveCmd(sRfHandle);
        otEXPECT_ACTION(sReceiveCmdHandle >= 0, retval = OT_ERROR_FAILED);
    }

exit:
    return retval;
}

/**
 * @brief Callback for the Energy Detect command.
 *
 * @param [in] aRfHandle    Handle of the RF object
 * @param [in] aRfCmdHandle Handle of the command chain that finished
 * @param [in] aRfEventMask Events that happened to trigger this callback
 */
static void rfCoreEdScanCmdCallback(RF_Handle aRfHandle, RF_CmdHandle aRfCmdHandle, RF_EventMask aRfEventMask)
{
    radioSignal(RF_EVENT_ED_SCAN_DONE);
}

/**
 * @brief sends the energy detect scan command to the radio core
 *
 * Sends the Energy Detect scan command to the radio core. This scans the given
 * channel for activity.
 *
 * @param [in] aRfHandle The handle for the RF core client
 * @param [in] aChannel  The IEEE page 0 channel to scan
 * @param [in] aDuration Time in ms to scan
 *
 * @return handle of the running command returned by the command scheduler
 */
static RF_CmdHandle rfCoreSendEdScanCmd(RF_Handle aRfHandle, uint8_t aChannel, uint16_t aDuration)
{
    RF_ScheduleCmdParams rfScheduleCmdParams;
    RF_ScheduleCmdParams_init(&rfScheduleCmdParams);

#ifdef USE_DMM
    rfScheduleCmdParams.activityInfo = dmmGetActivityPriorityRx();
#endif
    rfScheduleCmdParams.coexPriority = sPriorityCoex;
    rfScheduleCmdParams.coexRequest  = sRequestCoex;

    sEdScanCmd = RF_cmdIeeeEdScan;

    sEdScanCmd.channel    = aChannel;
    sEdScanCmd.ccaRssiThr = -90;

    sEdScanCmd.startTrigger.pastTrig = 1; // XXX: workaround for RF scheduler
    sEdScanCmd.condition.rule        = COND_NEVER;

    sEdScanCmd.ccaOpt.ccaEnEnergy = 1;
    sEdScanCmd.ccaOpt.ccaEnCorr   = 1;
    sEdScanCmd.ccaOpt.ccaEnSync   = 1;
    sEdScanCmd.ccaOpt.ccaCorrOp   = 1;
    sEdScanCmd.ccaOpt.ccaSyncOp   = 0;
    sEdScanCmd.ccaOpt.ccaCorrThr  = 3;

    sEdScanCmd.endTrigger.triggerType = TRIG_REL_START;
    sEdScanCmd.endTrigger.pastTrig    = 1;

    /* duration is in ms */
    sEdScanCmd.endTime = RF_convertUsToRatTicks(aDuration * 1000);

    return RF_scheduleCmd(aRfHandle, (RF_Op *)&sEdScanCmd, &rfScheduleCmdParams, rfCoreEdScanCmdCallback,
                          RF_EventLastCmdDone);
}

/**
 * @brief Callback for the Transmit Test command.
 *
 * @param [in] aRfHandle    Handle of the RF object
 * @param [in] aRfCmdHandle Handle of the command chain that finished
 * @param [in] aRfEventMask Events that happened to trigger this callback
 */
static void rfCoreTxTestCmdCallback(RF_Handle aRfHandle, RF_CmdHandle aRfCmdHandle, RF_EventMask aRfEventMask)
{
    (void)aRfHandle;
    (void)aRfCmdHandle;
    (void)aRfEventMask;

    return;
}

/**
 * @param [in] aRfHandle  The handle for the RF core client
 * @param [in] aModulated TRUE: Send modulated word
 *                        FALSE: Send unmodulated word
 *
 * @return handle of the running command returned by the command scheduler
 */
static RF_CmdHandle rfCoreSendTxTestCmd(RF_Handle aRfHandle, bool aModulated)
{
    RF_ScheduleCmdParams rfScheduleCmdParams;
    RF_ScheduleCmdParams_init(&rfScheduleCmdParams);

    rfScheduleCmdParams.coexPriority = sPriorityCoex;
    rfScheduleCmdParams.coexRequest  = sRequestCoex;

    sTxTestCmd = RF_cmdTxTest;

    sTxTestCmd.startTrigger.pastTrig = 1; // XXX: workaround for RF scheduler
    sTxTestCmd.condition.rule        = COND_NEVER;

    if (aModulated)
    {
        sTxTestCmd.txWord = PLATFORM_RADIO_TX_TEST_MODULATED_WORD;
    }
    else
    {
        sTxTestCmd.config.bUseCw = 1;
        sTxTestCmd.txWord        = PLATFORM_RADIO_TX_TEST_UNMODULATED_WORD;
    }

    return RF_scheduleCmd(aRfHandle, (RF_Op *)&sTxTestCmd, &rfScheduleCmdParams, rfCoreTxTestCmdCallback,
                          RF_EventLastCmdDone);
}

/**
 * Default error callback for RF Driver.
 *
 * Errors are unlikely, and fatal.
 */
static void rfCoreErrorCallback(RF_Handle aHandle, RF_CmdHandle aCmdHandle, RF_EventMask aEvents)
{
    while (1)
        ;
}

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

void rfCorePriorityCoex(bool aEnable)
{
    if (aEnable)
    {
        sPriorityCoex = RF_PriorityCoexHigh;
    }
    else
    {
        sPriorityCoex = RF_PriorityCoexLow;
    }
}

void rfCoreRequestCoex(bool aEnable)
{
    if (aEnable)
    {
        sRequestCoex = RF_RequestCoexAssertRx;
    }
    else
    {
        sRequestCoex = RF_RequestCoexNoAssertRx;
    }
}

void platformRadioInit(void)
{
    rfCoreInitBufs();
    rfCoreInitSetupCmd();
    /* Populate the RX parameters data structure with default values */
    rfCoreInitReceiveParams();

    sState = platformRadio_phyState_Disabled;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    otError   error = OT_ERROR_BUSY;
    RF_Params rfParams;
    (void)aInstance;

    if (sState == platformRadio_phyState_Sleep)
    {
        error = OT_ERROR_NONE;
    }
    else if (sState == platformRadio_phyState_Disabled)
    {
        RF_Params_init(&rfParams);

        rfParams.pErrCb = rfCoreErrorCallback;
        rfParams.nID    = RF_STACK_ID_THREAD;
        sRfHandle       = RF_open(&sRfObject, &RF_prop, (RF_RadioSetup *)&sRadioSetupCmd, &rfParams);

        otEXPECT_ACTION(sRfHandle != NULL, error = OT_ERROR_FAILED);
        sState = platformRadio_phyState_Sleep;

        error = OT_ERROR_NONE;
    }

exit:
    if (error == OT_ERROR_FAILED)
    {
        sState = platformRadio_phyState_Disabled;
    }

    return error;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;
    return (sState != platformRadio_phyState_Disabled);
}

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
        RF_close(sRfHandle);
        sState = platformRadio_phyState_Disabled;
        error  = OT_ERROR_NONE;
    }

    return error;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;

    switch (sState)
    {
    case platformRadio_phyState_Receive:
    {
        sState = platformRadio_phyState_EdScan;
        /* abort receive */
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);
        /* fall through */
    }

    case platformRadio_phyState_Sleep:
    {
        sState = platformRadio_phyState_EdScan;
        otEXPECT_ACTION(rfCoreSendEdScanCmd(sRfHandle, aScanChannel, aScanDuration) >= 0, error = OT_ERROR_FAILED);
        break;
    }

    default:
    {
        error = OT_ERROR_BUSY;
        break;
    }
    }

exit:
    if (OT_ERROR_NONE != error)
    {
        sState = platformRadio_phyState_Sleep;
    }
    return error;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    (void)aInstance;
    /* update the tracking variable */
    sReqTxPower = aPower;
    /* set the power */
    return rfCoreSetTransmitPower(aPower);
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;

    otEXPECT_ACTION(aPower != NULL, error = OT_ERROR_INVALID_ARGS);
    *aPower = RF_TxPowerTable_findPowerLevel(txPowerTable, RF_getTxPower(sRfHandle));

exit:
    return error;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    return otPlatRadioReceiveAt(aInstance, aChannel, 0, UINT32_MAX);
}

OT_TOOL_WEAK otError otPlatRadioReceiveAt(otInstance *aInstance, uint8_t aChannel, uint32_t aStart, uint32_t aDuration)
{
    otError error = OT_ERROR_BUSY;
    (void)aInstance;

    if (sState == platformRadio_phyState_Transmit)
    {
        sState = platformRadio_phyState_Receive;
        rfCoreExecuteAbortCmd(sRfHandle, sTransmitCmdHandle);
    }
    // NOTE: no else, process receive after if channel has changed

    if (sState == platformRadio_phyState_Sleep)
    {
        /* initialize the receive command
         *
         * no memset here because we assume init has been called and we may
         * have changed some values in the rx command
         */
        if (!sDisableChannel)
        {
            /* If the diag module has not locked out changing the channel */
            sReceiveCmd.channel = aChannel;
        }
        if (aStart == 0U && aDuration == UINT32_MAX)
        {
            sReceiveCmd.startTrigger.triggerType = TRIG_NOW;
            sReceiveCmd.endTrigger.triggerType   = TRIG_NEVER;
            sReceiveCmd.startTime                = 0U;
            sReceiveCmd.endTime                  = 0U;
        }
        else
        {
            sReceiveCmd.startTrigger.triggerType = TRIG_ABSTIME;
            sReceiveCmd.endTrigger.triggerType   = TRIG_REL_START;
            sReceiveCmd.startTime                = RF_convertUsToRatTicks(aStart);
            sReceiveCmd.endTime                  = RF_convertUsToRatTicks(aDuration);
        }
        /* allow the transmit power helper function to manage the characterized
         * max power.
         */
        rfCoreSetTransmitPower(sReqTxPower);
        /* send the command to the radio */
        rfCoreSendReceiveCmd(sRfHandle);
        otEXPECT_ACTION(sReceiveCmdHandle >= 0, error = OT_ERROR_FAILED);
        /* update the tracking variables */
        sState = platformRadio_phyState_Receive;
        error  = OT_ERROR_NONE;
    }
    else if (sState == platformRadio_phyState_Receive || sState == platformRadio_phyState_RxTxAck)
    {
        if (sReceiveCmd.channel == aChannel || sDisableChannel)
        {
            /* we are already running on the correct channel or the diag module
             * has disallowed switching channels.
             */
            error = OT_ERROR_NONE;
        }
        else
        {
            if (sState == platformRadio_phyState_RxTxAck)
            {
                // aborting the RX command 'should' abort all running FG commands
                rfCoreExecuteAbortCmd(sRfHandle, sTransmitAckCmdHandle);
            }
            rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);

            sReceiveCmd.channel = aChannel;
            if (aStart == 0U && aDuration == UINT32_MAX)
            {
                sReceiveCmd.startTrigger.triggerType = TRIG_NOW;
                sReceiveCmd.endTrigger.triggerType   = TRIG_NEVER;
                sReceiveCmd.startTime                = 0U;
                sReceiveCmd.endTime                  = 0U;
            }
            else
            {
                sReceiveCmd.startTrigger.triggerType = TRIG_ABSTIME;
                sReceiveCmd.endTrigger.triggerType   = TRIG_REL_START;
                sReceiveCmd.startTime                = RF_convertUsToRatTicks(aStart);
                sReceiveCmd.endTime                  = RF_convertUsToRatTicks(aDuration);
            }
            /* allow the transmit power helper function to manage the characterized
             * max power.
             */
            rfCoreSetTransmitPower(sReqTxPower);
            /* send the command to the radio */
            rfCoreSendReceiveCmd(sRfHandle);
            otEXPECT_ACTION(sReceiveCmdHandle >= 0, error = OT_ERROR_FAILED);
            sState = platformRadio_phyState_Receive;
            error  = OT_ERROR_NONE;
        }
    }

exit:
    return error;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    otError error = OT_ERROR_BUSY;
    (void)aInstance;

    if (sState == platformRadio_phyState_Sleep)
    {
        error = OT_ERROR_NONE;
    }
    else if (sState == platformRadio_phyState_Receive)
    {
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);

        sState = platformRadio_phyState_Sleep;

        /* The upper layers like to thrash the interface from RX to sleep.
         * Aborting and restarting the commands wastes time and energy, but
         * can be done as often as requested; yielding the RF driver causes
         * the whole core to be shutdown. Delay yield until the rf processing
         * loop to make sure we actually want to sleep.
         */
        radioSignal(RF_EVENT_SLEEP_YIELD);
        error = OT_ERROR_NONE;
    }

    return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void)aInstance;
    return &sTransmitFrame;
}

/**
 * This function is designed to generate an empty Enh-Ack frame to enable
 * scheduling the necessary TX command. This is highly specific to Thread and
 * should not be used as a general solution.
 */
static otError rfCoreGenerateEmptyEnhAck(otRadioFrame *aRxFrame, otRadioFrame *aAckFrame)
{
    uint16_t     fcf     = *((uint16_t *)(&aRxFrame->mPsdu[0]));
    uint8_t      ieLen   = 0;
    otError      ret     = OT_ERROR_NONE;
    otMacAddress srcAddr = {0};
    uint8_t      len     = 0;

    // FCF length
    len += sizeof(uint16_t);
    // sequence Number
    len += sizeof(uint8_t);

    // PAN id compression
    if (0U == (fcf & (1 << 6)))
    {
        // PAN id
        len += sizeof(otPanId);
    }

    // dst Addr
    otEXPECT_ACTION(OT_ERROR_NONE == otMacFrameGetSrcAddr(aRxFrame, &srcAddr), ret = OT_ERROR_PARSE);
    switch (srcAddr.mType)
    {
    case OT_MAC_ADDRESS_TYPE_NONE:
        break;

    case OT_MAC_ADDRESS_TYPE_SHORT:
        len += sizeof(otShortAddress);
        break;

    case OT_MAC_ADDRESS_TYPE_EXTENDED:
        len += sizeof(otExtAddress);
        break;

    default:
        otEXPECT_ACTION(false, ret = OT_ERROR_PARSE);
    }

    // security
    if (otMacFrameIsSecurityEnabled(aRxFrame))
    {
        // do not look at the aux-sec header, has not been received
        //otEXPECT_ACTION(otMacFrameIsKeyIdMode1(aRxFrame), ret = OT_ERROR_PARSE);

        // Thread only uses Security level 5
        len += sizeof(uint8_t);  // Sec control field
        len += sizeof(uint32_t); // Frame Counter
        len += sizeof(uint8_t);  // key index
        len += sizeof(uint32_t); // MAC MIC
    }

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (sCslPeriod > 0)
    {
        ieLen += sizeof(uint16_t); // IE Header
        ieLen += sizeof(uint16_t); // Phase
        ieLen += sizeof(uint16_t); // Period
    }
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    {
        ieLen += otLinkMetricsEnhAckGetDataLen(&srcAddr);
    }
#endif

    if (ieLen > 0)
    {
        len += ieLen;
        // not used in Thread Acks, no body
        //len += sizeof(uint16_t) // Header termination IE
    }

    len += sizeof(uint16_t); // FCS

    aAckFrame->mChannel = aRxFrame->mChannel;
    aAckFrame->mLength = len;
    memset(aAckFrame->mPsdu, 0, len);

exit:
    return ret;
}

static otError rfCoreGenerateEnhAck(otRadioFrame *aRxFrame, otRadioFrame *aAckFrame)
{
    uint8_t ackIeData[OT_ACK_IE_MAX_SIZE];
    uint8_t ackIeDataLength = 0;
    otError ret = OT_ERROR_NONE;

    // handle TX ACK on our own
    sReceiveCmd.frameFiltOpt.autoAckEn = 0;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (sCslPeriod > 0)
    {
        ackIeDataLength += otMacFrameGenerateCslIeTemplate(ackIeData);
    }
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    {
        uint8_t      linkMetricsData[OT_ENH_PROBING_IE_DATA_MAX_SIZE];
        uint8_t      linkMetricsIeDataLen = 0;
        otMacAddress srcAddr              = {0};

        otEXPECT_ACTION(OT_ERROR_NONE == otMacFrameGetSrcAddr(aRxFrame, &srcAddr), ret = OT_ERROR_PARSE);
        linkMetricsIeDataLen = otLinkMetricsEnhAckGenData(&srcAddr, aRxFrame->mInfo.mRxInfo.mLqi,
                                                          aRxFrame->mInfo.mRxInfo.mRssi, linkMetricsData);
        if (linkMetricsIeDataLen > 0)
        {
            ackIeDataLength += otMacFrameGenerateEnhAckProbingIe(ackIeData, linkMetricsData, linkMetricsIeDataLen);
        }
    }
#endif
    otEXPECT_ACTION(OT_ERROR_NONE == otMacFrameGenerateEnhAck(aRxFrame, aRxFrame->mInfo.mRxInfo.mAckedWithFramePending, ackIeData, ackIeDataLength,
                             aAckFrame), ret = OT_ERROR_PARSE);

exit:
    return ret;
}

static otError rfCoreProcessTransmitSecurity(otRadioFrame *aFrame)
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

    uint8_t *localAddr = (uint8_t *)&sReceiveCmd.localExtAddr;
    for (size_t i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
    {
        // reverse
        extAddr.m8[i] = localAddr[OT_EXT_ADDRESS_SIZE - 1 - i];
    }

    otMacFrameProcessTransmitAesCcm(aFrame, &extAddr);
    aFrame->mInfo.mTxInfo.mIsSecurityProcessed = true;

exit:
    return error;
}

void updateIeInfoTxFrame(otInstance *aInstance, otRadioFrame *aFrame)
{
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
    if (sCslPeriod > 0)
    {
        otMacFrameSetCslIe(aFrame, (uint16_t)sCslPeriod, getCslPhase(aFrame));
        aFrame->mInfo.mTxInfo.mCslPresent = true;
    }
#endif // OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    otError error = OT_ERROR_BUSY;

    if (sState == platformRadio_phyState_RxTxAck)
    {
        // TODO queue after TX ACK or should return a channel access error
        rfCoreExecuteAbortCmd(sRfHandle, sTransmitAckCmdHandle);
        sState = platformRadio_phyState_Receive;
    }

    if (sState == platformRadio_phyState_Receive)
    {
        sState = platformRadio_phyState_Transmit;

        if (!aFrame->mInfo.mTxInfo.mIsHeaderUpdated)
        {
            updateIeInfoTxFrame(aInstance, aFrame);
            otEXPECT_ACTION(OT_ERROR_NONE == rfCoreProcessTransmitSecurity(aFrame), error = OT_ERROR_FAILED);
            aFrame->mInfo.mTxInfo.mIsHeaderUpdated = true;
        }

        // if the header was updated but security was not processed, for some reason
        if (!aFrame->mInfo.mTxInfo.mIsSecurityProcessed)
        {
            otEXPECT_ACTION(OT_ERROR_NONE == rfCoreProcessTransmitSecurity(aFrame), error = OT_ERROR_FAILED);
        }

        sTransmitCmdHandle = rfCoreSendTransmitCmd(aInstance, sRfHandle, aFrame);
        otEXPECT_ACTION(sTransmitCmdHandle >= 0, error = OT_ERROR_FAILED);
        error = OT_ERROR_NONE;
        otPlatRadioTxStarted(aInstance, aFrame);
    }

exit:
    return error;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void)aInstance;
    return sRfStats.maxRssi;
}

#ifdef __TI_ARM__
/*
 * ti-cgt warns about using enums as bitfields because by default all enums
 * are packed to save space. We could pass the `--enum_type=int` switch to the
 * compiler, but this creates linking errors with our board support libraries.
 *
 * Instead we disable `#190-D enumerated type mixed with another type`.
 */
#pragma diag_push
#pragma diag_suppress 190
#endif
otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void)aInstance;
    return
        /* DISABLED: Enhanced Protocol Features
         * Changes to enable IEEE 802.15.4-2015 Enh-Ack make using the built in
         * protocol features of the RF Core difficult. This may be enabled in a
         * future release.
         *  OT_RADIO_CAPS_ACK_TIMEOUT      |
         *  OT_RADIO_CAPS_TRANSMIT_RETRIES |
         *  OT_RADIO_CAPS_CSMA_BACKOFF     |
         */
        /* DISABLED: Hardware Limitation
         * The RF Core must have a running backgrounded RX command to TX. Let
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
            OT_RADIO_CAPS_ENERGY_SCAN      |
            OT_RADIO_CAPS_TRANSMIT_SEC     |
            OT_RADIO_CAPS_TRANSMIT_TIMING  |
            OT_RADIO_CAPS_NONE;
}
#ifdef __TI_ARM__
#pragma diag_pop
#endif

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;

    /* verify that we have a running or backgrounded rx command */
    if (rfCoreModifyRxAutoPend(sRfHandle, aEnable) != RF_StatCmdDoneSuccess)
    {
        /* if we are promiscuous, then auto pend should be disabled */
        sReceiveCmd.frameFiltOpt.autoPendEn = aEnable ? 1 : 0;
    }

    return;
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;
    uint8_t idx = rfCoreFindShortSrcMatchIdx(aShortAddress);

    if (idx == PLATFORM_RADIO_SRC_MATCH_NONE)
    {
        /* the entry does not exist already, add it */
        otEXPECT_ACTION((idx = rfCoreFindEmptyShortSrcMatchIdx()) != PLATFORM_RADIO_SRC_MATCH_NONE,
                        error = OT_ERROR_NO_BUFS);
        sSrcMatchShortData.shortAddrEnt[idx].shortAddr = aShortAddress;
        sSrcMatchShortData.shortAddrEnt[idx].panId     = sReceiveCmd.localPanID;
    }

    /* attempt to modify running or backgrounded rx command */
    if (rfCoreModifySourceMatchEntry(sRfHandle, idx, platformRadio_address_short, true) != RF_StatCmdDoneSuccess)
    {
        /* we are not running, so we must update the values ourselves */
        sSrcMatchShortData.srcPendEn[idx / 32] |= (1 << (idx % 32));
        sSrcMatchShortData.srcMatchEn[idx / 32] |= (1 << (idx % 32));
    }

exit:
    return error;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    otError error = OT_ERROR_NONE;
    (void)aInstance;
    uint8_t idx;
    otEXPECT_ACTION((idx = rfCoreFindShortSrcMatchIdx(aShortAddress)) != PLATFORM_RADIO_SRC_MATCH_NONE,
                    error = OT_ERROR_NO_ADDRESS);

    /* verify that we have a running or backgrounded rx command */
    if (rfCoreModifySourceMatchEntry(sRfHandle, idx, platformRadio_address_short, false) != RF_StatCmdDoneSuccess)
    {
        /* we are not running, so we must update the values ourselves */
        sSrcMatchShortData.srcPendEn[idx / 32] &= ~(1 << (idx % 32));
        sSrcMatchShortData.srcMatchEn[idx / 32] &= ~(1 << (idx % 32));
    }

exit:
    return error;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    (void)aInstance;

    otError  error      = OT_ERROR_NONE;
    uint64_t local_addr = *((uint64_t *)aExtAddress); // ensures alignment
    uint8_t  idx        = rfCoreFindExtSrcMatchIdx(local_addr);

    if (idx == PLATFORM_RADIO_SRC_MATCH_NONE)
    {
        /* the entry does not exist already, add it */
        otEXPECT_ACTION((idx = rfCoreFindEmptyExtSrcMatchIdx()) != PLATFORM_RADIO_SRC_MATCH_NONE,
                        error = OT_ERROR_NO_BUFS);
        sSrcMatchExtData.extAddrEnt[idx] = *((uint64_t *)aExtAddress);
    }

    /* verify that we have a running or backgrounded rx command */
    if (rfCoreModifySourceMatchEntry(sRfHandle, idx, platformRadio_address_ext, true) != RF_StatCmdDoneSuccess)
    {
        /* we are not running, so we must update the values ourselves */
        sSrcMatchExtData.srcPendEn[idx / 32] |= (1 << (idx % 32));
        sSrcMatchExtData.srcMatchEn[idx / 32] |= (1 << (idx % 32));
    }

exit:
    return error;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    (void)aInstance;

    otError  error      = OT_ERROR_NONE;
    uint64_t local_addr = *((uint64_t *)aExtAddress); // ensures alignment
    uint8_t  idx;
    otEXPECT_ACTION(idx   = rfCoreFindExtSrcMatchIdx(local_addr) != PLATFORM_RADIO_SRC_MATCH_NONE,
                    error = OT_ERROR_NO_ADDRESS);

    /* verify that we have a running or backgrounded rx command */
    if (rfCoreModifySourceMatchEntry(sRfHandle, idx, platformRadio_address_ext, false) != RF_StatCmdDoneSuccess)
    {
        /* we are not running, so we must update the values ourselves */
        sSrcMatchExtData.srcPendEn[idx / 32] &= ~(1 << (idx % 32));
        sSrcMatchExtData.srcMatchEn[idx / 32] &= ~(1 << (idx % 32));
    }

exit:
    return error;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;

    if (sState == platformRadio_phyState_Receive || sState == platformRadio_phyState_Transmit)
    {
        unsigned int i;
        for (i = 0; i < PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM; i++)
        {
            /* we have a running or backgrounded rx command */
            otEXPECT(rfCoreModifySourceMatchEntry(sRfHandle, i, platformRadio_address_short, false) == CMDSTA_Done);
        }
    }
    else
    {
        /* we are not running, so we can erase them ourselves */
        memset((void *)&sSrcMatchShortData, 0, sizeof(sSrcMatchShortData));
    }
exit:
    return;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;

    if (sState == platformRadio_phyState_Receive || sState == platformRadio_phyState_Transmit)
    {
        unsigned int i;
        for (i = 0; i < PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM; i++)
        {
            /* we have a running or backgrounded rx command */
            otEXPECT(rfCoreModifySourceMatchEntry(sRfHandle, i, platformRadio_address_ext, false) == CMDSTA_Done);
        }
    }
    else
    {
        /* we are not running, so we can erase them ourselves */
        memset((void *)&sSrcMatchExtData, 0, sizeof(sSrcMatchExtData));
    }
exit:
    return;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void)aInstance;
    /* we are promiscuous if we are not filtering */
    return sReceiveCmd.frameFiltOpt.frameFiltEn == 0;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;

    if (rfCoreModifyRxFrameFilter(sRfHandle, !aEnable) != RF_StatCmdDoneSuccess)
    {
        sReceiveCmd.frameFiltOpt.frameFiltEn = aEnable ? 0 : 1;
    }

    return;
}

void otPlatRadioSetMacKey(otInstance *            aInstance,
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
    case platformRadio_phyState_EdScan:
        return OT_RADIO_STATE_RECEIVE;

    case platformRadio_phyState_Transmit:
        return OT_RADIO_STATE_TRANSMIT;

    default:
        return OT_RADIO_STATE_INVALID;
    }
}

uint64_t otPlatRadioGetNow(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    /*
     * It may be more accurate to use the RAT, but these values are used to
     * schedule with the uS alarm
     *
    return RF_convertRatTicksToUs(RF_getCurrentTime());
     */

    return otPlatTimeGet();
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    uint8_t *    eui64;
    unsigned int i;
    (void)aInstance;

    /* The IEEE MAC address can be stored two places. We check the Customer
     * Configuration was not set before defaulting to the Factory
     * Configuration.
     */
    eui64 = (uint8_t *)(CCFG_BASE + CCFG_O_IEEE_MAC_0);

    for (i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
    {
        /* If the EUI is all ones, then the EUI is invalid, or wasn't set. */
        if (eui64[i] != 0xFF)
        {
            break;
        }
    }

    if (i >= OT_EXT_ADDRESS_SIZE)
    {
        /* The ccfg address was all 0xFF, switch to the fcfg */
        eui64 = (uint8_t *)(FCFG1_BASE + FCFG1_O_MAC_15_4_0);
    }

    /* The IEEE MAC address is stored in network byte order (big endian).
     * The caller seems to want the address stored in little endian format,
     * which is backwards of the conventions setup by @ref
     * otPlatRadioSetExtendedAddress. otPlatRadioSetExtendedAddress assumes
     * that the address being passed to it is in network byte order (big
     * endian), so the caller of otPlatRadioSetExtendedAddress must swap the
     * endianness before calling.
     *
     * It may be easier to have the caller of this function store the IEEE
     * address in network byte order (big endian).
     */
    for (i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
    {
        aIeeeEui64[i] = eui64[(OT_EXT_ADDRESS_SIZE - 1) - i];
    }
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanid)
{
    (void)aInstance;

    /* XXX: if the pan id is the broadcast pan id (0xFFFF) the auto ack will
     * not work. This is due to the design of the CM0 and follows IEEE 802.15.4
     */
    if (platformRadio_phyState_Sleep == sState || platformRadio_phyState_Disabled == sState)
    {
        sReceiveCmd.localPanID = aPanid;
    }
    else
    {
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);

        sReceiveCmd.localPanID = aPanid;

        rfCoreSendReceiveCmd(sRfHandle);
    }

    return;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    (void)aInstance;

    if (platformRadio_phyState_Sleep == sState || platformRadio_phyState_Disabled == sState)
    {
        sReceiveCmd.localExtAddr = *((uint64_t *)(aAddress));
    }
    else
    {
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);

        sReceiveCmd.localExtAddr = *((uint64_t *)(aAddress));

        rfCoreSendReceiveCmd(sRfHandle);
    }

    return;
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    (void)aInstance;
    if (platformRadio_phyState_Sleep == sState || platformRadio_phyState_Disabled == sState)
    {
        sReceiveCmd.localShortAddr = aAddress;
    }
    else
    {
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);

        sReceiveCmd.localShortAddr = aAddress;

        rfCoreSendReceiveCmd(sRfHandle);
    }

    return;
}

void otPlatRadioUpdateCslSampleTime(otInstance *aInstance, uint32_t aCslSampleTime)
{
    OT_UNUSED_VARIABLE(aInstance);

    sCslSampleTime = aCslSampleTime;
}

otError otPlatRadioEnableCsl(otInstance *        aInstance,
                             uint32_t            aCslPeriod,
                             otShortAddress      aShortAddr,
                             const otExtAddress *aExtAddr)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aShortAddr);
    OT_UNUSED_VARIABLE(aExtAddr);

    sCslPeriod = aCslPeriod;

    return OT_ERROR_NONE;
}

uint8_t otPlatRadioGetCslAccuracy(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return XTAL_UNCERTAINTY;
}

uint8_t otPlatRadioGetCslUncertainty(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return CSL_TX_UNCERTAINTY;
}

otError otPlatRadioConfigureEnhAckProbing(otInstance *         aInstance,
                                          otLinkMetrics        aLinkMetrics,
                                          const otShortAddress aShortAddress,
                                          const otExtAddress * aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    return otLinkMetricsConfigureEnhAckProbing(aShortAddress, aExtAddress, aLinkMetrics);
}

otError otPlatDiagRadioToneStart(otInstance *aInstance, bool aModulated)
{
    (void)aInstance;
    otError retval = OT_ERROR_NONE;

    otEXPECT_ACTION(platformRadio_phyState_Receive == sState || platformRadio_phyState_Sleep == sState,
                    retval = OT_ERROR_INVALID_STATE);

    if (platformRadio_phyState_Receive == sState)
    {
        /* stop the running receive operation */
        rfCoreExecuteAbortCmd(sRfHandle, sReceiveCmdHandle);
    }

    sTxTestCmdHandle = rfCoreSendTxTestCmd(sRfHandle, aModulated);

    otEXPECT_ACTION(RF_ALLOC_ERROR != sTxTestCmdHandle, retval = OT_ERROR_FAILED);

exit:
    return retval;
}

otError otPlatDiagRadioToneStop(otInstance *aInstance)
{
    (void)aInstance;
    otError retval = OT_ERROR_NONE;

    otEXPECT_ACTION(platformRadio_phyState_Receive == sState || platformRadio_phyState_Sleep == sState,
                    retval = OT_ERROR_INVALID_STATE);

    /* stop the running receive operation */
    rfCoreExecuteAbortCmd(sRfHandle, sTxTestCmdHandle);

    if (platformRadio_phyState_Receive == sState)
    {
        /* re-issue the receive operation */
        rfCoreSendReceiveCmd(sRfHandle);
    }

exit:
    return retval;
}

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

otError otPlatRadioSetRegion(otInstance *aInstance, uint16_t aRegionCode)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aRegionCode);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetRegion(otInstance *aInstance, uint16_t *aRegionCode)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aRegionCode);

    return OT_ERROR_NOT_IMPLEMENTED;
}

#endif /* OPENTHREAD_CONFIG_RADIO_NO_WEAK_DEFINITIONS */


static void platformRadioProcessTransmitDone(otInstance *  aInstance,
                                             otRadioFrame *aTransmitFrame,
                                             otRadioFrame *aAckFrame,
                                             otError       aTransmitError)
{
    /* clear the pseudo-transmit-active flag */
    sTransmitCmd.pPayload = NULL;

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
    }
}

static uint64_t convertRatTimestamp(uint32_t timestamp)
{
    /* The RAT runs at 4MHz and only returns 32 bits. This is not the format
     * that the calling sub_mac expects the timestamps. We construct the full
     * uint64_t based on the system timer even though most of this information
     * is lost. To get a full uint64_t timestamp we; get the current time from
     * both sources, calculate the delta between the RAT values, translate that
     * to uS, and subtract it from the current uS counter.
     *
     * The RAT and RTC are synchronized when the RF core is powered on. This
     * should result in the same 0 across the system timer and the RAT
     * timestamps. It would be possible to use the RAT directly, however the
     * resolution is only 30-bits once converted to uS and may cause errors
     * with the other 32-bit values used elsewhere.
     */
    uint64_t currentSysTime = otPlatRadioGetNow(NULL); // uS
    uint32_t currentRatTime = RF_getCurrentTime();     // RAT Ticks
    uint32_t delta = currentRatTime - timestamp;       // RAT Ticks (modulus uint32_t)

    return currentSysTime - RF_convertRatTicksToUs(delta);
}

static otError populateReceiveFrame(otRadioFrame *aFrame, uint8_t *aData)
{
    int                  infoIdx;
    rfc_ieeeRxCorrCrc_t *crcCorr;

    struct rfPktAdditionalInfo
    {
        uint8_t  rssi;
        uint8_t  crcCorr;
        uint8_t  sourceIndex;
        uint32_t timestamp;
    } __attribute__((__packed__));

    struct rfPktAdditionalInfo *aInfo;

    infoIdx = aData[0] + 1 - sizeof(struct rfPktAdditionalInfo);

    aInfo = (struct rfPktAdditionalInfo *)&(aData[infoIdx]);

    crcCorr = (rfc_ieeeRxCorrCrc_t *)&aInfo->crcCorr;

    /* get SFD ts from the packet */
    aFrame->mInfo.mRxInfo.mTimestamp = convertRatTimestamp(aInfo->timestamp);

    // length of rx element ===== aData[0]
    aFrame->mLength             = aData[1] & 0x7F; // length of rx frame (phy hdr)
    aFrame->mPsdu               = &(aData[2]);     // start of psdu
    aFrame->mChannel            = sReceiveCmd.channel;
    aFrame->mInfo.mRxInfo.mRssi = aInfo->rssi;
    aFrame->mInfo.mRxInfo.mLqi  = crcCorr->status.corr;

    bool pend = false;

    if (aInfo->sourceIndex != 0xff)
    {
        if ((aFrame->mPsdu[1] & IEEE802154_FRAME_PENDING_MASK) >> 6)
        {
            pend = ((sSrcMatchExtData.srcPendEn[aInfo->sourceIndex / 32] & (1 << (aInfo->sourceIndex % 32))) != 0);
        }
        else
        {
            pend = ((sSrcMatchShortData.srcPendEn[aInfo->sourceIndex / 32] & (1 << (aInfo->sourceIndex % 32))) != 0);
        }
    }
    aFrame->mInfo.mRxInfo.mAckedWithFramePending = pend;

    if (otMacFrameIsVersion2015(aFrame))
    {
        aFrame->mInfo.mRxInfo.mAckedWithSecEnhAck = true;
        aFrame->mInfo.mRxInfo.mAckFrameCounter    = sAckFrameCounter;
        aFrame->mInfo.mRxInfo.mAckKeyId           = sAckKeyId;
    }

    if (crcCorr->status.bCrcErr)
    {
        memset(aFrame, 0U, sizeof(*aFrame));
        return OT_ERROR_FCS;
    }
    else
    {
        return OT_ERROR_NONE;
    }
}

/**
 * An RX queue entry is in the finished state, process it.
 */
static void handleRxDataFinish(otInstance *aInstance, unsigned int aEvents, rfc_dataEntryGeneral_t *curEntry)
{
    otError      error;
    otRadioFrame receiveFrame = {0};

    error = populateReceiveFrame(&receiveFrame, &(curEntry->data));
    if (OT_ERROR_NONE != error)
    {
        curEntry->status = DATA_ENTRY_PENDING;

        if (platformRadio_phyState_RxTxAck == sState)
        {
            rfCoreExecuteAbortCmd(sRfHandle, sTransmitAckCmdHandle);
            sState = platformRadio_phyState_Receive;
        }
        /* Indicate a receive error to the upper layers */
        platformRadioProcessReceiveDone(aInstance, &receiveFrame, error);
        return;
    }

    /* Is this an ACK frame? */
    if (otMacFrameIsAck(&receiveFrame))
    {
        if (platformRadio_phyState_Transmit == sState && otMacFrameIsAckRequested(&sTransmitFrame) &&
            otMacFrameGetSequence(&receiveFrame) == otMacFrameGetSequence(&sTransmitFrame))
        {
            sState = platformRadio_phyState_Receive;

            platformRadioProcessTransmitDone(aInstance, &sTransmitFrame, &receiveFrame, error);

#ifdef USE_DMM
            if (receiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING)
            {
                dmmSetThreadActivityRx(aInstance, true);
            }
#endif
        }

        curEntry->status = DATA_ENTRY_PENDING;
        return;
    }

    if (otMacFrameIsAckRequested(&receiveFrame))
    {
        // ack requested
        if (0 != (aEvents & RF_EVENT_RX_ACK_DONE))
        {
            platformRadioProcessReceiveDone(aInstance, &receiveFrame, OT_ERROR_NONE);

            curEntry->status = DATA_ENTRY_PENDING;
        }
        else
        {
            // ack has not been sent
            if (otMacFrameIsVersion2015(&receiveFrame))
            {
                if (sState == platformRadio_phyState_RxTxAck)
                {
                    // We have queued up an ack transmission, update the frame with LQI
                    rfCoreGenerateEnhAck(&receiveFrame, &sAckFrame);
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
                    // Update IE data in the 802.15.4 header with the newest CSL period / phase
                    if (sCslPeriod > 0)
                    {
                        otMacFrameSetCslIe(&sAckFrame, (uint16_t)sCslPeriod, getCslPhase(&sAckFrame));
                    }
#endif
                    rfCoreProcessTransmitSecurity(&sAckFrame);
                }
                else
                {
                    // something failed in the setup of the TX command, drop the frame
                    curEntry->status = DATA_ENTRY_PENDING;
                }
            }
            else
            {
#if PLAT_RADIO_SOFTWARE_IMM_ACK
                /* software is handling the imm-ack, update with the pend-bit
                 * set correctly. We use the RF Core's source match idx.
                 */
                otMacFrameGenerateImmAck(&receiveFrame, receiveFrame.mInfo.mRxInfo.mAckedWithFramePending, &sAckFrame);
#endif
            }
        }
    }
    else
    {
        platformRadioProcessReceiveDone(aInstance, &receiveFrame, OT_ERROR_NONE);

        curEntry->status = DATA_ENTRY_PENDING;
    }
}

/**
 * Empties the rx queue, regardless of the current state of the entries.
 */
static void clearRxQueue(void)
{
    rfc_dataEntryGeneral_t *curEntry   = (rfc_dataEntryGeneral_t *)sRxDataQueue.pCurrEntry;
    rfc_dataEntryGeneral_t *startEntry = curEntry;

    /* loop through receive queue */
    do
    {
        curEntry->status = DATA_ENTRY_PENDING;
        curEntry         = (rfc_dataEntryGeneral_t *)curEntry->pNextEntry;
    } while (curEntry != startEntry);
}

void processAckCreation(otInstance *aInstance)
{
    rfc_dataEntryGeneral_t *curEntry, *startEntry;
    startEntry = curEntry = (rfc_dataEntryGeneral_t *)sRxDataQueue.pCurrEntry;

    /* loop through receive queue */
    do
    {
        if (DATA_ENTRY_ACTIVE == curEntry->status || DATA_ENTRY_BUSY == curEntry->status)
        {
            // entry is being used by the CM0, this is the current frame
            otRadioFrame rxFrame = {0};
            uint8_t *    data    = &(curEntry->data);

            // data[0]          // rx element length
            rxFrame.mLength  = data[1] & 0x7F; // rx frame length (phy hdr)
            rxFrame.mPsdu    = &(data[2]);     // beginning of PSDU
            rxFrame.mChannel = sReceiveCmd.channel;

            if (!otMacFrameIsAckRequested(&rxFrame))
            {
                // we are not requesting an acknowledgment
                break;
            }

            if (otMacFrameIsVersion2015(&rxFrame))
            {
                /* Aux-Security header and LQI are not valid here, generate an
                 * empty frame to queue up the TX command. Contents will be
                 * filled in the RX_ENTRY_DONE handler.
                 */
                if (OT_ERROR_NONE != rfCoreGenerateEmptyEnhAck(&rxFrame, &sAckFrame))
                {
                    // fail handling the RX frame
                    break;
                }
            }
            else
            {
#if PLAT_RADIO_SOFTWARE_IMM_ACK
                sReceiveCmd.frameFiltOpt.autoAckEn = 0;
                // place empty imm-ack in TX command
                sAckFrame.mChannel = rxFrame.mChannel;
                sAckFrame.mLength = 5;
                memset(sAckFrame.mPsdu, 0U, 5);
#else
                // allow the CPE handle the ACK transmission
                sReceiveCmd.frameFiltOpt.autoAckEn = 1;
                break;
#endif
            }

            sAckFrame.mInfo.mTxInfo.mTxDelayBaseTime =
                RF_convertRatTicksToUs(RFCGetIeeeRxCaptureTime()); // timestamp of SHR
            sAckFrame.mInfo.mTxInfo.mTxDelay = ((4 + 1 + 1 + rxFrame.mLength) * 32U) + 192U;
            /* 4 octets of sync + 1 octet of SFD + 1 octet of PHR + Frame Length = length in octets
             * length in octets * 32uS per octet + 192uS of turnaround = tx delay
             */
            sTransmitAckCmdHandle = rfCoreSendTransmitCmd(aInstance, sRfHandle, &sAckFrame);
            if (sTransmitCmdHandle > 0)
            {
                sState = platformRadio_phyState_RxTxAck;
            }
            else
            {
                // main loop will see we are still in receive, error case
            }
            break;
        }
        curEntry = (rfc_dataEntryGeneral_t *)curEntry->pNextEntry;
    } while (curEntry != startEntry);
}

/**
 * Scan through the RX queue, looking for completed entries.
 */
static void processRxQueue(otInstance *aInstance, unsigned int aEvents)
{
    rfc_dataEntryGeneral_t *curEntry   = (rfc_dataEntryGeneral_t *)sRxDataQueue.pCurrEntry;
    rfc_dataEntryGeneral_t *startEntry = curEntry;

    /* loop through receive queue */
    do
    {
        switch (curEntry->status)
        {
        case DATA_ENTRY_UNFINISHED:
        {
            /* the command was aborted, cleanup the entry
             * Release this entry and move to the next entry
             */
            curEntry->status = DATA_ENTRY_PENDING;
            break;
        }

        case DATA_ENTRY_FINISHED:
        {
            /* Something is in this queue entry, process what we find */
            handleRxDataFinish(aInstance, aEvents, curEntry);
            break;
        }

        default:
        {
            /* Else - busy, or unused, just move to the next entry */
            break;
        }
        }
        curEntry = (rfc_dataEntryGeneral_t *)curEntry->pNextEntry;
    } while (curEntry != startEntry);
}

/**
 * Handle events in the TX state.
 */
static void handleTxState(otInstance *aInstance, unsigned int events)
{
    /* Save error on the stack and clear global variable */
    otError error = sTransmitError;

    if (OT_ERROR_NONE != error || 0U == (sTransmitCmd.pPayload[0] & IEEE802154_ACK_REQUEST))
    {
        sTransmitError = OT_ERROR_NONE;

        // error or we are not looking for an ACK
        sState = platformRadio_phyState_Receive;
        platformRadioProcessTransmitDone(aInstance, &sTransmitFrame, NULL, error);
    }
    else
    {
        // Acks are handled in the receive handler
    }
}

void platformRadioProcess(otInstance *aInstance, uintptr_t arg)
{
    /* Handle the events based on the radio state */
    switch (sState)
    {
    case platformRadio_phyState_Sleep:
    {
        if (arg & RF_EVENT_SLEEP_YIELD)
        {
            /* we have not been thrashed back into receive state, actually
             * release the RFC and clear the rx queue.
             */
            clearRxQueue();
            RF_yield(sRfHandle);
        }
        if (arg & (RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE))
        {
            /* Unfortunately, the frame must be discarded or we risk
             * asserting the MAC.
             */
            clearRxQueue();
        }
        break;
    }

    case platformRadio_phyState_Transmit:
    {
        if (arg & RF_EVENT_TX_CMD_PREEMPTED)
        {
            /* The RF driver has preempted the TX command string. This
             * is likely due to a temperature change preemption.
             * Notify of a TX failure and let retries restart the
             * command. It is very unlikely this will happen due to the
             * delay in scheduling of a command.
             */
            sTransmitError = OT_ERROR_ABORT;
            handleTxState(aInstance, RF_EVENT_TX_DONE);
        }
        else
        {
            /* The TX command string has finished */
            if (arg & RF_EVENT_TX_DONE)
            {
                handleTxState(aInstance, arg);
            }

            /* Handle new received frame */
            if (arg & (RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE))
            {
                processRxQueue(aInstance, arg);
            }

            /* Clear the receive buffer if the radio can't find space to put RX frames */
            if (arg & RF_EVENT_BUF_FULL)
            {
                clearRxQueue();
            }
        }
        break;
    }

    case platformRadio_phyState_RxTxAck:
    {
        /* The TX command string has finished */
        if (arg & RF_EVENT_TX_DONE)
        {
            if (OT_ERROR_NONE == sTransmitError)
            {
                arg |= RF_EVENT_RX_ACK_DONE;
            }
            else
            {
                // failed to transmit, let the RX process queue handle failure
            }
            sState = platformRadio_phyState_Receive;
            processRxQueue(aInstance, arg);
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

    case platformRadio_phyState_EdScan:
    {
        if (arg & RF_EVENT_ED_SCAN_DONE)
        {
            sState = platformRadio_phyState_Receive;

            /* restart receive command if necessary */
            rfCoreSendReceiveCmd(sRfHandle);

            if (sEdScanCmd.status == IEEE_DONE_OK)
            {
                otPlatRadioEnergyScanDone(aInstance, sEdScanCmd.maxRssi);
            }
            else
            {
                otPlatRadioEnergyScanDone(aInstance, PLATFORM_RADIO_INVALID_RSSI);
            }
        }
        /* fall through */
    }

    case platformRadio_phyState_Receive:
    {
        /* handle ack generation */
        if (arg & RF_EVENT_RX_FRM_FILT)
        {
            processAckCreation(aInstance);
        }

        /* Handle new received frame */
        if (arg & (RF_EVENT_RX_DONE | RF_EVENT_RX_ACK_DONE))
        {
            processRxQueue(aInstance, arg);
        }

        /* Clear the receive buffer if the radio can't find space to put RX frames */
        if (arg & RF_EVENT_BUF_FULL)
        {
            clearRxQueue();
        }

        /* Re-start the RX command since we are still in the state. */
        if (arg & RF_EVENT_RX_CMD_STOP)
        {
            rfCoreSendReceiveCmd(sRfHandle);
        }
        break;
    }

    case platformRadio_phyState_Disabled:
    default:
        break;
    }
}

