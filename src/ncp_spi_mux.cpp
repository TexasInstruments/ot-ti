/*
 *    Copyright (c) 2022, Texas Instruments Incorporated
 *    Copyright (c) 2016, The OpenThread Authors.
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements a SPI interface to the OpenThread stack.
 */

#include "ncp_spi_mux.hpp"

#include <openthread/ncp.h>
#include <openthread/platform/misc.h>
#include <openthread/platform/spi-slave.h>
#include <openthread/platform/toolchain.h>

#include "openthread-core-config.h"
#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/instance.hpp"
#include "common/new.hpp"
#include "net/ip6.hpp"

#include <string.h>

#if OPENTHREAD_CONFIG_DIAG_ENABLE
static_assert(OPENTHREAD_CONFIG_DIAG_OUTPUT_BUFFER_SIZE <=
                  OPENTHREAD_CONFIG_NCP_SPI_BUFFER_SIZE - ot::Ncp::NcpBase::kSpinelCmdHeaderSize -
                      ot::Ncp::NcpBase::kSpinelPropIdSize - ot::Ncp::SpiFrame::kHeaderSize,
              "diag output should be smaller than NCP SPI tx buffer");
static_assert(OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE <= OPENTHREAD_CONFIG_NCP_SPI_BUFFER_SIZE,
              "diag command line should be smaller than NCP SPI rx buffer");
#endif

namespace ot {
namespace Ncp {

static OT_DEFINE_ALIGNED_VAR(sNcpRaw, sizeof(NcpSpiMux), uint64_t);

extern "C" void otAppNcpInit(otInstance *aInstance)
{
    NcpSpiMux * ncpSpiMux = nullptr;
    Instance *instance   = static_cast<Instance *>(aInstance);

    ncpSpiMux = new (&sNcpRaw) NcpSpiMux(instance);

    if (ncpSpiMux == nullptr || ncpSpiMux != NcpBase::GetNcpInstance())
    {
        OT_ASSERT(false);
    }
}

NcpSpiMux::NcpSpiMux(Instance *aInstance)
    : NcpBase(aInstance)
    , mTxState(kTxStateIdle)
    , mHandlingRxFrame(false)
    , mResetFlag(true)
    , mPrepareTxFrameTask(*aInstance, NcpSpiMux::PrepareTxFrame)
    , mSendFrameLength(0)
{
    SpiFrame sendFrame(mSendFrame);
    SpiFrame emptyFullAccept(mEmptySendFrameFullAccept);
    SpiFrame emptyZeroAccept(mEmptySendFrameZeroAccept);

    sendFrame.SetHeaderFlagByte(/* aResetFlag */ true);
    sendFrame.SetHeaderAcceptLen(0);
    sendFrame.SetHeaderDataLen(0);

    emptyFullAccept.SetHeaderFlagByte(/* aResetFlag */ true);
    emptyFullAccept.SetHeaderAcceptLen(kSpiBufferSize - kSpiHeaderSize);
    emptyFullAccept.SetHeaderDataLen(0);

    emptyZeroAccept.SetHeaderFlagByte(/* aResetFlag */ true);
    emptyZeroAccept.SetHeaderAcceptLen(0);
    emptyZeroAccept.SetHeaderDataLen(0);

    mNli1RecvCallback = nullptr;
    mNli1SentCallback = nullptr;
    mNli1Context      = nullptr;
    mNli1IsSpinel     = false;
    mNli1TxBuffer     = nullptr;
    mNli1TxLen        = 0;

    mNli2RecvCallback = nullptr;
    mNli2SentCallback = nullptr;
    mNli2Context      = nullptr;
    mNli2IsSpinel     = false;
    mNli2TxBuffer     = nullptr;
    mNli2TxLen        = 0;

    mNli3RecvCallback = nullptr;
    mNli3SentCallback = nullptr;
    mNli3Context      = nullptr;
    mNli3IsSpinel     = false;
    mNli3TxBuffer     = nullptr;
    mNli3TxLen        = 0;

    mTxFrameBuffer.SetFrameAddedCallback(HandleFrameAddedToTxBuffer, this);

    IgnoreError(otPlatSpiSlaveEnable(&NcpSpiMux::SpiTransactionComplete, &NcpSpiMux::SpiTransactionProcess, this));

    // We signal an interrupt on this first transaction to
    // make sure that the host processor knows that our
    // reset flag was set.

    IgnoreError(otPlatSpiSlavePrepareTransaction(mEmptySendFrameZeroAccept, kSpiHeaderSize, mEmptyReceiveFrame,
                                                 kSpiHeaderSize,
                                                 /* aRequestTransactionFlag */ true));
}

extern "C" void SetNliCallback(uint8_t aNli, NcpSpiMuxRecvCallback aRecvCallback, NcpSpiMuxSentCallback aSentCallback, void *aContext, bool aIsSpinel)
{
    static_cast<NcpSpiMux *>(NcpBase::GetNcpInstance())->SetNliCallback(aNli, aRecvCallback, aSentCallback, aContext, aIsSpinel);
}

void NcpSpiMux::SetNliCallback(uint8_t aNli, NcpSpiMuxRecvCallback aRecvCallback, NcpSpiMuxSentCallback aSentCallback, void *aContext, bool aIsSpinel)
{
    if (0 == aNli)
    {
        // cannot set the OpenThread nli
    }
    else if (1 == aNli)
    {
        mNli1RecvCallback = aRecvCallback;
        mNli1SentCallback = aSentCallback;
        mNli1Context      = aContext;
        mNli1IsSpinel     = aIsSpinel;
    }
    else if (2 == aNli)
    {
        mNli2RecvCallback = aRecvCallback;
        mNli2SentCallback = aSentCallback;
        mNli2Context      = aContext;
        mNli2IsSpinel     = aIsSpinel;
    }
    else if (3 == aNli)
    {
        mNli3RecvCallback = aRecvCallback;
        mNli3SentCallback = aSentCallback;
        mNli3Context      = aContext;
        mNli3IsSpinel     = aIsSpinel;
    }
}

extern "C" void SendNli(uint8_t aNli, const uint8_t *aBuf, uint16_t aBufLength)
{
    static_cast<NcpSpiMux *>(NcpBase::GetNcpInstance())->SendNli(aNli, aBuf, aBufLength);
}

void NcpSpiMux::SendNli(uint8_t aNli, const uint8_t *aBuf, uint16_t aBufLength)
{
    if (0 == aNli)
    {
        // cannot set the OpenThread nli
    }
    else if (1 == aNli)
    {
        mNli1TxBuffer = aBuf;
        mNli1TxLen    = aBufLength;
        mPrepareTxFrameTask.Post();
    }
    else if (2 == aNli)
    {
        mNli2TxBuffer = aBuf;
        mNli2TxLen    = aBufLength;
        mPrepareTxFrameTask.Post();
    }
    else if (3 == aNli)
    {
        mNli3TxBuffer = aBuf;
        mNli3TxLen    = aBufLength;
        mPrepareTxFrameTask.Post();
    }
    else
    {
        // invalid NLI
    }
}

bool NcpSpiMux::SpiTransactionComplete(void *   aContext,
                                    uint8_t *aOutputBuf,
                                    uint16_t aOutputLen,
                                    uint8_t *aInputBuf,
                                    uint16_t aInputLen,
                                    uint16_t aTransLen)
{
    NcpSpiMux *ncp = reinterpret_cast<NcpSpiMux *>(aContext);

    return ncp->SpiTransactionComplete(aOutputBuf, aOutputLen, aInputBuf, aInputLen, aTransLen);
}

bool NcpSpiMux::SpiTransactionComplete(uint8_t *aOutputBuf,
                                    uint16_t aOutputLen,
                                    uint8_t *aInputBuf,
                                    uint16_t aInputLen,
                                    uint16_t aTransLen)
{
    // This can be executed from an interrupt context, therefore we cannot
    // use any of OpenThread APIs here. If further processing is needed,
    // returned value `shouldProcess` is set to `true` to indicate to
    // platform SPI slave driver to invoke `SpiTransactionProcess()` callback
    // which unlike this callback must be called from the same OS context
    // that OpenThread APIs/callbacks are executed.

    uint16_t transDataLen;
    bool     shouldProcess = false;
    SpiFrame outputFrame(aOutputBuf);
    SpiFrame inputFrame(aInputBuf);
    SpiFrame sendFrame(mSendFrame);

    VerifyOrExit((aTransLen >= kSpiHeaderSize) && (aInputLen >= kSpiHeaderSize) && (aOutputLen >= kSpiHeaderSize));
    VerifyOrExit(inputFrame.IsValid() && outputFrame.IsValid());

    transDataLen = aTransLen - kSpiHeaderSize;

    if (!mHandlingRxFrame)
    {
        uint16_t rxDataLen = inputFrame.GetHeaderDataLen();

        // A new frame is successfully received if input frame
        // indicates that there is data and the "data len" is not
        // larger than than the "accept len" we provided in the
        // exchanged output frame.

        if ((rxDataLen > 0) && (rxDataLen <= transDataLen) && (rxDataLen <= outputFrame.GetHeaderAcceptLen()))
        {
            mHandlingRxFrame = true;
            shouldProcess    = true;
        }
    }

    if (mTxState == kTxStateSending)
    {
        uint16_t txDataLen = outputFrame.GetHeaderDataLen();

        // Frame transmission is successful if master indicates
        // in the input frame that it could accept the frame
        // length that was exchanged, i.e., the "data len" in
        // the output frame is smaller than or equal to "accept
        // len" in the received input frame from master.

#ifdef SPI_NO_RETRY_TX_ON_RX_ERR
        if ((txDataLen > 0) && (txDataLen <= inputFrame.GetHeaderAcceptLen()))
#else
        if ((txDataLen > 0) && (txDataLen <= transDataLen) && (txDataLen <= inputFrame.GetHeaderAcceptLen()))
#endif
        {
            mTxState      = kTxStateHandlingSendDone;
            shouldProcess = true;
        }
    }

exit:
    // Determine the input and output frames to prepare a new transaction.

    if (mResetFlag && (aTransLen > 0) && (aOutputLen > 0))
    {
        mResetFlag = false;
        sendFrame.SetHeaderFlagByte(/*aResetFlag */ false);
        SpiFrame(mEmptySendFrameFullAccept).SetHeaderFlagByte(/*aResetFlag */ false);
        SpiFrame(mEmptySendFrameZeroAccept).SetHeaderFlagByte(/*aResetFlag */ false);
    }

    if (mTxState == kTxStateSending)
    {
        aOutputBuf = mSendFrame;
        aOutputLen = mSendFrameLength;
    }
    else
    {
        aOutputBuf = mHandlingRxFrame ? mEmptySendFrameZeroAccept : mEmptySendFrameFullAccept;
        aOutputLen = kSpiHeaderSize;
    }

    if (mHandlingRxFrame)
    {
        aInputBuf = mEmptyReceiveFrame;
        aInputLen = kSpiHeaderSize;
    }
    else
    {
        aInputBuf = mReceiveFrame;
        aInputLen = kSpiBufferSize;
    }

    sendFrame.SetHeaderAcceptLen(aInputLen - kSpiHeaderSize);

    IgnoreError(
        otPlatSpiSlavePrepareTransaction(aOutputBuf, aOutputLen, aInputBuf, aInputLen, (mTxState == kTxStateSending)));

    return shouldProcess;
}

void NcpSpiMux::SpiTransactionProcess(void *aContext)
{
    reinterpret_cast<NcpSpiMux *>(aContext)->SpiTransactionProcess();
}

void NcpSpiMux::SpiTransactionProcess(void)
{
    if (mTxState == kTxStateHandlingSendDone)
    {
        mPrepareTxFrameTask.Post();
    }

    if (mHandlingRxFrame)
    {
        HandleRxFrame();
    }
}

void NcpSpiMux::HandleFrameAddedToTxBuffer(void *                   aContext,
                                        Spinel::Buffer::FrameTag aTag,
                                        Spinel::Buffer::Priority aPriority,
                                        Spinel::Buffer *         aBuffer)
{
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aTag);
    OT_UNUSED_VARIABLE(aPriority);

    static_cast<NcpSpiMux *>(aContext)->mPrepareTxFrameTask.Post();
}


void NcpSpiMux::PrepareNextSpiSendFrameOpenThread(void)
{
    otError  error = OT_ERROR_NONE;
    uint16_t frameLength;
    uint16_t readLength;
    SpiFrame sendFrame(mSendFrame);

    if (ShouldWakeHost())
    {
        otPlatWakeHost();
    }

    SuccessOrExit(error = mTxFrameBuffer.OutFrameBegin());

    frameLength = mTxFrameBuffer.OutFrameGetLength();
    OT_ASSERT(frameLength <= kSpiBufferSize - kSpiHeaderSize);

    // The "accept length" in `mSendFrame` is already updated based
    // on current state of receive. It is changed either from the
    // `SpiTransactionComplete()` callback or from `HandleRxFrame()`.

    readLength = mTxFrameBuffer.OutFrameRead(frameLength, sendFrame.GetData());
    OT_ASSERT(readLength == frameLength);

    // Suppress the warning when assertions are disabled
    OT_UNUSED_VARIABLE(readLength);

    sendFrame.SetHeaderDataLen(frameLength);
    mSendFrameLength = frameLength + kSpiHeaderSize;

    mTxState = kTxStateSending;

    // Prepare new transaction by using `mSendFrame` as the output
    // frame while keeping the input frame unchanged.

    error = otPlatSpiSlavePrepareTransaction(mSendFrame, mSendFrameLength, nullptr, 0, /* aRequestTrans */ true);

    if (error == OT_ERROR_BUSY)
    {
        // Being busy is OK. We will get the transaction set up
        // properly when the current transaction is completed.
        error = OT_ERROR_NONE;
    }

    if (error != OT_ERROR_NONE)
    {
        mTxState = kTxStateIdle;
        mPrepareTxFrameTask.Post();
        ExitNow();
    }

    IgnoreError(mTxFrameBuffer.OutFrameRemove());

exit:
    return;
}

void NcpSpiMux::PrepareNextSpiSendFrameNli(uint8_t aNli, bool aIsSpinel, const uint8_t *aBuf, uint16_t aBufLength)
{
    otError  error = OT_ERROR_NONE;
    uint16_t frameLength;
    SpiFrame sendFrame(mSendFrame);

    if (ShouldWakeHost())
    {
        otPlatWakeHost();
    }

    if (aIsSpinel)
    {
        frameLength = aBufLength;
        memcpy(sendFrame.GetData(), aBuf, aBufLength);
    }
    else
    {
        uint8_t* data = sendFrame.GetData();
        frameLength = aBufLength + 1;
        data[0] = (aNli << 4); // set nli
        memcpy(&(data[1]), aBuf, aBufLength);
    }

    sendFrame.SetHeaderDataLen(frameLength);
    mSendFrameLength = frameLength + kSpiHeaderSize;

    mTxState = kTxStateSending;

    // Prepare new transaction by using `mSendFrame` as the output
    // frame while keeping the input frame unchanged.

    error = otPlatSpiSlavePrepareTransaction(mSendFrame, mSendFrameLength, nullptr, 0, /* aRequestTrans */ true);

    if (error == OT_ERROR_BUSY)
    {
        // Being busy is OK. We will get the transaction set up
        // properly when the current transaction is completed.
        error = OT_ERROR_NONE;
    }

    if (error != OT_ERROR_NONE)
    {
        mTxState = kTxStateIdle;
        mPrepareTxFrameTask.Post();
        ExitNow();
    }

exit:
    return;
}

void NcpSpiMux::PrepareNextSpiSendFrame(void)
{
    if (mTxState == kTxStateIdle)
    {
        if (!mTxFrameBuffer.IsEmpty())
        {
            PrepareNextSpiSendFrameOpenThread();
        }
        else if (nullptr != mNli1TxBuffer)
        {
            const uint8_t* buffer = mNli1TxBuffer;
            uint16_t       length = mNli1TxLen;

            PrepareNextSpiSendFrameNli(1, mNli1IsSpinel, mNli1TxBuffer, mNli1TxLen);

            mNli1TxBuffer = nullptr;
            mNli1TxLen    = 0;

            if (mNli1SentCallback)
            {
                mNli1SentCallback(buffer, length, mNli1Context);
            }
        }
        else if (nullptr != mNli2TxBuffer)
        {
            const uint8_t* buffer = mNli2TxBuffer;
            uint16_t       length = mNli2TxLen;

            PrepareNextSpiSendFrameNli(2, mNli2IsSpinel, mNli2TxBuffer, mNli2TxLen);

            mNli2TxBuffer = nullptr;
            mNli2TxLen    = 0;

            if (mNli2SentCallback)
            {
                mNli2SentCallback(buffer, length, mNli2Context);
            }
        }
        else if (nullptr != mNli2TxBuffer)
        {
            const uint8_t* buffer = mNli3TxBuffer;
            uint16_t       length = mNli3TxLen;

            PrepareNextSpiSendFrameNli(3, mNli3IsSpinel, mNli3TxBuffer, mNli3TxLen);

            mNli3TxBuffer = nullptr;
            mNli3TxLen    = 0;

            if (mNli3SentCallback)
            {
                mNli3SentCallback(buffer, length, mNli3Context);
            }
        }
    }
}

void NcpSpiMux::PrepareTxFrame(Tasklet &aTasklet)
{
    OT_UNUSED_VARIABLE(aTasklet);
    static_cast<NcpSpiMux *>(GetNcpInstance())->PrepareTxFrame();
}

void NcpSpiMux::PrepareTxFrame(void)
{
    switch (mTxState)
    {
    case kTxStateHandlingSendDone:
        mTxState = kTxStateIdle;

        OT_FALL_THROUGH;
        // to next case to prepare the next frame (if any).

    case kTxStateIdle:
        PrepareNextSpiSendFrame();
        break;

    case kTxStateSending:
        // The next frame in queue (if any) will be prepared when the
        // current frame is successfully sent and this task is posted
        // again from the `SpiTransactionComplete()` callback.
        break;
    }
}

void NcpSpiMux::HandleRxFrame(void)
{
    SpiFrame recvFrame(mReceiveFrame);
    SpiFrame sendFrame(mSendFrame);

    uint8_t *data = recvFrame.GetData();
    uint16_t len = recvFrame.GetHeaderDataLen();

    uint8_t frame_nli = (((data[0]) >> 4) & 0x03);

    if (0 == frame_nli)
    {
        // OpenThread
        // Pass the received frame to base class to process.
        HandleReceive(data, len);
    }
    else if (1 == frame_nli && nullptr != mNli1RecvCallback)
    {
        if (mNli1IsSpinel)
        {
            mNli1RecvCallback(data, len, mNli1Context);
        }
        else
        {
            mNli1RecvCallback(&(data[1]), len - 1, mNli1Context);
        }
    }
    else if (2 == frame_nli && nullptr != mNli2RecvCallback)
    {
        if (mNli2IsSpinel)
        {
            mNli2RecvCallback(data, len, mNli2Context);
        }
        else
        {
            mNli2RecvCallback(&(data[1]), len - 1, mNli2Context);
        }
    }
    else if (3 == frame_nli && nullptr != mNli3RecvCallback)
    {
        if (mNli3IsSpinel)
        {
            mNli3RecvCallback(data, len, mNli3Context);
        }
        else
        {
            mNli3RecvCallback(&(data[1]), len - 1, mNli3Context);
        }
    }

    // The order of operations below is important. We should clear
    // the `mHandlingRxFrame` before checking `mTxState` and possibly
    // preparing the next transaction. Note that the callback
    // `SpiTransactionComplete()` can be invoked from ISR at any point.
    //
    // If we switch the order, we have the following race situation:
    // We check `mTxState` and it is in `kTxStateSending`, so we skip
    // preparing the transaction here. But before we set the
    // `mHandlingRxFrame` to `false`, the `SpiTransactionComplete()`
    // happens and prepares the next transaction and sets the accept
    // length to zero on `mSendFrame` (since it assumes we are still
    // handling the previous received frame).

    mHandlingRxFrame = false;

    // If tx state is in `kTxStateSending`, we wait for the callback
    // `SpiTransactionComplete()`  which will then set up everything
    // and prepare the next transaction.

    if (mTxState != kTxStateSending)
    {
        sendFrame.SetHeaderAcceptLen(kSpiBufferSize - kSpiHeaderSize);

        IgnoreError(otPlatSpiSlavePrepareTransaction(mEmptySendFrameFullAccept, kSpiHeaderSize, mReceiveFrame,
                                                     kSpiBufferSize,
                                                     /* aRequestTrans */ false));

        // No need to check the error status. Getting `OT_ERROR_BUSY`
        // is OK as everything will be set up properly from callback when
        // the current transaction is completed.
    }
}

} // namespace Ncp
} // namespace ot
