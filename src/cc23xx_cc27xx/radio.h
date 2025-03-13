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

// name collision with openthread/radio.h
#ifndef PLATFORM_RADIO_H_
#define PLATFORM_RADIO_H_

// clang-format off
#include <ti/devices/DeviceFamily.h>
// clang-format on

#include <openthread/instance.h>


/**
 * Event flags for the radio process function
 */
#define RF_EVENT_TX_DONE (1U << 0)
#define RF_EVENT_ED_SCAN_DONE (1U << 1)
#define RF_EVENT_RX_DONE (1U << 2)
#define RF_EVENT_RX_ACK_DONE (1U << 3)
#define RF_EVENT_SLEEP_YIELD (1U << 4)
#define RF_EVENT_BUF_FULL (1U << 5)
#define RF_EVENT_RX_CMD_STOP (1U << 6)
#define RF_EVENT_RX_CMD_ERROR (1U << 7)
#define RF_EVENT_TX_CMD_PREEMPTED (1U << 8)
#define RF_EVENT_RX_FRAME_FILT (1U << 9)
#define RF_EVENT_RX_FRM_FILT (1U << 10)

/**
 * (IEEE 802.15.4-2006) PSDU.FCF.framePending.
 */
#define IEEE802154_FRAME_PENDING_MASK (0x40)

/**
 * (IEEE 802.15.4-2006) PSDU.FCF.bFramePending.
 */
#define IEEE802154_FRAME_PENDING (1 << 4)

/**
 * (IEEE 802.15.4-2006) PSDU.FCF.bAR.
 */
#define IEEE802154_ACK_REQUEST (1 << 5)

/**
 * Return value used when searching the source match array.
 *
 * Returned if an address could not be found or if an empty element could not
 * be found.
 */
#define PLATFORM_RADIO_SRC_MATCH_NONE 0xFF

/**
 * Number of short addresses supported.
 */
#define PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM 20

#define PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM_WORDS \
(((PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM) + ((8 * sizeof(uint16_t)) - 1)) / (8 * sizeof(uint16_t)))

/**
 * Invalid RSSI value returned from an ED scan.
 */
#define PLATFORM_RADIO_INVALID_RSSI (127)

/**
 * This enum represents the state of a radio.
 *
 * Initially, a radio is in the Disabled state.
 *
 * The following are valid radio state transitions for the platform encompasing
 * an existing receive in the Transmit RxTxAck and EDScan states.
 *
 * ```
 *                                                 +---------+
 *                                                 | RxTxAck |
 *                                                 +---------+
 *                                                       ^ |
 *                                          FRM_FILT_INT | | TX_Done
 *                                                       | V
 *  +----------+  Enable()  +-------+  Receive()   +---------+   Transmit()  +----------+
 *  |          |----------->|       |------------->|         |-------------->|          |
 *  | Disabled |            | Sleep |              | Receive |               | Transmit |
 *  |          |<-----------|       |<-------------|         |<--------------|          |
 *  +----------+  Disable() |       |   Sleep()    +---------+ Receive() or  +----------+
 *    ^                     |       |                    | ^   Transmit complete
 *    |                     |       |       EnergyScan() | |
 *    |                     |       |                    V | Scan Complete
 *    |                     |       |               +--------+
 *  Init()                  |       |               | EdScan |
 *                          +-------+               +--------+
 * ```
 * he following are valid radio state transitions for the platform
 * in which the device can Transmit or perform an EDSCan directly from sleep.
 * ```
 *                               
 *  +----------+  Enable()  +-------+  Transmit()  +-------------------+  
 *  |          |----------->|       |------------->|                   |
 *  | Disabled |            | Sleep |              | TransmitStandlone |
 *  |          |<-----------|       |<-------------|                   |
 *  +----------+  Disable() |       |   Sleep()    +-------------------+ 
 *    ^                     |       | 
 *    |                     |       |   Scan Complete
 *    |                     |       |<-----------
 *    |                     |       | EnergyScan()  +------------------+
 *  Init()                  |       |-------------->| EdScanStandalone |
 *                          +-------+               +------------------+
 * ```
 *
 * These states slightly differ from the states in include/openthread/platform/radio.h
 * from OpenThread. The additional states the phy can be in are due to the asynchronous
 * nature of the radio core.
 *
 * | state              | description                                                                |
 * |------------------  |--------------------------------------------------------------------------- |
 * | Disabled           | The modem powerdomain is off                                               |
 * | Sleep              | The modem is powered and ready to run scheduled commands                   |
 * | Receive            | The modem is running a a continuous receive                                |
 * | RxTXAck            | A frame has been filtered and an Ack frame queued                          |
 * | EdScan             | The modem is running a energy detect operation and will return to receive  |
 * | EdScanStandalone   | The modem is running a energy detect operation and will return to sleep    |
 * | Transmit           | The modem is running a transmit command and will return to receive         |
 * | TransmitStandalone | The modem is running a transmit command and will return to sleep           |
 *
 */
typedef enum platformRadio_phyState
{
    platformRadio_phyState_Disabled = 0,
    platformRadio_phyState_Sleep,
    platformRadio_phyState_Receive,
    platformRadio_phyState_RxTxAck,
    platformRadio_phyState_EdScanStandalone,
    platformRadio_phyState_EdScan,
    platformRadio_phyState_TransmitStandalone,
    platformRadio_phyState_Transmit,
} platformRadio_phyState;

typedef enum platformRadio_txState
{
    /* Radio is not transmitting */
    platformRadio_txState_Inactive = 0, 
    /* Radio has issued a standalone Tx without CCA/RxAck capability */
    platformRadio_txState_StandaloneActive, 
    /* Radio has issued a combined Rx & Tx */
    platformRadio_txState_CombinedActive, 
} platformRadio_txState;
#endif /* PLATFORM_RADIO_H_ */

/**
 * The diagnostic module calls this function to begin transmitting a continuous tone. The tone will be transmitted on
 * the current receive channel.
 *
 * @param[in]  aInstance      The OpenThread instance structure.
 * @param[in]  aModulated     Indicates whether or not the tone was modulated or not.
 *
 * @retval OT_ERROR_NONE             Successfully started sending the RF tone.
 * @retval OT_ERROR_NOT_IMPLEMENTED  The radio doesn't support sending a test tone.
 */
otError otPlatDiagRadioToneStart(otInstance *aInstance, bool aModulated);

/**
 * The diagnostic module calls this to stop transmitting a continuous tone.
 *
 * @param[in]  aInstance      The OpenThread instance structure.
 *
 * @retval OT_ERROR_NONE             Successfully stopped sending the RF tone.
 * @retval OT_ERROR_NOT_IMPLEMENTED  The radio doesn't support sending a test tone.
 */
otError otPlatDiagRadioToneStop(otInstance *aInstance);

/**
 * The diagnostic module calls this to set the channel and disable channel switching.
 *
 * This function was added to support automated Thread Test Harness execution.
 * It is intended to simulate the device being placed within a shield box by
 * switching the radio to an unused channel and disallowing the stack switching
 * back to the set channel.
 *
 * @param[in]  aChannel The new (empty) channel.
 *
 */
void rfCoreDiagChannelDisable(uint8_t aChannel);

/**
 * The diagnostic module calls this to reset the channel and re-enable channel switching.
 *
 * @sa rfCoreDiagChannelDisable
 *
 * @param[in]  aChannel The original channel.
 *
 */
void rfCoreDiagChannelEnable(uint8_t aChannel);
