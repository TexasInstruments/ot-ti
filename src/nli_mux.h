/******************************************************************************

 @file nli_mux.h

 @brief C bindings for the NLI extensions to the mux adapter.

 Group: CMCU, LPC
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2022-2023, Texas Instruments Incorporated
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

#ifndef NLI_MUX_H_
#define NLI_MUX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback for indicating a new frame has been received. The buffer is not
 * stored in static memory.
 */
typedef void (*NcpSpiMuxRecvCallback)(const uint8_t *aBuf, uint16_t aBufLength, void *aContext);

/**
 * Callback for indicating a frame has been sent over the transport. This can
 * be used to re-claim resources used by the buffer.
 */
typedef void (*NcpSpiMuxSentCallback)(const uint8_t *aBuf, uint16_t aBufLength, void *aContext);

/**
 * Sets a callback for the given network link identifier (NLI).
 */
void SetNliCallback(uint8_t aNli, NcpSpiMuxRecvCallback aRecvCallback, NcpSpiMuxSentCallback aSentCallback, void *aContext, bool aIsSpinel);

/**
 * Sets a buffer to be sent when available over the transport. Overwrites
 * values stored by previous calls to this function.
 */
void SendNli(uint8_t aNli, const uint8_t *aBuf, uint16_t aBufLength);

#ifdef __cplusplus
}
#endif

#endif /* NLI_MUX_H_ */
