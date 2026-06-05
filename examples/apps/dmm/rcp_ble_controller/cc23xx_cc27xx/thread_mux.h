/*
 *  Copyright (c) 2025, Texas Instruments Incorporated
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

/*!
 * @file  thread_mux.h
 * @brief Thread (OpenThread) side of the Combined Serial MUX integration.
 *
 * Registers the OpenThread NLI (MUX_NLI_OT = 0) receive callback with the
 * MUX task so that decoded spinel frames arriving from the host are delivered
 * to the OpenThread processing loop via platformUartMuxDeliver().
 *
 * Call ThreadMux_init() once, before MuxTask_create(), in the application
 * main() initialisation sequence.
 */

#ifndef THREAD_MUX_H
#define THREAD_MUX_H

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Register the OpenThread NLI receive callback with the MUX task.
 *
 * Must be called before MuxTask_create() so the callback is in place before
 * the MUX task can dispatch inbound NLI_OT frames.
 */
void ThreadMux_init(void);

#ifdef __cplusplus
}
#endif

#endif /* THREAD_MUX_H */
