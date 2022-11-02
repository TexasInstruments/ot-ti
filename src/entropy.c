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

#include <utils/code_utils.h>

#include <openthread/platform/entropy.h>

#include "ti_drivers_config.h"
#include <ti/drivers/TRNG.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>

#include <assert.h>

TRNG_Handle TRNG_handle;

static int getRandom(uint8_t *aOutput, size_t aLen)
{
    CryptoKey entropyKey;

    CryptoKeyPlaintext_initBlankKey(&entropyKey, aOutput, aLen);

    if (TRNG_STATUS_SUCCESS == TRNG_generateEntropy(TRNG_handle, &entropyKey))
    {
        return OT_ERROR_NONE;
    }
    else
    {
        return OT_ERROR_FAILED;
    }
}

void platformRandomInit(void)
{
    otError     error = OT_ERROR_NONE;
    TRNG_Params TRNGParams;

    TRNG_init();

    TRNG_Params_init(&TRNGParams);
    TRNGParams.returnBehavior = TRNG_RETURN_BEHAVIOR_POLLING;

    TRNG_handle = TRNG_open(CONFIG_TRNG_THREAD, &TRNGParams);
    otEXPECT_ACTION(NULL != TRNG_handle, error = OT_ERROR_FAILED);

exit:

    assert(error == OT_ERROR_NONE);

    /* suppress the compiling warning */
    (void)error;

    return;
}

otError otPlatEntropyGet(uint8_t *aOutput, uint16_t aOutputLength)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(NULL != aOutput, error = OT_ERROR_INVALID_ARGS);

    error = getRandom(aOutput, aOutputLength);

exit:
    return error;
}
