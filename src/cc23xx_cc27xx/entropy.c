/******************************************************************************

 @file entropy.c

 @brief Platform specific entropy functions for OpenThread

 Group: CMCU, LPC
 Target Device: cc13xx_cc26xx

 ******************************************************************************
 
 Copyright (c) 2017-2023, Texas Instruments Incorporated
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

/**
 * @file
 *   This file implements an entropy source based on TRNG.
 *
 */

#include <openthread/config.h>

#include <utils/code_utils.h>

#include <ti/devices/DeviceFamily.h>

#include <openthread/platform/entropy.h>
#include <ti/drivers/RNG.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
#if defined(DeviceFamily_CC23X0R5)
#include <ti/drivers/rng/RNGLPF3RF.h>
#endif
#include "ti_drivers_config.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h> 

RNG_Handle OT_RNG_handle;
#if defined(DeviceFamily_CC23X0R5)
extern const uint32_t RNGLPF3RF_noiseInputWordLen;
extern int_fast16_t RCL_AdcNoise_get_samples_blocking(uint32_t *buffer, uint32_t numWords);
#endif
/**
 * @internal
 * @brief Fill an arbitrary area with the random data.
 *
 * @param aOutput area to place the random data
 * @param aLen size if the area to place random data
 *
 *
 * @return returns 0 if no error occurred, -1 if error.
 */
    volatile int_fast16_t rtn;

static int getRandom(uint8_t *aOutput, size_t aLen)
{
    CryptoKey entropyKey;

    /*
     * prepare the data buffer
     */
    CryptoKeyPlaintext_initBlankKey(&entropyKey, aOutput, aLen);

    /* get entropy */
    rtn = RNG_generateKey(OT_RNG_handle, &entropyKey);
    if (rtn != RNG_STATUS_SUCCESS)
    {
        return OT_ERROR_FAILED;
    }

    return OT_ERROR_NONE;
}

#if defined(DeviceFamily_CC23X0R5)
/*******************************************************************************
 * This function is used by osal to initialize the RNG driver before the system boots
 */
static otError init_random_noise( void )
{
  uint16_t rclStatus = OT_ERROR_FAILED;
  uint16_t result;

  /* User's global array for noise input based on size provided in syscfg */
  uint32_t *localNoiseInput = malloc(RNGLPF3RF_noiseInputWordLen * sizeof(uint32_t));

  if (!localNoiseInput)
  {
    return ( OT_ERROR_FAILED );
  }

  /* Zeroize localNoiseInput */
  memset(localNoiseInput, 0, RNGLPF3RF_noiseInputWordLen * sizeof(uint32_t));

  while( rclStatus != OT_ERROR_NONE )
  {
    /* Fill noise input from RCL */
    rclStatus = RCL_AdcNoise_get_samples_blocking(localNoiseInput, RNGLPF3RF_noiseInputWordLen);
  }

  /* Initialize the RNG driver noise input pointer with global noise input array from user */
  result = RNGLPF3RF_conditionNoiseToGenerateSeed(localNoiseInput);

  /* Free before exiting */
  free(localNoiseInput);

  /* If we could not condition the RNG, fail */
  if ( result != RNG_STATUS_SUCCESS )
  {
    return ( OT_ERROR_FAILED );
  }

  return ( OT_ERROR_NONE );
}
#endif
/**
 * Function documented in system.h
 */
void platformRandomInit(void)
{
    RNG_Params RNGParams;

#if defined(DeviceFamily_CC23X0R5)
    /* Seed initialization */
    init_random_noise();
#endif
    /* Init the RNG HW */
    RNG_init();

    RNG_Params_init(&RNGParams);
    /* use the polling mode */
    RNGParams.returnBehavior = RNG_RETURN_BEHAVIOR_POLLING;

    OT_RNG_handle= RNG_open(CONFIG_RNG_THREAD, &RNGParams);
    if (OT_RNG_handle == NULL)
    {
        while (true)
            ;
    }

    return;
}
/**
 * Function documented in system.h
 */
void platformRandomProcess(void)
{
    /* place holder */

}

/**
 * Function documented in platform/entropy.h
 */
otError otPlatEntropyGet(uint8_t *aOutput, uint16_t aOutputLength)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(NULL != aOutput, error = OT_ERROR_INVALID_ARGS);

    otEXPECT_ACTION(getRandom(aOutput, aOutputLength) == 0, error = OT_ERROR_FAILED);
exit:
    return error;
}

