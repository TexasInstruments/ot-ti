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

/******************************************************************************
 Includes
 *****************************************************************************/
#include <openthread/config.h>

/* Standard Library Header files */
#include <stdbool.h>
#include <stdint.h>

/* ClockP Header files */
#include <ti/drivers/dpl/ClockP.h>

/* OpenThread public API Header files */
#include <openthread/platform/alarm-micro.h>

#include "system.h"

/******************************************************************************
 Local variables
 *****************************************************************************/

static uint32_t         AlarmMicro_time0   = 0;
static uint32_t         AlarmMicro_time    = 0;
static ClockP_Handle    AlarmMicro_handle  = 0;
static ClockP_Struct    AlarmMicro_Struct;
static ClockP_Params    AlarmMicro_Params;
static bool             AlarmMicro_running = false;

/**
 * Microoseconds converted to system clock ticks. Minumum value of 1.
 */
uint32_t microToTicks(uint32_t micro)
{
    uint32_t ticks = micro;
    uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();

    if(micro != 0U)
    {
        ticks = micro/tickPeriodUs;
        if(ticks == 0U)
        {
            ticks = 1U;
        }
    }

    return(ticks);
}

/**
 * System clock ticks converted to microseconds. Minumum value of 1.
 */
uint32_t ticksToMicro(uint32_t ticks)
{
    uint32_t micro = ticks;
    uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();

    if(ticks != 0U)
    {
        micro = (ticks * tickPeriodUs);
        if(micro == 0U)
        {
            micro = 1U;
        }
    }

    return(micro);
}

/**
 * Handler for the ClockP clock callback.
 */
void AlarmMicro_handler(uintptr_t val)
{
    (void)val;
    platformAlarmMicroSignal();
}

/**
 * Function documented in system.h
 */
void platformAlarmMicroInit(void)
{
    ClockP_Params_init(&AlarmMicro_Params);
    AlarmMicro_Params.startFlag = 0;

    AlarmMicro_handle = ClockP_construct(&AlarmMicro_Struct,
                                        AlarmMicro_handler,
                                        0,
                                        &AlarmMicro_Params);   

    AlarmMicro_running = false;
}

uint64_t otPlatTimeGet(void)
{
    uint32_t ticks = ClockP_getSystemTicks();

    return (ticksToMicro(ticks));
}

/**
 * Function documented in platform/alarm-micro.h
 */
uint32_t otPlatAlarmMicroGetNow(void)
{
    uint32_t ticks = ClockP_getSystemTicks();

    return (ticksToMicro(ticks));
}

/**
 * Function documented in platform/alarm-micro.h
 */
void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;
    uint32_t delta = (otPlatAlarmMicroGetNow() - aT0);

    AlarmMicro_time0   = aT0;
    AlarmMicro_time    = aDt;
    AlarmMicro_running = true;

    if ((delta) >= aDt)
    {
        // alarm is in the past
        platformAlarmMicroSignal();
    }
    else
    {
        uint64_t timeout = aDt - delta;

        //use ClockP_setTimeout to set timeout later
        uint32_t ticksTimeout = microToTicks(timeout);

        ClockP_setTimeout(AlarmMicro_handle, ticksTimeout);
        ClockP_start(AlarmMicro_handle);
    }
}

/**
 * Function documented in platform/alarm-micro.h
 */
void otPlatAlarmMicroStop(otInstance *aInstance)
{
    (void)aInstance;

    ClockP_stop(AlarmMicro_handle);
    AlarmMicro_running = false;
}

/**
 * Function documented in system.h
 */
void platformAlarmMicroProcess(otInstance *aInstance)
{
#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
    if (AlarmMicro_running)
    {
        uint32_t nowTime    = otPlatAlarmMicroGetNow();
        uint32_t offsetTime = nowTime - AlarmMicro_time0;

        if (AlarmMicro_time <= offsetTime)
        {
            AlarmMicro_running = false;

            otPlatAlarmMicroFired(aInstance);
        }
        else
        {
            // restart the timer, might have overflowed or fired early for some reason
            otPlatAlarmMicroStartAt(aInstance, nowTime, AlarmMicro_time - offsetTime);
        }
    }
#endif /* OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE */
}

