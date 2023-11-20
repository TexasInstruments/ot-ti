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

/* Standard Library Header files */
#include <stdbool.h>
#include <stdint.h>

/* ClockP Header files */
#include <ti/drivers/dpl/ClockP.h>

/* OpenThread public API Header files */
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>

#include "system.h"

#ifdef OT_TI_KERNEL_freertos
#include <FreeRTOS.h>
#endif

static uint32_t         Alarm_time0   = 0;
static uint32_t         Alarm_time    = 0;
static ClockP_Handle    Alarm_handle  = 0;
static ClockP_Struct    Alarm_Struct;
static ClockP_Params    Alarm_Params;
static bool             Alarm_running = false;

/**
 * Milliseconds converted to system clock ticks. Minimum value of 1.
 */
uint32_t milliToTicks(uint32_t milli)
{
    uint32_t ticks = milli;
    uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();

    if(milli != 0U)
    {
        ticks = (milli * 1000)/tickPeriodUs;
        if(ticks == 0U)
        {
            ticks = 1U;
        }
    }

    return(ticks);
}

/**
 * System clock ticks converted to milliseconds. Minumum value of 1.
 */
uint32_t ticksToMilli(uint32_t ticks)
{
    uint32_t milli = ticks;
    uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();

    if(ticks != 0U)
    {
        milli = (ticks * tickPeriodUs) / 1000;
        if(milli == 0U)
        {
            milli = 1U;
        }
    }

    return(milli);
}

/**
 * Handler for the ClockP clock callback.
 */
void Alarm_handler(uintptr_t val)
{
    (void)val;
    platformAlarmSignal();
}

/**
 * Function documented in system.h
 */
void platformAlarmInit(void)
{
    ClockP_Params_init(&Alarm_Params);
    Alarm_Params.startFlag = 0;

    Alarm_handle = ClockP_construct(&Alarm_Struct,
                                    Alarm_handler,
                                    0,
                                    &Alarm_Params);

    Alarm_running = false;
}

/**
 * Function documented in platform/alarm-milli.h
 */
uint32_t otPlatAlarmMilliGetNow(void)
{
    uint32_t ticks = ClockP_getSystemTicks();

    return (ticksToMilli(ticks));
}

/**
 * Function documented in platform/alarm-milli.h
 */
void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;
    uint32_t delta = (otPlatAlarmMilliGetNow() - aT0);

    Alarm_time0   = aT0;
    Alarm_time    = aDt;
    Alarm_running = true;

    if ((delta) >= aDt)
    {
        // alarm is in the past
        platformAlarmSignal();
    }
    else
    {
        uint64_t timeout = aDt - delta;
#ifdef OT_TI_KERNEL_freertos
        if (configTICK_RATE_HZ > 1000U)
        {
            if (timeout > UINT32_MAX / (configTICK_RATE_HZ / 1000U))
            {
                timeout = UINT32_MAX / (configTICK_RATE_HZ / 1000U);
            }
        }
#endif
        //use ClockP_setTimeout to set timeout later
        uint32_t ticksTimeout = milliToTicks(timeout);

        ClockP_setTimeout(Alarm_handle, ticksTimeout);
        ClockP_start(Alarm_handle);
    }
}

/**
 * Function documented in platform/alarm-milli.h
 */
void otPlatAlarmMilliStop(otInstance *aInstance)
{
    (void)aInstance;

    ClockP_stop(Alarm_handle);
    Alarm_running = false;
}

/**
 * Function documented in system.h
 */
void platformAlarmProcess(otInstance *aInstance)
{
    if (Alarm_running)
    {
        uint32_t nowTime    = otPlatAlarmMilliGetNow();
        uint32_t offsetTime = nowTime - Alarm_time0;

        if (Alarm_time <= offsetTime)
        {
            Alarm_running = false;
#if OPENTHREAD_CONFIG_DIAG_ENABLE

            if (otPlatDiagModeGet())
            {
                otPlatDiagAlarmFired(aInstance);
            }
            else
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */
            {
                otPlatAlarmMilliFired(aInstance);
            }
        }
        else
        {

            // restart the timer, might have overflowed or fired early for some reason
            otPlatAlarmMilliStartAt(aInstance, nowTime, Alarm_time - offsetTime);
        }
    }
}

