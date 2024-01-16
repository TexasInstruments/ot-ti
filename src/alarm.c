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
#include <ti/drivers/dpl/HwiP.h>
/* OpenThread public API Header files */
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>

#include "system.h"

#ifdef OT_TI_KERNEL_freertos
#include <FreeRTOS.h>
#endif

#define SYSTICK64_ROLLOVER_DELAY      0x80000
#define SET_SYSTICK64_UPDATE_STATUS   0
#define CLEAR_SYSTICK64_UPDATE_STATUS 1

static uint32_t         Alarm_time0   = 0;
static uint32_t         Alarm_time    = 0;
static ClockP_Handle    Alarm_handle  = 0;
static ClockP_Struct    Alarm_Struct;
static ClockP_Params    Alarm_Params;
static bool             Alarm_running = false;

/* Upper 32 bits of the 64-bit SysTickCount */
static uint32_t upperSysTicks64    = 0;
static bool updatedUpperSysTicks64 = true;

/* Callback function to increment 64-bit counter on 32-bit counter overflow */
static void systemTicks64Callback(uintptr_t arg);

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
uint32_t ticksToMilli(uint64_t ticks)
{
    uint64_t milli = ticks;
    uint32_t tickPeriodUs = ClockP_getSystemTickPeriod();

    if(ticks != 0U)
    {
        milli = (ticks * tickPeriodUs) / 1000;
        if(milli == 0U)
        {
            milli = 1U;
        }
    }

    return((uint32_t)milli);
}

static void systemTicks64Callback(uintptr_t arg)
{
    uint32_t flag = (uint32_t)arg;

    if (flag == SET_SYSTICK64_UPDATE_STATUS)
    {
        upperSysTicks64++;
        updatedUpperSysTicks64 = true;
    }
    else if (flag == CLEAR_SYSTICK64_UPDATE_STATUS)
    {
        updatedUpperSysTicks64 = false;
    }
}

/*
 *  ======== ClockP_getSystemTicks64 ========
 */
uint64_t Alarm_getSystemTicks64(void)
{
    static ClockP_Struct sysTicks64SetFlag;
    static ClockP_Struct sysTicks64ClearFlag;
    static bool sysTicks64Initialised = false;
    ClockP_Params params;
    uint32_t lowerSysTicks64;
    uintptr_t key;
    uint64_t tickValue;

    /* Initialise clocks needed to maintain 64-bit SystemTicks when function
     * is called for the first time.
     */
    if (!sysTicks64Initialised)
    {

        ClockP_Params_init(&params);

        /* Start clock immediately when created */
        params.startFlag = true;
        /* Both clocks must trigger with same frequency as 32-bit overflow */
        params.period    = UINT32_MAX;

        uint32_t currentTick = ClockP_getSystemTicks();

        /* The clock which updates the upper 32-bit must not trigger before the
         * lower 32 bits have overflowed. The same clock will also set a flag
         * indicating that the upper 32 bits have been incremented.
         */
        uint32_t delayedStartSetFlag = UINT32_MAX - currentTick;

        /* The second clock will trigger shortly before the lower 32 bits
         * overflow, and clears the flag to indicate that the upper 32-bit
         * counter has not been incremented yet
         */
        uint32_t delayedStartClearFlag = UINT32_MAX - currentTick;

        if (delayedStartClearFlag <= SYSTICK64_ROLLOVER_DELAY)
        {
            /* If the current tick value is too close to the overflow, delay
             * the start of the clock until the next cycle. Manually clear flag.
             */
            delayedStartClearFlag += (UINT32_MAX - SYSTICK64_ROLLOVER_DELAY);
            updatedUpperSysTicks64 = false;
        }
        else
        {
            /* Make sure the flag is cleared well before the overflow occurs */
            delayedStartClearFlag -= SYSTICK64_ROLLOVER_DELAY;
        }

        /* Start both clocks with the same callback function, but different
         * flags. One clock will indicate that the upper 32 bits have
         * incremented shortly after the lower 32 bits overflow, and the other
         * clock will clear the flag shortly before the next overflow.
         */
        params.arg = SET_SYSTICK64_UPDATE_STATUS;
        ClockP_construct(&sysTicks64SetFlag, systemTicks64Callback, delayedStartSetFlag, &params);

        params.arg = CLEAR_SYSTICK64_UPDATE_STATUS;
        ClockP_construct(&sysTicks64ClearFlag, systemTicks64Callback, delayedStartClearFlag, &params);

        sysTicks64Initialised = true;
    }

    key = HwiP_disable();

    lowerSysTicks64 = ClockP_getSystemTicks();

    if ((lowerSysTicks64 < SYSTICK64_ROLLOVER_DELAY) && (updatedUpperSysTicks64 == false))
    {
        /* If the lower 32 bits have recently overflowed, but the upper 32 bits
         * have not yet been incremented then artificially increment upper bits
         */
        tickValue = ((uint64_t)(upperSysTicks64 + 1) << 32) | lowerSysTicks64;
    }
    else
    {
        /* In all other cases return the upper 32 bits + lower 32 bits as is */
        tickValue = ((uint64_t)upperSysTicks64 << 32) | lowerSysTicks64;
    }

    HwiP_restore(key);

    return tickValue;
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
    uint64_t ticks = Alarm_getSystemTicks64();

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

