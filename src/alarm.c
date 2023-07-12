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

/* POSIX Header files */
#include <time.h>

/* OpenThread public API Header files */
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>

#include "system.h"

#ifdef OT_TI_KERNEL_freertos
#include <FreeRTOS.h>
#endif

static uint32_t Alarm_time0   = 0;
static uint32_t Alarm_time    = 0;
static timer_t  Alarm_timerid = 0;
static bool     Alarm_running = false;

/**
 * Handler for the POSIX clock callback.
 */
void Alarm_handler(union sigval val)
{
    (void)val;
    platformAlarmSignal();
}

/**
 * Function documented in system.h
 */
void platformAlarmInit(void)
{
    struct timespec zeroTime = {0};
    struct sigevent event =
    {
        .sigev_notify_function = Alarm_handler,
        .sigev_notify = SIGEV_SIGNAL,
    };

    clock_settime(CLOCK_MONOTONIC, &zeroTime);

    timer_create(CLOCK_MONOTONIC, &event, &Alarm_timerid);

    Alarm_running = false;
}

/**
 * Function documented in platform/alarm-milli.h
 */
uint32_t otPlatAlarmMilliGetNow(void)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec * 1000U) + ((now.tv_nsec / 1000000U) % 1000);
}

/**
 * Function documented in platform/alarm-milli.h
 */
void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;
    struct itimerspec timerspec = {0};
    uint32_t          delta     = (otPlatAlarmMilliGetNow() - aT0);

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
        /* The timer is based on the FreeRTOS Kernel tick. If the RTOS is
         * ticking faster than 1ms a sufficiently large number will overflow the
         * tick count for the timer's timeout. This happens when the uptime.cpp
         * counter is registered. We'll let the processing function re-register
         * the timer with the remaining time.
         */
        if (configTICK_RATE_HZ > 1000U)
        {
            // FreeRTOS is ticking faster than 1ms and may overflow the timer
            if (timeout > UINT32_MAX / (configTICK_RATE_HZ / 1000U))
            {
                timeout = UINT32_MAX / (configTICK_RATE_HZ / 1000U);
            }
        }
#endif

        timerspec.it_value.tv_sec  = (timeout / 1000U);
        timerspec.it_value.tv_nsec = ((timeout % 1000U) * 1000000U);

        timer_settime(Alarm_timerid, 0, &timerspec, NULL);
    }
}

/**
 * Function documented in platform/alarm-milli.h
 */
void otPlatAlarmMilliStop(otInstance *aInstance)
{
    (void)aInstance;
    struct itimerspec zeroTime = {0};

    timer_settime(Alarm_timerid, TIMER_ABSTIME, &zeroTime, NULL);
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

