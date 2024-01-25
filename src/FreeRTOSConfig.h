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
    See http://www.freertos.org/a00110.html for an explanation of the
    definitions contained in this file.
******************************************************************************/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 * http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* General options */
#define configCPU_CLOCK_HZ ((unsigned long)(48000000))

/* Stack sizes, all in words */
#define configMINIMAL_STACK_SIZE ((unsigned short)(512))
#define configIDLE_TASK_STACK_DEPTH ((unsigned short)(512))
#define configPOSIX_STACK_SIZE ((unsigned short)(configMINIMAL_STACK_SIZE * 2))

/* Device specific options options */
#if defined(DeviceFamily_CC13X2_CC26X2) \
    || defined(DeviceFamily_CC13X2) \
    || defined(DeviceFamily_CC26X2)
#define configTOTAL_HEAP_SIZE ((size_t)(0x4000))
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#elif defined(DeviceFamily_CC13X2X7_CC26X2X7) \
    || defined(DeviceFamily_CC13X2X7) \
    || defined(DeviceFamily_CC26X2X7)
#define configTOTAL_HEAP_SIZE ((size_t)(0x14000))
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#elif defined(DeviceFamily_CC13X4_CC26X4) \
    || defined(DeviceFamily_CC13X4) \
    || defined(DeviceFamily_CC26X4)
//#define configTOTAL_HEAP_SIZE ((size_t)(0x14000))
#define configTOTAL_HEAP_SIZE ((size_t) (0))
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
/* TrustZone/PSA settings */
/* We do not set ENABLE_TRUSTZONE, as this is only for Secure Side function call support */
#define configENABLE_TRUSTZONE 0
#define configRUN_FREERTOS_SECURE_ONLY 1
/* ARM-v8m specific settings: floating point unit is enabled, memory protection unit is disabled */
#define configENABLE_FPU 1
#define configENABLE_MPU 0
/* The CM33 port requires an additional stack size definition */
#define configMINIMAL_SECURE_STACK_SIZE configMINIMAL_STACK_SIZE

#else
#error "unknown SimpleLink DeviceFamily, consult SDK configuration"

#endif

#define configCHECK_FOR_STACK_OVERFLOW 2
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP 2

/* Software timer definitions. */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (6)
#define configTIMER_QUEUE_LENGTH (20)
#define configTIMER_TASK_STACK_DEPTH ((unsigned short)(configMINIMAL_STACK_SIZE * 2))
/*
 * The ISR stack will be initialized in the startup_<device>_<compiler>.c file
 * to 0xa5a5a5a5. The stack peak can then be displayed in Runtime Object View.
 */
#define configENABLE_ISR_STACK_INIT 1
/* Enable debugging features in the UI to use the queue registry or ROV features */
#define configQUEUE_REGISTRY_SIZE 0

#define configASSERT(x)           \
    if ((x) == 0)                 \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for (;;)                  \
            ;                     \
    }

/* Constants related to the behaviour or the scheduler. */
#define configTICK_RATE_HZ ((TickType_t)100000)
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0
#define configMAX_PRIORITIES (10UL)
#define configIDLE_SHOULD_YIELD 0
#define configUSE_16_BIT_TICKS 0 /* Only for 8 and 16-bit hardware. */

/* Constants that describe the hardware and memory usage. */
#define configMAX_TASK_NAME_LEN (12)
#define configRECORD_STACK_HIGH_ADDRESS 1

/* Required by TI driver implementations */
#define configSUPPORT_STATIC_ALLOCATION 1
#define configAPPLICATION_ALLOCATED_HEAP 1

/* Constants that build features in or out. */
#define configUSE_MUTEXES 1
#define configUSE_TICKLESS_IDLE 1
#define configUSE_APPLICATION_TASK_TAG 1 /* Needed by POSIX/pthread */
#define configUSE_CO_ROUTINES 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_QUEUE_SETS 0
#define configUSE_TASK_NOTIFICATIONS 1

/* Constants that define which hook (callback) functions should be used. */
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_MALLOC_FAILED_HOOK 0

/* Constants provided for debugging and optimisation assistance. */
#define configENABLE_BACKWARD_COMPATIBILITY 1

#if defined(__TI_COMPILER_VERSION__) || defined(__ti_version__)
#include <ti/posix/freertos/PTLS.h>
#define traceTASK_DELETE(pxTCB) PTLS_taskDeleteHook(pxTCB)
#elif defined(__IAR_SYSTEMS_ICC__)
#ifndef __IAR_SYSTEMS_ASM__
#include <ti/posix/freertos/Mtx.h>
#define traceTASK_DELETE(pxTCB) Mtx_taskDeleteHook(pxTCB)
#endif
#endif

/*
 *  Enable thread local storage
 *
 *  Assign TLS array index ownership here to avoid collisions.
 *  TLS storage is needed to implement thread-safe errno with
 *  TI and IAR compilers. With GNU compiler, we enable newlib.
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__ti_version__) || defined(__IAR_SYSTEMS_ICC__)

#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 2

#if defined(__TI_COMPILER_VERSION__) || defined(__ti_version__)
#define PTLS_TLS_INDEX 0 /* ti.posix.freertos.PTLS */
#elif defined(__IAR_SYSTEMS_ICC__)
#define MTX_TLS_INDEX 0 /* ti.posix.freertos.Mtx */
#endif

#define NDK_TLS_INDEX 1 /* Reserve an index for NDK TLS */

#elif defined(__GNUC__)

#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1

#define NDK_TLS_INDEX 0 /* Reserve an index for NDK TLS */

/* note: system locks required by newlib are not implemented */
#define configUSE_NEWLIB_REENTRANT 1
#endif

/*
 * Set the following definitions to 1 to include the API function, or zero
 * to exclude the API function.  NOTE:  Setting an INCLUDE_ parameter to 0 is
 * only necessary if the linker does not automatically remove functions that
 * are not referenced anyway.
 */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskCleanUpResources 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#if !defined(DeviceFamily_CC13X2_CC26X2)
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#else
#define INCLUDE_uxTaskGetStackHighWaterMark 0
#endif
#define INCLUDE_xTaskGetIdleTaskHandle 0
#define INCLUDE_eTaskGetState 1
#define INCLUDE_xTaskResumeFromISR 0
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xSemaphoreGetMutexHolder 0
#define INCLUDE_xTimerPendFunctionCall 0

/* Cortex-M3/4 interrupt priority configuration follows...................... */

/* Use the system definition, if there is one. */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 3 /* 8 priority levels */
#endif

/*
 * The lowest interrupt priority that can be used in a call to a "set priority"
 * function.
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0x07

/*
 * The highest interrupt priority that can be used by any interrupt service
 * routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT
 * CALL INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A
 * HIGHER PRIORITY THAN THIS! (higher priorities are lower numeric values.
 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 1

/*
 * Interrupt priorities used by the kernel port layer itself.  These are generic
 * to all Cortex-M ports, and do not rely on any particular library functions.
 */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*
 * !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
 * See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html.
 *
 * Priority 1 (shifted 5 since only the top 3 bits are implemented).
 * Priority 1 is the second highest priority.
 * Priority 0 is the highest priority.
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*
 * The trace facility is turned on to make some functions available for use in
 * CLI commands.
 */
#define configUSE_TRACE_FACILITY 1

/*
 * Runtime Object View is a Texas Instrument host tool that helps visualize
 * the application. When enabled, the ISR stack will be initialized in the
 * startup_<device>_<compiler>.c file to 0xa5a5a5a5. The stack peak can then
 * be displayed in Runtime Object View.
 */
#define configENABLE_ISR_STACK_INIT 1
#endif /* FREERTOS_CONFIG_H */
