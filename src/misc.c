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
#include OPENTHREAD_PROJECT_CORE_CONFIG_FILE


#include <core/config/logging.h>
#include <openthread/logging.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/misc.h>

// clang-format off
#include <ti/devices/DeviceFamily.h>
#if !defined(DeviceFamily_CC23X0R5) && !defined (DeviceFamily_CC27XX)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/flash.h)
#else
#include DeviceFamily_constructPath(driverlib/pmctl.h)
#endif
// clang-format on
#include <FreeRTOS.h>
#include <task.h>

#include <ti/drivers/NVS.h>
#include <ti/log/Log.h>
#include "ti_drivers_config.h"

#if OPENTHREAD_CONFIG_PLATFORM_ASSERT_MANAGEMENT
#define MAIN_ASSERT_THREAD  7
#define FAULT_STORE_ADDRESS 0x000ED800
#define CORE_DUMP_SIZE 32
void *nvsDataWrite(uint32_t* CoreDumpData, size_t core_dump_size);
#endif
void *nvsDataDump(void);

#if !defined(DeviceFamily_CC23X0R5) && !defined (DeviceFamily_CC27XX)

void otPlatReset(otInstance *aInstance)
{
    (void)aInstance;
    SysCtrlSystemReset();
}

otPlatResetReason otPlatGetResetReason(otInstance *aInstance)
{
    (void)aInstance;

    switch (SysCtrlResetSourceGet())
    {
    case RSTSRC_PWR_ON:
    {
        return OT_PLAT_RESET_REASON_POWER_ON;
    }

    case RSTSRC_PIN_RESET:
    {
        return OT_PLAT_RESET_REASON_EXTERNAL;
    }

    case RSTSRC_VDDS_LOSS:
    case RSTSRC_VDDR_LOSS:
    case RSTSRC_CLK_LOSS:
    {
        return OT_PLAT_RESET_REASON_CRASH;
    }

    case RSTSRC_WARMRESET:
    case RSTSRC_SYSRESET:
    case RSTSRC_WAKEUP_FROM_SHUTDOWN:
    {
        return OT_PLAT_RESET_REASON_SOFTWARE;
    }

    default:
    {
        return OT_PLAT_RESET_REASON_UNKNOWN;
    }
    }
}
#else

void otPlatReset(otInstance *aInstance)
{
    (void)aInstance;
    PMCTLResetSystem();
}

otPlatResetReason otPlatGetResetReason(otInstance *aInstance)
{
    (void)aInstance;

    switch (PMCTLGetResetReason())
    {
    case PMCTL_RESET_POR:
    {
        return OT_PLAT_RESET_REASON_POWER_ON;
    }

    case PMCTL_RESET_PIN:
    {
        return OT_PLAT_RESET_REASON_EXTERNAL;
    }

    case PMCTL_RESET_VDDS:
    case PMCTL_RESET_VDDR:
    case PMCTL_RESET_LFXT:
    {
        return OT_PLAT_RESET_REASON_CRASH;
    }

    case PMCTL_RESET_SYSTEM:
    case PMCTL_RESET_SHUTDOWN_SWD:
    {
        return OT_PLAT_RESET_REASON_SOFTWARE;
    }

    default:
    {
        return OT_PLAT_RESET_REASON_UNKNOWN;
    }
    }
}
#endif
void otPlatWakeHost(void)
{
}

#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED
void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
}
#endif

otError otPlatResetToBootloader(otInstance *aInstance)
{
    (void)aInstance;

    // Reset the system
    SysCtrlSystemReset();

    return OT_ERROR_NONE;
}


#if OPENTHREAD_CONFIG_PLATFORM_ASSERT_MANAGEMENT

void otPlatAssertFail(const char *aFilename, int aLineNumber)
{
    uint32_t CoreDumpData[CORE_DUMP_SIZE];
    uint32_t exc_lr_thread;
    uint32_t exc_sp_thread;

    uint16_t i;
    uint32_t *pAddr;

     __asm volatile (
        "str sp, %0 \n" // Store SP into exc_sp_thread
        "str lr, %1 \n" // Store LR into exc_lr_thread
        : "=m" (exc_sp_thread), "=m" (exc_lr_thread) // Output operands
    );

        /* save the following content to Flash (words)
        Reason
        LR (if hard fault, find LR in Stack dump)
        SP
        contents of SP (16 words) - IF ASSERT

        HWI HardFault: content of SP - IF HARDFAULT
        R4,R5,R6,R7, R8,R9,R10,R11
        R0,R1,R2,R3, R12,LR, PC,xPRS

    */
    CoreDumpData[0] = (uint8_t)MAIN_ASSERT_THREAD;
    CoreDumpData[1] = exc_lr_thread;
    CoreDumpData[2] = exc_sp_thread;
    pAddr = (uint32_t*)exc_sp_thread;
    // try to avoid the memcpy function
    for (i=0;i<16;i++)
    {
        CoreDumpData[3+i] = pAddr[i];
    }

    nvsDataWrite(CoreDumpData, sizeof(CoreDumpData));
}
#endif //OPENTHREAD_CONFIG_PLATFORM_ASSERT_MANAGEMENT

#if OPENTHREAD_CONFIG_PLATFORM_LOG_CRASH_DUMP_ENABLE
otError otPlatLogCrashDump(void)
{
    /* Define the maximum number of tasks you expect */
    const UBaseType_t uxArraySize = 10;
    TaskStatus_t pxTaskStatusArray[10];
    UBaseType_t uxTaskCount;
    UBaseType_t x;
    uint32_t ulTotalRunTime;
    uxTaskCount = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
    /* Iterate through each task and print its stack high water mark */
    for (x = 0; x < uxTaskCount; x++)
    {
        otLogCritPlat("Task: %s, Minimum free stack space: %u bytes", pxTaskStatusArray[x].pcTaskName,
                pxTaskStatusArray[x].usStackHighWaterMark);
    }

    size_t xFreeHeapSpace            = xPortGetFreeHeapSize();
    size_t xMinimumEverFreeHeapSpace = xPortGetMinimumEverFreeHeapSize();
    (void) xFreeHeapSpace;
    (void) xMinimumEverFreeHeapSpace;
    /* Print the heap usage statistics */
    otLogCritPlat("Current free heap space: %u bytes", (unsigned int) xFreeHeapSpace);
    otLogCritPlat("Minimum ever free heap space: %u bytes", (unsigned int) xMinimumEverFreeHeapSpace);

    //dump crash data if it exists
    nvsDataDump();

    return OT_ERROR_NONE;
}

void *nvsDataWrite(uint32_t* CoreDumpData, size_t core_dump_size)
{
    NVS_Handle nvsHandle;
    NVS_Attrs regionAttrs;
    NVS_Params nvsParams;
    uint_fast16_t status;
    NVS_init();

    NVS_Params_init(&nvsParams);
    nvsHandle = NVS_open(CONFIG_NVS_DBG_INTERNAL, &nvsParams);

    if (nvsHandle == NULL)
    {
        // NVS_open() failed
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "NVS open fail");
        return(NULL);
    }

    // Get NVS attribute structure and define DEBUG region base
    NVS_getAttrs(nvsHandle, &regionAttrs);

    NVS_erase(nvsHandle, 0, regionAttrs.regionSize);
    
    status = NVS_write(nvsHandle, 0, (uint8_t*)CoreDumpData, core_dump_size, NVS_WRITE_POST_VERIFY);
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "NVS write fail");
    }

    // Close NVS driver
    NVS_close(nvsHandle);
}

void *nvsDataDump(void)
{
    // Open NVS for DEBUG
    NVS_Handle nvsHandle;
    NVS_Attrs regionAttrs;
    NVS_Params nvsParams;
    NVS_init();

    NVS_Params_init(&nvsParams);
    nvsHandle = NVS_open(CONFIG_NVS_DBG_INTERNAL, &nvsParams);

    if (nvsHandle == NULL)
    {
        // NVS_open() failed
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "NVS open fail");
        return(NULL);
    }

    // Get NVS attribute structure and define DEBUG region base
    NVS_getAttrs(nvsHandle, &regionAttrs);
    const uint32_t* debugDataBuffer = (uint32_t*)regionAttrs.regionBase;

    // Check whether address range is empty (0xFFFFFFFF)
    uint32_t emptyMatch = 0xFFFFFFFF;
    uint8_t wordCount = 32;
    uint8_t numBytesToCheck = (sizeof(uint32_t) * wordCount);
    uint32_t *pWord = (uint32_t *)debugDataBuffer;
    uint32_t *pEnd = (uint32_t *)((uint32_t)debugDataBuffer + numBytesToCheck);
    while (pWord < pEnd)
    {
        emptyMatch &= *(pWord++);
    }

    // If the entire address range is 0xFFFFFFFF, then the NVS region is empty
    if (emptyMatch != 0xFFFFFFFF)
    {
        // NVS region has debug data. Write to logsink
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Core Data Dump Found!");

        uint8_t i = 0;
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump Reason = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump LR = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 1 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 2 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 3 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 4 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 5 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 6 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 7 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 8 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 9 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 10 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 11 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 12 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 13 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 14 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 15 = 0x%08X", debugDataBuffer[i++]);
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Data Dump SP Val 16 = 0x%08X", debugDataBuffer[i++]);

        // Clear NVS region after logging data dump
        NVS_erase(nvsHandle, 0, regionAttrs.regionSize);
    }
    else 
    {
        otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_PLATFORM, "Core Data Dump Not Found");
    }

    // Close NVS driver
    NVS_close(nvsHandle);
}
#endif // OPENTHREAD_CONFIG_PLATFORM_LOG_CRASH_DUMP_ENABLE
