#
#  Copyright (c) 2021, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

if(NOT $ENV{TI_SIMPLELINK_SDK_DIR} STREQUAL "")
    set(TI_SIMPLELINK_SDK_DIR $ENV{TI_SIMPLELINK_SDK_DIR} CACHE PATH "")
else()
    set(TI_SIMPLELINK_SDK_DIR ${PROJECT_SOURCE_DIR}/third_party/ti_simplelink_sdk/repo_cc13xx_cc26xx CACHE PATH "")
endif()

# NOTE: posix source only needed for NVOCMP middleware
if(TI_SIMPLELINK_KERNEL STREQUAL "freertos")
    set(CC13XX_CC26XX_DPL_SRCS
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/startup/startup_${TI_SIMPLELINK_FAMILY}_gcc.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/AppHooks_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/ClockPCC26X2_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/ClockP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/DebugP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/HwiPCC26X2_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/MutexP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/PowerCC26X2_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/QueueP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/SemaphoreP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/StaticAllocs_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/SwiP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/SystemP_freertos.c
        ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos/dpl/TimerPCC26XX_freertos.c
    )
    set(CC13XX_CC26XX_POSIX_SRCS
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/aeabi_portable.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/clock.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/memory.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/mqueue.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/pthread.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/pthread_barrier.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/pthread_cond.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/pthread_mutex.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/pthread_rwlock.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/sched.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/semaphore.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/sleep.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/freertos/timer.c
    )

    if(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2653P10")
        set(CC13XX_CC26XX_LDS
            ${TI_SIMPLELINK_SDK_DIR}/source/ti/boards/cc26x3/cc26x3_freertos.lds
        )
    else()
        set(CC13XX_CC26XX_LDS
            ${TI_SIMPLELINK_SDK_DIR}/source/ti/boards/${TI_SIMPLELINK_DEVICE}/${TI_SIMPLELINK_DEVICE}_freertos.lds
        )
    endif()

else()
    message(FATAL_ERROR "Unsuported TI_SIMPLELINK_KERNEL: ${TI_SIMPLELINK_KERNEL}")
endif()

set(CC13XX_CC26XX_POSIX_INCLUDES
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/gcc
)

set(CC13XX_CC26XX_NVOCMP_SRCS
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv/crc.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv/nvintf.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv/nvocmp.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv/crc.c
    ${TI_SIMPLELINK_SDK_DIR}/../../../src/nvocmp.c
)

set(CC13XX_CC26XX_NVOCMP_DEFS
    NVOCMP_POSIX_MUTEX
    NVOCMP_NVPAGES=3
)

set(CC13XX_CC26XX_NVOCMP_INCLUDES
${TI_SIMPLELINK_SDK_DIR}/source/ti/devices/${TI_SIMPLELINK_DEVICE}
${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv
)

add_library(cc13xx-cc26xx-sdk)

target_sources(cc13xx-cc26xx-sdk
    PRIVATE
        ${CC13XX_CC26XX_DPL_SRCS}
        ${CC13XX_CC26XX_POSIX_SRCS}
        ${CC13XX_CC26XX_NVOCMP_SRCS}
)

target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC ${CC13XX_CC26XX_NVOCMP_DEFS})

if(TI_SIMPLELINK_DEVICE STREQUAL "cc13x2_cc26x2")
    target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC DeviceFamily_CC13X2_CC26X2)
    target_compile_definitions(ot-config INTERFACE DeviceFamily_CC13X2_CC26X2)

elseif(TI_SIMPLELINK_DEVICE STREQUAL "cc13x2x7_cc26x2x7")
    target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC DeviceFamily_CC13X2X7_CC26X2X7)
    target_compile_definitions(ot-config INTERFACE DeviceFamily_CC13X2X7_CC26X2X7)

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2653P10")
    target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC DeviceFamily_CC26X3)
    target_compile_definitions(ot-config INTERFACE DeviceFamily_CC26X3)

elseif(TI_SIMPLELINK_DEVICE STREQUAL "cc13x4_cc26x4")
    target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC DeviceFamily_CC13X4_CC26X4)
    target_compile_definitions(ot-config INTERFACE DeviceFamily_CC13X4_CC26X4)

    if(${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC13*")
        target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC DeviceFamily_CC13X4)
    else()
        target_compile_definitions(cc13xx-cc26xx-sdk PUBLIC DeviceFamily_CC26X4)
    endif()

else()
    message(FATAL_ERROR "Unsuported TI_SIMPLELINK_DEVICE: ${TI_SIMPLELINK_DEVICE}")
endif()

target_include_directories(cc13xx-cc26xx-sdk
    PUBLIC
        ${TI_PLATFORM_INCLUDES}
        ${CC13XX_CC26XX_DPL_INCLUDES}
        ${TI_SIMPLELINK_SDK_DIR}/source
        ${CC13XX_CC26XX_NVOCMP_INCLUDES}
        ${CC13XX_CC26XX_POSIX_INCLUDES}
)

target_include_directories(ot-config INTERFACE
    ${TI_PLATFORM_INCLUDES}
    ${TI_SIMPLELINK_SDK_DIR}/source
)

add_library(cc13xx-cc26xx-driverlib INTERFACE)

target_link_libraries(cc13xx-cc26xx-driverlib
    INTERFACE
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/devices/${TI_SIMPLELINK_DEVICE}/driverlib/bin/gcc/driverlib.lib
)

if(TI_SIMPLELINK_KERNEL STREQUAL "freertos")
    target_link_libraries(cc13xx-cc26xx-sdk PUBLIC freertos)
endif()

target_link_libraries(cc13xx-cc26xx-sdk
    PUBLIC
        cc13xx-cc26xx-driverlib
        -Wl,--undefined=__ccfg
        -T${CC13XX_CC26XX_LDS}
)
