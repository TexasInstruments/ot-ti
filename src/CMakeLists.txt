#
#  Copyright (c) 2021, Texas Instruments Incorporated
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

if(NOT $ENV{TI_SYSCONFIG_ROOT} STREQUAL "")
    set(TI_SYSCONFIG_INSTALL_DIR $ENV{TI_SYSCONFIG_ROOT} CACHE PATH "")
else()
    set(TI_SYSCONFIG_INSTALL_DIR $ENV{HOME}/ti/sysconfig_1.15.0 CACHE PATH "")
endif()

if(WIN32)
    set(TI_SYSCONFIG_CMD ${TI_SYSCONFIG_INSTALL_DIR}/sysconfig_cli.bat CACHE PATH "")
else()
    set(TI_SYSCONFIG_CMD ${TI_SYSCONFIG_INSTALL_DIR}/sysconfig_cli.sh CACHE PATH "")
endif()

# NOTE: posix source only needed for NVOCMP middleware
if(TI_SIMPLELINK_KERNEL STREQUAL "freertos")
    set(SYSCONFIG_OUTPUT_C
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_devices_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_devices_config.h
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_drivers_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_drivers_config.h
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_radio_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_radio_config.h

        # TI SimpleLink FreeRTOS configuration and build is not possible until
        # configTICK_RATE_HZ is configurable and source directory set correctly.
        #${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_freertos_config.c
        #${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_freertos_portable_config.c
        #${CMAKE_CURRENT_BINARY_DIR}/syscfg/FreeRTOSConfig.h
    )
    set(SYSCONFIG_OUTPUT_OTHER
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/syscfg_c.rov.xs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_build_config.opt
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_Makefile
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_model.gv
    )
    set(GENLIBS_INCLUDES
        -L${TI_SIMPLELINK_SDK_DIR}/source
    )

elseif(TI_SIMPLELINK_KERNEL STREQUAL "tirtos7")
    set(SYSCONFIG_OUTPUT_C
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_drivers_config.h
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_radio_config.h
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_sysbios_config.h
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_devices_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_drivers_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_radio_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_sysbios_config.c
    )
    set(SYSCONFIG_OUTPUT_OTHER
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/syscfg_c.rov.xs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_Makefile
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_model.gv
    )
    set(SYSCONFIG_INCLUDES
        ${TI_SIMPLELINK_SDK_DIR}/source
        ${TI_SIMPLELINK_SDK_DIR}/kernel/tirtos7/packages
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/gcc
    )
    set(GENLIBS_INCLUDES
        -L${TI_SIMPLELINK_SDK_DIR}/source
        -L${TI_SIMPLELINK_SDK_DIR}/kernel/tirtos7/packages
    )

else()
    message(FATAL_ERROR "Unsuported TI_SIMPLELINK_KERNEL: ${TI_SIMPLELINK_KERNEL}")
endif()

add_custom_command(
    OUTPUT
        ${SYSCONFIG_OUTPUT_C}
        ${SYSCONFIG_OUTPUT_OTHER}
    COMMAND
        ${TI_SYSCONFIG_CMD}
            -s ${TI_SIMPLELINK_SDK_DIR}/.metadata/product.json
            -o ${CMAKE_CURRENT_BINARY_DIR}/syscfg
            --compiler gcc
            --board /ti/boards/${TI_SIMPLELINK_BOARD}
            --rtos ${TI_SIMPLELINK_KERNEL}
            ${CMAKE_CURRENT_SOURCE_DIR}/openthread.syscfg
    DEPENDS
        openthread.syscfg
    VERBATIM
)

add_library(openthread-cc13xx_cc26xx
    ${SYSCONFIG_OUTPUT_C}
    alarm.c
    alarm_micro.c
    diag.c
    entropy.c
    misc.c
    radio.c
    radio.h
    settings.c
    spi_periph.c
    system.c
    system.h
    uart.c
    $<TARGET_OBJECTS:openthread-platform-utils>
)

list(APPEND TI_PLATFORM_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(openthread-cc13xx_cc26xx
    PUBLIC
        cc13xx-cc26xx-sdk
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<TARGET_PROPERTY:NAME>.map
        ${GENLIBS_INCLUDES}
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        cc13xx-cc26xx-driverlib
    PRIVATE
        ot-config
)

target_compile_definitions(openthread-cc13xx_cc26xx
    PUBLIC
        ${OT_PLATFORM_DEFINES}
)

target_compile_options(openthread-cc13xx_cc26xx
    PRIVATE
        ${OT_CFLAGS}
)

target_include_directories(openthread-cc13xx_cc26xx
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg
    PRIVATE
        ${OT_PLATFORM_INCLUDES}
        ${OT_PUBLIC_INCLUDES}
        ${SYSCONFIG_INCLUDES}
        ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
)

