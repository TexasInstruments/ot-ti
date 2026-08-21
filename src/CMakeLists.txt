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
    set(TI_SYSCONFIG_INSTALL_DIR $ENV{HOME}/ti/sysconfig_1.27.0 CACHE PATH "")
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
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genmap
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_Makefile
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_model.gv
    )
    set(GENLIBS_INCLUDES
    -L${TI_SIMPLELINK_SDK_DIR}/source
    -L${CMAKE_CURRENT_BINARY_DIR}/syscfg
    )


else()
    message(FATAL_ERROR "Unsuported TI_SIMPLELINK_KERNEL: ${TI_SIMPLELINK_KERNEL}")
endif()

if (OT_APP_NCP_BLE_CONTROLLER)
    list(APPEND SYSCONFIG_OUTPUT_C ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_dmm_application_policy.c)

    set(DMM_SRC
        dmm/dmm_thread_activity.c
    )

    if ( ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC2340R5*" OR ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC27(55|45)*")
        set(BLE_LIBS
            ble-stack-lib
            ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/lib/CC27XXX10/ControllerLib/lib/ticlang/m33f/ControllerLib.a
            ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        )

        set(DMM_INCLUDES
            ${CMAKE_CURRENT_SOURCE_DIR}/dmm/cc23xx_cc27xx
            ${CMAKE_CURRENT_SOURCE_DIR}/dmm
            ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm
            ${TI_SIMPLELINK_SDK_DIR}/source/ti
        )

        list(APPEND OT_PLATFORM_DEFINES USE_DMM USE_DMM_OVRDE USE_COMBINED_SERIAL)
    endif()
endif()

#if statement used for 2674 migration
if(${TI_SIMPLELINK_BOARD} MATCHES "CC2674*")
    set(syscfg_cmd ${TI_SYSCONFIG_CMD}
                    -s ${TI_SIMPLELINK_SDK_DIR}/.metadata/product.json
                    -o ${CMAKE_CURRENT_BINARY_DIR}/syscfg
                    --compiler gcc
                    --rtos ${TI_SIMPLELINK_KERNEL}
                    ${CMAKE_CURRENT_SOURCE_DIR}/CC2674.syscfg)

    set(sysconfig_file CC2674.syscfg)

elseif(${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC2340R5*" OR ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC27(55|45)*")
    set(syscfg_cmd ${TI_SYSCONFIG_CMD}
                    -s ${TI_SIMPLELINK_SDK_DIR}/.metadata/product.json
                    -o ${CMAKE_CURRENT_BINARY_DIR}/syscfg
                    --board /ti/boards/${TI_SIMPLELINK_BOARD}
                    --compiler gcc
                    --rtos ${TI_SIMPLELINK_KERNEL}
                    ${CMAKE_CURRENT_SOURCE_DIR}/cc23xx_cc27xx/openthread.syscfg)

    set(sysconfig_file openthread.syscfg)
    if(OT_APP_NCP_BLE_CONTROLLER)
        set(syscfg_cmd ${TI_SYSCONFIG_CMD}
                        -s ${TI_SIMPLELINK_SDK_DIR}/.metadata/product.json
                        -o ${CMAKE_CURRENT_BINARY_DIR}/syscfg
                        --board /ti/boards/${TI_SIMPLELINK_BOARD}
                        --compiler gcc
                        --rtos ${TI_SIMPLELINK_KERNEL}
                        ${CMAKE_CURRENT_SOURCE_DIR}/dmm/cc23xx_cc27xx/ncp_ble_controller.syscfg)
        set(sysconfig_file dmm/cc23xx_cc27xx/ncp_ble_controller.syscfg)
    endif()

else()
    set(syscfg_cmd ${TI_SYSCONFIG_CMD}
                    -s ${TI_SIMPLELINK_SDK_DIR}/.metadata/product.json
                    -o ${CMAKE_CURRENT_BINARY_DIR}/syscfg
                    --board /ti/boards/${TI_SIMPLELINK_BOARD}
                    --compiler gcc
                    --rtos ${TI_SIMPLELINK_KERNEL}
                    ${CMAKE_CURRENT_SOURCE_DIR}/openthread.syscfg)

    set(sysconfig_file openthread.syscfg)

endif()

# Create empty logging files in case user has completely disabled logging
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_log_config.c )
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_log_config.h )

list(APPEND SYSCONFIG_OUTPUT_C
        # SILK Log required dependencies
	${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_log_config.c
)

add_custom_command(
    OUTPUT
        ${SYSCONFIG_OUTPUT_C}
        ${SYSCONFIG_OUTPUT_OTHER}
    COMMAND
        ${syscfg_cmd}
    DEPENDS
        ${sysconfig_file} 
    VERBATIM
)

set(PLATFORM_SRCS
    ${SYSCONFIG_OUTPUT_C}
    alarm.c
    alarm_micro.c
    misc.c
    settings.c
    system.c
    system.h
    uart.c
    diag.c
    infra_if.c
)

if (OT_APP_NCP_BLE_CONTROLLER)
    list(APPEND PLATFORM_SRCS ${DMM_SRC})
    list(APPEND THREAD_LIBS ${BLE_LIBS})
endif()

if ( ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC2340R5*")
    list(APPEND PLATFORM_SRCS
        cc23xx_cc27xx/startup_cc23xx.c
        cc23xx_cc27xx/radio.c
        cc23xx_cc27xx/radio.h
        cc23xx_cc27xx/entropy.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/rcl_stub_dmm.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/rcl_override_dmm.c
        $<TARGET_OBJECTS:openthread-platform-utils>
    )
    set(DMM_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/dmm
    ${CMAKE_CURRENT_SOURCE_DIR}/dmm/cc23xx_cc27xx
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm
)   
    list(APPEND TI_PLATFORM_INCLUDES  ${CMAKE_CURRENT_SOURCE_DIR}/cc23xx_cc27xx  ${CMAKE_CURRENT_SOURCE_DIR} ${DMM_INCLUDES})


elseif ( ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC27(55|45)*")
    list(APPEND PLATFORM_SRCS
        cc23xx_cc27xx/startup_cc27xx.c
        cc23xx_cc27xx/radio.c
        cc23xx_cc27xx/radio.h
        cc23xx_cc27xx/entropy.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/rcl_stub_dmm.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/rcl_override_dmm.c
        $<TARGET_OBJECTS:openthread-platform-utils>
    )

    set(DMM_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/dmm

    ${CMAKE_CURRENT_SOURCE_DIR}/dmm/cc23xx_cc27xx
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm
)
    list(APPEND TI_PLATFORM_INCLUDES  ${CMAKE_CURRENT_SOURCE_DIR}/cc23xx_cc27xx  ${CMAKE_CURRENT_SOURCE_DIR} ${DMM_INCLUDES})

else()
        list(APPEND PLATFORM_SRCS
        spi_periph.c
        radio.c
        radio.h
        entropy.c
        $<TARGET_OBJECTS:openthread-platform-utils>
    )
    list(APPEND TI_PLATFORM_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR} ${DMM_INCLUDES})
endif()


if ( ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC2340R5*" OR  ${TI_SIMPLELINK_BOARD} MATCHES "LP_EM_CC27(55|45)*")
    list(APPEND PLATFORM_LIBRARIES
        cc23xx-driverlib
    )
    list(APPEND PLATFORM_LIBRARIES
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genmap
    )
else()
    list(APPEND PLATFORM_LIBRARIES
        cc13xx-cc26xx-driverlib
    )
endif()

set(THREAD_LIBS
    cc13xx-cc26xx-sdk
)



add_custom_target(build_syscfg DEPENDS ${SYSCONFIG_OUTPUT_C})
add_library(openthread-cc13xx_cc26xx
    ${SYSCONFIG_OUTPUT_C}
    ${PLATFORM_SRCS}
    $<TARGET_OBJECTS:openthread-platform-utils>
)


target_link_libraries(openthread-cc13xx_cc26xx
    PUBLIC
        ${THREAD_LIBS}
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<TARGET_PROPERTY:NAME>.map
        ${GENLIBS_INCLUDES}
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        ${PLATFORM_LIBRARIES}
    PRIVATE
        ot-config
)

target_compile_definitions(openthread-cc13xx_cc26xx
    PUBLIC
        ${OT_PLATFORM_DEFINES}
	    #TIOP_RADIO_USE_CFS
	    #PLAT_RADIO_SOFTWARE_IMM_ACK=1
        OT_TI_KERNEL_${TI_SIMPLELINK_KERNEL}
)

target_compile_options(openthread-cc13xx_cc26xx
PUBLIC
    @${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_compiler.opt
    PRIVATE
        ${OT_CFLAGS}
)


target_include_directories(openthread-cc13xx_cc26xx
    PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/syscfg
    ${TI_PLATFORM_INCLUDES}

    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        ${SYSCONFIG_INCLUDES}
        ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
)
add_dependencies(openthread-cc13xx_cc26xx build_syscfg)
