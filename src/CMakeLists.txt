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

        # JJM - adding the BLE configuration source 
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_ble_config.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_ble_config.h


        # TI SimpleLink FreeRTOS configuration and build is not possible until
        # configTICK_RATE_HZ is configurable and source directory set correctly.
        #${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_freertos_config.c
        #${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_freertos_portable_config.c
        #${CMAKE_CURRENT_BINARY_DIR}/syscfg/FreeRTOSConfig.h
    )

    # JJM - System Cofiguration Include Paths 
    set(SYSCONFIG_INCLUDES
        #DMM
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/thread/platform
        ${TI_SIMPLELINK_SDK_DIR}/source/ti
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/thread
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/thread/source/activity
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv

        #
        # JJM - Note: the include paths below are a mirror (or an attempt to mirror)
        # the /Includes folder from the CCS simple-peripheral project
        # 

        # BLE
        # JJM - including /src here to avoid duplicates of FreeRTOSConfig.h. I think this is ok? 
        ${PROJECT_SOURCE_DIR}/src

        # BLE (cont)
        ${TI_SIMPLELINK_SDK_DIR}/kernel/tirtos7/packages
        ${TI_SIMPLELINK_SDK_DIR}/source
        ${TI_SIMPLELINK_SDK_DIR}/source/ti
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/npi/stack/
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/rcosc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/controller/cc26xx/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/heapmgr
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/dev_info
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/simple_profile
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/services/src/saddr
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/services/src/sdata
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/cc26xx
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/devices/cc13x4_cc26x4
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack/common/cc26xx/freertos
    )

    # JJM - setup the source files required for DMM builds 
    set(DMM_OUTPUT_C
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/thread/source/activity/dmm_thread_activity.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_dmm_application_policy.c
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_dmm_application_policy.h
    )

    # JJM - setup the source files for BLE
    set(BLE_OUTPUT_C
        # JJM - mirror simple peripheral iCall folder
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/icall_cc2650.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/icall_user_config.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/freertos/icall_FreeRTOS.c

        # JJM - mirror simple peripheral iCallBLE folder 
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/stack/ble_user_config_stack.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/app/ble_user_config.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/app/icall_api_lite.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/app/icall_hci_tl.c 

        # JJM - mirror the simple peripheral Profiles folder
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/dev_info/cc26xx/devinfoservice.c
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/simple_profile/cc26xx/simple_gatt_profile.c
    )

    set(SYSCONFIG_OUTPUT_OTHER
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/syscfg_c.rov.xs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_build_config.opt
        # JJM - this doesn't seem to work for some reason, had to add to User Defs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_ble_app_config.opt
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_Makefile
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_runtime_model.gv

        # mirrors the simple-peripheral Startup folder
        # JJM - cannot get this to link for stack_main() 
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/ble_remote_display/stack/osal_icall_ble.c
    )
    set(GENLIBS_INCLUDES
        -L${TI_SIMPLELINK_SDK_DIR}/source
    )

elseif(TI_SIMPLELINK_KERNEL STREQUAL "tirtos7")

    # JJM - not used, using FreeRTOS 

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
        ${DMM_OUTPUT_C}
        ${BLE_OUTPUT_C}
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
    ${DMM_OUTPUT_C}
    ${BLE_OUTPUT_C}
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

# retargeted 
target_link_libraries(openthread-cc13xx_cc26xx
    PUBLIC
        cc13xx-cc26xx-sdk
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<TARGET_PROPERTY:NAME>.map
        ${GENLIBS_INCLUDES}
        ${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
        cc13xx-cc26xx-driverlib
        -Wl,${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_linker.cmd.genlibs
    PRIVATE
        ot-config
)

# JJM - setting the user compile definitions here as there seems to be 
# an issue with the .opt files 
set(USR_COMPILE_DEFINITIONS
    "USE_DMM"
    "SYSCFG"
    "CC13X4"
    "CC13XX"
    "POWER_SAVING"
    "TBM_ACTIVE_ITEMS_ONLY"
    "STACK_LIBRARY"
    "EXTENDED_STACK_SETTINGS=EXTENDED_STACK_SETTINGS_DEFAULT"
    "NPI_USE_UART"
    "ICALL_EVENTS"
    "ICALL_JT"
    "ICALL_LITE"
    "ICALL_STACK0_ADDR"
    "USE_ICALL"
    "ICALL_MAX_NUM_ENTITIES=6"
    "ICALL_MAX_NUM_TASKS=3"
    "OSAL_CBTIMER_NUM_TASKS=1"
    "ONE_BLE_LIB_SIZE_OPTIMIZATION"

    # the following are from ti_build_config.opt 
    "HOST_CONFIG=PERIPHERAL_CFG"
    "USE_AE"
    "GAP_BOND_MGR"
    "HCI_TL_NONE"
    "FREERTOS"
)

target_compile_definitions(openthread-cc13xx_cc26xx
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        OT_TI_KERNEL_${TI_SIMPLELINK_KERNEL}
        # JJM - add in the user compile definitions 
        ${USR_COMPILE_DEFINITIONS}
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

