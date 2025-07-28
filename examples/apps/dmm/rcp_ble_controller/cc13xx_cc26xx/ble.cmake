#
#  Copyright (c) 2024, Texas Instruments Incorporated
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

add_library(ble-stack-lib
    hci_test_app.c
    osal_icall_ble.c
    icall_FreeRTOS.c # Updated based on latest BLE repo to use EventP rather than direct FreeRTOS semaphore for signaling
    icall_hci_tl.c # Copied latest from BLE Repo, for GCC mapped API calls to VA_ARGS must not be empty, replaced with NULL parameter
    icall_ble_apimsg.h # Updated with correct BLE Controller excluded defines
    ble_user_config.c
    Queue_freertos.c # Copied from BLE Repo
    ble_dispatch_JT.c # Updated to point to local icall_ble_api_msg.h
    npi_task.c # Copied from BLE Repo, updated to use EventP for FreeRTOS on Agama as well as Loki
    #${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom/agama_r1/rom_init.c
    osal.c #Updated to use SW ltoa implementation
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/icall_user_config.c

    dmm_priority_ble_thread.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/util.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/util.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/rf_api.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/npi/stack/npi.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/rcosc/rcosc_calibration.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/rcosc/rcosc_calibration.h
    #${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/app/icall_hci_tl.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/inc/icall.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/inc/icall_addrs.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/icall_cc2650.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/icall_platform.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/stack/ble_user_config_stack.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/app/icall_api_lite.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/crypto_api.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/ecc_api.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/rtos_drivers.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/trng_api.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/common/hal_assert.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_adc.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_assert.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_board.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_defs.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_key.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_lcd.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_led.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_sleep.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_timer.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc/hal_uart.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/hal_mcu.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/rf_hal.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/hal_board_cfg.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/hal_flash_wrapper.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/hal_rtc_wrapper.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/host/gap.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/host/gapbondmgr.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc/gapbondmgr.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc/gapgattserver.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/host/gatt_uuid.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc/gatt_uuid.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc/gattservapp.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/host/sm_ecc.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/icall_lite_translation.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/comdef.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_bufmgr.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_bufmgr.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_cbtimer.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_cbtimer.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_clock.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_clock.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_list.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_list.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_memory.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_memory_icall.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_pwrmgr.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_pwrmgr.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_snv.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/mcu/cc26xx/osal_snv_wrapper.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_task.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/common/osal_timers.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc/osal_timers.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom/agama_r1/common_rom_init.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom/agama_r1/rom_flash_jt.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/icall_startup.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/onboard.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/onboard.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/host/rtls_srv.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc/rtls_srv_api.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/hal_trng_wrapper.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/hal_trng_wrapper.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common/mb.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/inc/icall_user_config.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/npi_tl_spi.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_tl_spi.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/npi_tl_uart.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_tl_uart.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_config.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/npi_tl.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_tl.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_ble.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_frame.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/npi_frame_hci.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/npi_rxbuf.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_rxbuf.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc/npi_task.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/inc/ble_user_config.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/inc/ble_dispatch.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/stack/ble_dispatch_JT.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/stack/ble_dispatch_lite.c
)

set(GENERIC_BLE_INCLUDES 
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/npi/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/rcosc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/npi/stack
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/controller/cc26xx/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/heapmgr
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/dev_info
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/simple_profile
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/services/src/saddr
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/services/src/sdata
)

set(GENERIC_BLE_DEFINES 
PUBLIC
    HCI_TL_FULL
    CTRL_CONFIG=ADV_NCONN_CFG+ADV_CONN_CFG+SCAN_CFG+INIT_CFG
    BLE_VS_FEATURES=SCAN_REQ_RPT_CFG
    BLE_V50_FEATURES=PHY_2MBPS_CFG+HDC_NC_ADV_CFG+CHAN_ALGO2_CFG+AE_CFG+PHY_LR_CFG
    ICALL_LITE_12_PARAMS
    NO_OSAL_SNV
    EM_CC1354P10_6_LP
    CC13XX
    CC13X4
    FLASH_ONLY_BUILD
    # NVOCMP_NWSAMEITEM=1
    #NVOCMP_NVPAGES=4
    xdc_target__isaCompatible_v8M
    HEAPMGR_CONFIG=0x80
    HEAPMGR_SIZE=0x0
    CC13X2P
    FREERTOS
    NVOCMP_POSIX_MUTEX
    Display_DISABLE_ALL
    #HEAPMGR_METRICS # Cannot use heap metrics with icall_hci_tl.c due to TIRTOS Dependency
    ICALL_EVENTS
    ICALL_JT
    ICALL_LITE
    ICALL_MAX_NUM_ENTITIES=6
    ICALL_MAX_NUM_TASKS=3
    ICALL_STACK0_ADDR
    MAX_NUM_BLE_CONNS=2
    NPI_SPI_CONFIG=CONFIG_SPI_0
    xNPI_USE_SPI
    NPI_USE_UART
    xPOWER_SAVING
    STACK_LIBRARY
    NO_OSAL_SNV
    USE_ICALL
    OSAL_CBTIMER_NUM_TASKS=1
    ONE_BLE_LIB_SIZE_OPTIMIZATION
    xRTLS_CTE
    USE_DMM
    NPI_UART_BR=921600

    USE_AE
    LEGACY_CMD
    LL_TEST_MODE
    LL_TEST_CASE=64
    MAX_PDU_SIZE=255
    QUAL_TEST
    DISABLE_AUTO_FEATURE_REQ
    DISABLE_VS_EVENTS
    CONTROLLER_ONLY
    NPI_MAX_DATA

    BLE_STACK_TASK_PRIORITY=6
)
target_compile_definitions(ble-stack-lib
    PUBLIC
        ${GENERIC_BLE_DEFINES}
)

target_compile_options(ble-stack-lib
    PUBLIC
        ${OT_CFLAGS}
        @${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/config/build_components.opt
)

target_include_directories(ble-stack-lib INTERFACE ${GENERIC_BLE_INCLUDES})
target_compile_definitions(ble-stack-lib INTERFACE ${GENERIC_BLE_DEFINES})
target_compile_options(ble-stack-lib INTERFACE ${GENERIC_BLE_OPTIONS})

target_include_directories(ble-stack-lib
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GENERIC_BLE_INCLUDES}
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/devices/cc13x4_cc26x4/rf_patches
    ${TI_SIMPLELINK_SDK_DIR}/source
    ${TI_SIMPLELINK_SDK_DIR}/source/ti
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/thread/source/activity/
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/cc26xx
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/devices/cc13x4_cc26x4
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/gcc
)

target_link_libraries(ble-stack-lib
    PUBLIC
    openthread-cc13xx_cc26xx
)

#add_dependencies(ble-stack-lib build_syscfg)
add_dependencies(ble-stack-lib freertos)
add_dependencies(ble-stack-lib cc13xx-cc26xx-sdk)
