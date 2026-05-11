#
#  Copyright (c) 2019, The OpenThread Authors.
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

set(COMMON_INCLUDES
    ${OT_PUBLIC_INCLUDES}
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
    ${PROJECT_SOURCE_DIR}/openthread/src/core
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/combined_serial
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/combined_serial/embedded
)

# BLE OPT files must occur in application build context as they override the library definitions
set(BLE_CONTROLLER_OPT_FILES
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_padv.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_pscan.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_cs.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_cs_test.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_initiator.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_connectable.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_scanner.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_cm.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_power_control.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_rssi_monitor.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_connection_handover.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_legacy_cmd.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_health_check.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_adv_conn.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_adv_nconn.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_ae.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_adaptivity.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_ble_health.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_dmm.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_dmm_dynamic_priority.c
    dmm_priority_ble_thread.c
)

set(COMMON_SOURCES
    main.c
    ncp.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/config/src/ble_user_config.c
    ${BLE_CONTROLLER_OPT_FILES}
)

if(TI_SIMPLELINK_KERNEL STREQUAL "freertos")
    list(APPEND COMMON_SOURCES freertos_main.c)
else()
    message(FATAL_ERROR "Unsupported TI_SIMPLELINK_KERNEL: ${TI_SIMPLELINK_KERNEL}")
endif()

if(OT_APP_NCP_BLE_CONTROLLER)
    # ble-stack-lib must be created exactly once, before any executable targets
    include(ble.cmake)
    if(OT_FTD)
        include(ncp_ble_ftd.cmake)
        message("Building DMM NCP BLE FTD")
    endif()
    if(OT_MTD)
        include(ncp_ble_mtd.cmake)
        message("Building DMM NCP BLE MTD")
    endif()
endif()
