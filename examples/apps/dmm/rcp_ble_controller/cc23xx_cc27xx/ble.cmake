#
#  Copyright (c) 2025, Texas Instruments Incorporated
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


set(BLE_CONTROLLER_SOURCE_FILES

    app_main.c
    Queue_freertos.c
    freertos_main.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/config/src/ble_user_config.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/config/ble_user_config.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/config/src/hci_supported_cmd.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/controller/hci/hci_supported_cmd.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/src/npi_tl_uart.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_tl_uart.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_config.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/src/npi_tl.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_tl.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_ble.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_frame.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/src/npi_frame_hci.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/src/npi_rxbuf.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_rxbuf.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/src/npi_task.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/npi/npi_task.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/health_toolkit/debugInfo.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/health_toolkit/debugInfo_errno.h
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/health_toolkit/src/debugInfo.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/icall/app/src/icall_POSIX.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/icall/app/icall_addrs.h
    
)
add_library(ble-stack-lib
    ${BLE_CONTROLLER_SOURCE_FILES}
)

set(GENERIC_BLE_INCLUDES 
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}

    ${TI_SIMPLELINK_SDK_DIR}/source
    ${TI_SIMPLELINK_SDK_DIR}/source/ti
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/cc26xx
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/drivers/rcl
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/gcc
    ${TI_SIMPLELINK_SDK_DIR}/source/third_party/freertos/include
    ${TI_SIMPLELINK_SDK_DIR}/source/third_party/freertos/portable/GCC/ARM_CM33_NTZ/non_secure
    ${TI_SIMPLELINK_SDK_DIR}/kernel/freertos
)

set(GENERIC_BLE_DEFINES 
PUBLIC
    CC23X0
    USE_HSM
    NVOCMP_NWSAMEITEM=1
    FREERTOS
    NVOCMP_POSIX_MUTEX
    NPI_UART_BR=921600

    USE_DMM
    USE_DMM_DYNAMIC_PRIORITY
    USE_DMM_OVRDE

)
target_compile_definitions(ble-stack-lib
    PUBLIC
        ${GENERIC_BLE_DEFINES}
)

target_compile_options(ble-stack-lib
    PUBLIC
        ${OT_CFLAGS}
        @${CMAKE_CURRENT_SOURCE_DIR}/ble_controller.opt
        @${CMAKE_CURRENT_SOURCE_DIR}/build_config.opt
        @${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/config/build_components.opt
    
)

target_include_directories(ble-stack-lib INTERFACE ${GENERIC_BLE_INCLUDES})
target_compile_definitions(ble-stack-lib INTERFACE ${GENERIC_BLE_DEFINES})
target_compile_options(ble-stack-lib INTERFACE ${GENERIC_BLE_OPTIONS})

target_include_directories(ble-stack-lib
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GENERIC_BLE_INCLUDES}
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/posix/gcc
)

target_link_libraries(ble-stack-lib
    PUBLIC
    openthread-cc13xx_cc26xx

)

add_dependencies(ble-stack-lib freertos)
add_dependencies(ble-stack-lib cc13xx-cc26xx-sdk)