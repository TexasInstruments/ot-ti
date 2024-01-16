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
    ${TI_PUBLIC_INCLUDES}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
    ${PROJECT_SOURCE_DIR}/openthread/src/core

    # JJM include paths for ICall module
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/inc/
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/icall/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/hal/src/target/_common
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack/common/cc26xx/freertos
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/common/nv

    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom

    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/osal/src/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/controller/cc26xx/inc
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/dev_info
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/profiles/simple_profile
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/display
)

set(COMMON_SOURCES
    cli_uart.cpp
    main.c
    bleAppTask.c
    bleStackMenu.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/apps/common/ble_remote_display/stack/osal_icall_ble.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/rom/agama_r1/rom_init.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/util.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/menu/two_btn_menu.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/display/Display.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack_flash/common/cc26xx/board_key.c
    ${TI_SIMPLELINK_SDK_DIR}/kernel/nortos/dpl/SystemP_nortos.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble5stack/common/cc26xx/freertos/bget.c
)

if(TI_SIMPLELINK_KERNEL STREQUAL "freertos")
    list(APPEND COMMON_SOURCES freertos_main.c)

elseif(TI_SIMPLELINK_KERNEL STREQUAL "tirtos7")
    list(APPEND COMMON_SOURCES tirtos_main.c)

else()
    message(FATAL_ERROR "Unsuported TI_SIMPLELINK_KERNEL: ${TI_SIMPLELINK_KERNEL}")
endif()

#if(OT_FTD)
#    include(ftd.cmake)
#endif()

if(OT_MTD)
    include(mtd.cmake)
endif()

#if(OT_RCP)
#    include(radio.cmake)
#endif()
