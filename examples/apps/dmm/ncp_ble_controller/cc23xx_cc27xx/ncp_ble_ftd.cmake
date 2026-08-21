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

add_executable(ot-ncp-ble-controller-ftd
    ${COMMON_SOURCES}
)

target_include_directories(ot-ncp-ble-controller-ftd PUBLIC ${COMMON_INCLUDES})

if(NOT DEFINED OT_PLATFORM_LIB_FTD)
    set(OT_PLATFORM_LIB_FTD ${OT_PLATFORM_LIB})
endif()

target_link_libraries(ot-ncp-ble-controller-ftd PUBLIC
    openthread-ncp-ftd
    ${OT_PLATFORM_LIB_FTD}
    openthread-ftd
    ${OT_PLATFORM_LIB_FTD}
    openthread-ncp-ftd
    ${OT_MBEDTLS}
    ot-config-ftd
    ot-config
    ble-stack-lib
)

# ControllerLib.a (a prebuilt archive in the SDK) references BleSysStat_* symbols provided
# by ble-stack-lib, creating a circular dependency. Use --start-group/--end-group so the
# linker iterates until all cross-references are resolved.
target_link_libraries(ot-ncp-ble-controller-ftd PRIVATE
    -Wl,--start-group
    $<TARGET_FILE:ble-stack-lib>
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/lib/CC27XXX10/ControllerLib/lib/ticlang/m33f/ControllerLib.a
    -Wl,--end-group
)

install(TARGETS ot-ncp-ble-controller-ftd
    DESTINATION bin
)
set_target_properties(ot-ncp-ble-controller-ftd
    PROPERTIES
        SUFFIX .out
)
add_dependencies(ot-ncp-ble-controller-ftd ble-stack-lib)
