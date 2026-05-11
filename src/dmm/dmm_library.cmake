#
#  Copyright (c) 2026, Texas Instruments Incorporated
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

#
# Builds the DMM scheduler and policy manager from the SDK sources. Included only with
# -DOT_DMM_BUILD_FROM_SOURCE=ON, together with dmm.genLibs = false in the application SysConfig file
# so that the prebuilt dmm_library.a is not linked as well.
#

set(DMM_SDK_DIR ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm)

if(NOT EXISTS ${DMM_SDK_DIR}/internal/dmm_scheduler.c)
    message(FATAL_ERROR "DMM sources not found in ${DMM_SDK_DIR}/internal")
endif()

add_library(dmm-library STATIC
    ${DMM_SDK_DIR}/internal/dmm_scheduler.c
    ${DMM_SDK_DIR}/internal/dmm_policy.c
    ${DMM_SDK_DIR}/internal/RCL_AdcNoise_dmm.c
    ${DMM_SDK_DIR}/dual_pan_dmm_stub.c
)

target_include_directories(dmm-library
    PUBLIC
        ${DMM_SDK_DIR}
    PRIVATE
        ${DMM_SDK_DIR}/internal
        ${TI_SIMPLELINK_SDK_DIR}/source
        ${TI_SIMPLELINK_SDK_DIR}/source/ti
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/drivers
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/drivers/dpl
        ${TI_SIMPLELINK_SDK_DIR}/source/ti/drivers/rcl
)

# The DeviceFamily_* define comes from the SysConfig generated options file.
target_compile_options(dmm-library PRIVATE @${CMAKE_CURRENT_BINARY_DIR}/syscfg/ti_utils_build_compiler.opt -Wall)
add_dependencies(dmm-library build_syscfg)
