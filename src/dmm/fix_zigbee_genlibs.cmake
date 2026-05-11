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
# SysConfig (run with --compiler gcc) lists the Zigbee libraries under lib/gcc/,
# but the SDK ships the Zigbee archives only under lib/ticlang/ (ELF objects built
# with TI Clang, linkable by GNU ld just like the BLE ControllerLib.a).
# Rewrite those entries in the generated ti_utils_build_linker.cmd.genlibs.
#
# usage: cmake -DGENLIBS=<path to genlibs file> -P fix_zigbee_genlibs.cmake
#
if(NOT GENLIBS OR NOT EXISTS "${GENLIBS}")
    message(FATAL_ERROR "fix_zigbee_genlibs.cmake: GENLIBS file not found: '${GENLIBS}'")
endif()

file(READ "${GENLIBS}" content)
string(REGEX REPLACE "(third_party/zigbee/libraries/[^\"]*)/lib/gcc/" "\\1/lib/ticlang/" content "${content}")
file(WRITE "${GENLIBS}" "${content}")
