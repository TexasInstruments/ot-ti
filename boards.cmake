#
#  Copyright (c) 2020, The OpenThread Authors.
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

set(TI_BOARD_VALUES
    "CC1352P1_LAUNCHXL"
    "CC1352P_2_LAUNCHXL"
    "CC1352P_4_LAUNCHXL"
    "CC1352R1_LAUNCHXL"
    "CC26X2R1_LAUNCHXL"
    "LP_CC1352P7_1"
    "LP_CC1352P7_4"
    "LP_CC2652PSIP"
    "LP_CC2652R7"
    "LP_CC2652RB"
    "LP_CC2652RSIP"
    "LP_CC2653P10"
    "CC2674P10RGZ"
    "CC2674P10RSK"
    "CC2674R10RGZ"
    "CC2674R10RSK"
)

set_property(CACHE TI_SIMPLELINK_BOARD PROPERTY STRINGS ${TI_BOARD_VALUES})

if(TI_SIMPLELINK_BOARD STREQUAL "CC1352P1_LAUNCHXL")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC1352P_2_LAUNCHXL")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC1352P_4_LAUNCHXL")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC1352R1_LAUNCHXL")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC26X2R1_LAUNCHXL")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC1352P7_1")
    set(TI_SIMPLELINK_DEVICE    "cc13x2x7_cc26x2x7")
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC1352P7_4")
    set(TI_SIMPLELINK_DEVICE    "cc13x2x7_cc26x2x7")
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2652PSIP")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2652R7")
    set(TI_SIMPLELINK_DEVICE    "cc13x2x7_cc26x2x7")
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2652RB")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2652RSIP")
    set(TI_SIMPLELINK_DEVICE    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x2_cc26x2"    )
    set(TI_SIMPLELINK_ISA       "m4f"              )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_EM_CC1354P10_1")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_EM_CC1354P10_6")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )

elseif(TI_SIMPLELINK_BOARD STREQUAL "LP_CC2653P10")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC2674P10RGZ")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC2674P10RSK")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC2674R10RGZ")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )

elseif(TI_SIMPLELINK_BOARD STREQUAL "CC2674R10RSK")
    set(TI_SIMPLELINK_DEVICE    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_FAMILY    "cc13x4_cc26x4"    )
    set(TI_SIMPLELINK_ISA       "m33f"             )
else()
    if(TI_PLATFORM STREQUAL "cc13xx_cc26xx"
            AND NOT TI_SIMPLELINK_BOARD IN_LIST TI_BOARD_VALUES)
        message(FATAL_ERROR "Please select a supported board: ${TI_BOARD_VALUES}")
    endif()
endif()

