/*
 *  Copyright (c) 2024, Texas Instruments Incorporated
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file includes Sample Tx power table and region definitions
 */
#ifndef _REGION_SETTINGS_H_
#define _REGION_SETTINGS_H_
#define CC_UINT16(b0, b1) ((b0 << 8) | b1)

// TX power disabled
#define DX -128
#define REGION_IS_CH_DISABLED(val) ((val) == (DX))

/**
 * @brief Region enum
 */
typedef enum {
    /** United States */
    OT_HAL_REGION_UNITED_STATES = 0,
    /** Europe */
    OT_HAL_REGION_EUROPE = 1,
    /** Japan */
    OT_HAL_REGION_JAPAN = 2,
    /** India */
    OT_HAL_REGION_INDIA = 3,
    /** World wide */
    OT_HAL_REGION_WORLD_WIDE = 4, // Generic Tx Power set to default of 5 dbm
    /** Unified */
    OT_HAL_REGION_UNIFIED = 5,
    /** Canada */
    OT_HAL_REGION_CANADA = 6,
    /** Australia and New Zealand */
    OT_HAL_REGION_AU_NZ = 7,
    /** Brazil */
    OT_HAL_REGION_BRAZIL = 8,
    /** Mexico */
    OT_HAL_REGION_MEXICO = 9,
    /** Number of defined Regions */
    OT_HAL_REGION_MAX = 10,
} otPlat_radioRegion_t;

/** @brief Table of Tx power setting in dBm per channel and region. Power will be used as Tx power on selected channel.
           Amount of regions to match otPlat_powerTableRegionMax. */
const int8_t otPlat_powerTable[OT_HAL_REGION_MAX][16] = {
// 11  12  13   5   5  16  17  18  19  20  21  22  23  24  25  26
  {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, DX}, // US
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5}, // Europe
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5}, // Japan
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5}, // India
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5, 5}, // WW
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5, DX}, // Unified
  {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, DX}, // Canada
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5}, // AU/NZ
  { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5}, // Brazil
  {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, DX}, // Mexico
};
#endif // _REGION_SETTINGS_H_
