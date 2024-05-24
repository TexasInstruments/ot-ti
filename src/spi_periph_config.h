
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
 *   This file includes default SPI configuration definitions
 */
#ifndef _SPI_PERIPH_CONFIG_H_
#define _SPI_PERIPH_CONFIG_H_


#define PLATFORM_SPI_FREQ               1000000
#define PLATFORM_SPI_FRAME_FORMAT       SPI_POL0_PHA1
#define PLATFORM_SPI_DATA_SIZE          8
#define PLATFORM_SPI_MODE               SPI_PERIPHERAL
#define PLATFORM_SPI_TRANSFER_MODE      SPI_MODE_CALLBACK
#define PLATFORM_SPI_MAX_TRANSACTIONS   3
#define PLATFORM_SPI_FIRST_TRANSACTION  1
#define PLATFORM_SPI_MID_TRANSACTION    2
#define PLATFORM_SPI_LAST_TRANSACTION   3
#define PLATFORM_SPI_GET_CRC(X)         (X[0] & 0x40)
#define PLATFORM_SPI_SET_CRC(X)         (X[0] |= 0x40)
#define PLATFORM_SPI_UNSET_CRC(X)       (X[0] &= ~0x40)
#define PLATFORM_SPI_GET_CCF(X)         (X[0] & 0x20)
#define PLATFORM_SPI_SET_CCF(X)         (X[0] = X[0] | 0x20)
#define PLATFORM_SPI_GET_LEN(X)         (X[3] | (X[4] << 8))

#define SPINEL_INT_ASSERT 0
#define SPINEL_INT_DEASSERT 1

#define SPINEL_HEADER_LENGTH 5

#endif // _SPI_PERIPH_CONFIG_H_
