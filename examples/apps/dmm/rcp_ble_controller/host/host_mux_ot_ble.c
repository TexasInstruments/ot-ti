/*
 *  Copyright (c) 2025, Texas Instruments Incorporated
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

/*!
 * @file  host_mux_ot_ble.c
 * @brief Linux host daemon for the DMM RCP + BLE Controller Combined Serial MUX.
 *
 * Adapted from the SDK's combined_serial/host/host_mux.c (Zigbee+BLE variant).
 * The only functional difference is the use of MUX_NLI_OT (0) for the Thread
 * PTY instead of MUX_NLI_ZB (2).
 *
 * Wire protocol:
 *   Device ──(single UART)──► host_mux_ot_ble
 *     NLI 0 (MUX_NLI_OT)  ↔  OT PTY  (/dev/pts/X) ↔ otbr-agent
 *     NLI 1 (MUX_NLI_BLE) ↔  BLE PTY (/dev/pts/Y) ↔ hciattach / BlueZ
 *
 * Usage:
 *   ./host_mux_ot_ble --device /dev/ttyACM0 [--baud 921600]
 *
 * On startup the slave PTY paths are printed to stdout:
 *   OT PTY:  /dev/pts/X
 *   BLE PTY: /dev/pts/Y
 *
 * Event loop: poll() on five fds:
 *   [0] serial UART        — incoming HDLC frames from the device
 *   [1] BLE PTY master     — HCI bytes from BlueZ
 *   [2] OT  PTY master     — spinel bytes from otbr-agent
 *   [3] keepalive timerfd  — periodic watchdog tick
 *   [4] signalfd           — SIGTERM / SIGINT clean shutdown
 */

#define _GNU_SOURCE   /* timerfd, signalfd, posix_openpt, ptsname_r */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>

/* SDK combined_serial headers — adjust include path as needed */
#include "ti/dmm/combined_serial/mux_common.h"
#include "ti/dmm/combined_serial/mux_buffer.h"
#include "ti/dmm/combined_serial/hdlc_spinel.h"
#include "host_mux_uart.h"

/*---------------------------------------------------------------------------
 * Build-time configuration
 *--------------------------------------------------------------------------*/

#define HOST_MUX_UART_DEFAULT_BAUD      921600U
#define HOST_MUX_KEEPALIVE_INTERVAL_SEC 5
#define HOST_MUX_KEEPALIVE_MISS_LIMIT   3

/*---------------------------------------------------------------------------
 * Poll fd indices
 *--------------------------------------------------------------------------*/

#define POLL_IDX_UART   0
#define POLL_IDX_BLE    1
#define POLL_IDX_OT     2
#define POLL_IDX_TIMER  3
#define POLL_IDX_SIG    4
#define POLL_FD_COUNT   5

#define PTY_PATH_MAX        64U
#define UART_READ_CHUNK     (MAX_RING_BUF_SIZE / 2U)

/*---------------------------------------------------------------------------
 * Module state
 *--------------------------------------------------------------------------*/

static int  gBlePtyMasterFd = -1;
static int  gOtPtyMasterFd  = -1;
static char gBlePtySlavePath[PTY_PATH_MAX];
static char gOtPtySlavePath[PTY_PATH_MAX];

static uint8_t      gRxStorage[MAX_RING_BUF_SIZE];
static MuxRingBuf_t gRxBuf;

/* Scratch buffers — static to keep stack usage predictable */
static uint8_t gUartReadBuf[UART_READ_CHUNK];
static uint8_t gFrameScratch[MAX_FRAME_SIZE];
static uint8_t gSpinelScratch[MUX_SPINEL_BUF_MAX];
static uint8_t gEncodedBuf[MAX_FRAME_SIZE];
static uint8_t gPtyReadBuf[MUX_MSG_BUF_LEN];

static int  gKeepaliveWatchdog = 0;
static bool gKeepaliveReceived = false;

/*---------------------------------------------------------------------------
 * Internal helpers
 *--------------------------------------------------------------------------*/

static int openPty(int *masterFd, char *slavePath, size_t slavePathLen)
{
    int mfd;
    int sfd;
    struct termios tty;

    mfd = posix_openpt(O_RDWR | O_NOCTTY);
    if (mfd < 0)
    {
        perror("posix_openpt");
        return -1;
    }
    if (grantpt(mfd) < 0)  { perror("grantpt");  (void)close(mfd); return -1; }
    if (unlockpt(mfd) < 0) { perror("unlockpt"); (void)close(mfd); return -1; }
    if (ptsname_r(mfd, slavePath, slavePathLen) != 0)
    {
        perror("ptsname_r");
        (void)close(mfd);
        return -1;
    }

    /* Put slave into raw mode so binary data is not mangled by line discipline. */
    sfd = open(slavePath, O_RDWR | O_NOCTTY);
    if (sfd >= 0)
    {
        if (tcgetattr(sfd, &tty) == 0)
        {
            cfmakeraw(&tty);
            (void)tcsetattr(sfd, TCSANOW, &tty);
        }
        (void)close(sfd);
    }

    *masterFd = mfd;
    return 0;
}

static void sendKeepaliveAck(void)
{
    uint16_t encodedLen = 0U;
    MuxErr_t err;

    err = MuxSpinelHdlc_encode((uint8_t)MUX_NLI_KEEPALIVE,
                                (uint32_t)CMD_KEEPALIVE_ACK,
                                NULL, 0U,
                                gEncodedBuf, (uint16_t)sizeof(gEncodedBuf),
                                &encodedLen);
    if (err != MUX_SUCCESS)
    {
        fprintf(stderr, "[host_mux_ot_ble] keepalive ACK encode failed: %d\n", err);
        return;
    }
    if (HostMuxUart_write(gEncodedBuf, encodedLen) != MUX_SUCCESS)
    {
        fprintf(stderr, "[host_mux_ot_ble] keepalive ACK write failed\n");
    }
}

static void writeToPty(int ptyFd, const uint8_t *buf, uint16_t len)
{
    const uint8_t *ptr       = buf;
    uint16_t       remaining = len;

    while (remaining > 0U)
    {
        ssize_t n = write(ptyFd, ptr, (size_t)remaining);
        if (n < 0)
        {
            if (errno == EINTR)   { continue; }
            if (errno == EIO)     { return; }  /* slave not open — drop silently */
            fprintf(stderr, "[host_mux_ot_ble] PTY write error: %s\n", strerror(errno));
            return;
        }
        ptr       += (uint16_t)n;
        remaining -= (uint16_t)n;
    }
}

static void dispatchFrame(uint8_t nli, uint32_t cmd,
                          const uint8_t *payload, uint16_t payloadLen)
{
    switch (nli)
    {
        case MUX_NLI_BLE:
            if (payloadLen > 0U)
            {
                writeToPty(gBlePtyMasterFd, payload, payloadLen);
            }
            break;

        case MUX_NLI_OT:
            /* Thread spinel frames → forward to otbr-agent via OT PTY */
            if (payloadLen > 0U)
            {
                writeToPty(gOtPtyMasterFd, payload, payloadLen);
            }
            break;

        case MUX_NLI_KEEPALIVE:
            if (cmd == (uint32_t)CMD_KEEPALIVE)
            {
                gKeepaliveReceived = true;
                sendKeepaliveAck();
            }
            else if (cmd == (uint32_t)CMD_LOGGING)
            {
                if (payload != NULL && payloadLen > 0U)
                {
                    fprintf(stderr, "[device] %.*s\n",
                            (int)payloadLen, (const char *)payload);
                }
            }
            break;

        default:
            fprintf(stderr, "[host_mux_ot_ble] unknown NLI %u cmd %u len %u — dropped\n",
                    nli, (unsigned)cmd, payloadLen);
            break;
    }
}

static void handleSerialRx(void)
{
    ssize_t  n;
    MuxErr_t err;
    uint16_t frameLen;
    uint16_t spinelLen;
    uint8_t  nli;
    uint32_t cmd;
    const uint8_t *payloadPtr;
    uint16_t       payloadLen;

    n = read(HostMuxUart_getFd(), gUartReadBuf, sizeof(gUartReadBuf));
    if (n <= 0)
    {
        if (n < 0 && errno != EINTR)
        {
            fprintf(stderr, "[host_mux_ot_ble] UART read error: %s\n", strerror(errno));
        }
        return;
    }

    err = MuxBuf_write(&gRxBuf, gUartReadBuf, (uint16_t)n);
    if (err != MUX_SUCCESS)
    {
        fprintf(stderr, "[host_mux_ot_ble] RX ring buffer overflow — resetting\n");
        MuxBuf_reset(&gRxBuf);
        return;
    }

    for (;;)
    {
        err = MuxBuf_extractFrame(&gRxBuf, gFrameScratch,
                                  (uint16_t)sizeof(gFrameScratch), &frameLen);
        if (err == MUX_ERR_NO_PACKET)  { break; }
        if (err == MUX_ERR_OVERFLOW)
        {
            fprintf(stderr, "[host_mux_ot_ble] oversized frame discarded\n");
            continue;
        }
        if (err != MUX_SUCCESS)
        {
            fprintf(stderr, "[host_mux_ot_ble] extractFrame err %d\n", err);
            break;
        }

        err = MuxHdlc_decode(gFrameScratch, frameLen,
                             gSpinelScratch, (uint16_t)sizeof(gSpinelScratch),
                             &spinelLen);
        if (err == MUX_ERR_CRC)
        {
            fprintf(stderr, "[host_mux_ot_ble] CRC error — frame dropped\n");
            continue;
        }
        if (err != MUX_SUCCESS)
        {
            fprintf(stderr, "[host_mux_ot_ble] HDLC decode err %d\n", err);
            continue;
        }

        err = MuxSpinel_parseFrame(gSpinelScratch, spinelLen,
                                   &nli, &cmd, &payloadPtr, &payloadLen);
        if (err != MUX_SUCCESS)
        {
            fprintf(stderr, "[host_mux_ot_ble] Spinel parse err %d\n", err);
            continue;
        }

        dispatchFrame(nli, cmd, payloadPtr, payloadLen);
    }
}

static void handlePtyRx(int ptyFd, uint8_t nli)
{
    ssize_t  n;
    uint16_t encodedLen;
    MuxErr_t err;

    n = read(ptyFd, gPtyReadBuf, sizeof(gPtyReadBuf));
    if (n <= 0)
    {
        if (n == 0 || errno == EIO || errno == EINTR) { return; }
        fprintf(stderr, "[host_mux_ot_ble] PTY read error (nli=%u): %s\n",
                nli, strerror(errno));
        return;
    }

    err = MuxSpinelHdlc_encode(nli,
                                (uint32_t)SPINEL_CMD_PROP_VALUE_SET,
                                gPtyReadBuf, (uint16_t)n,
                                gEncodedBuf, (uint16_t)sizeof(gEncodedBuf),
                                &encodedLen);
    if (err != MUX_SUCCESS)
    {
        fprintf(stderr, "[host_mux_ot_ble] encode err %d (nli=%u)\n", err, nli);
        return;
    }

    if (HostMuxUart_write(gEncodedBuf, encodedLen) != MUX_SUCCESS)
    {
        fprintf(stderr, "[host_mux_ot_ble] UART write err (nli=%u)\n", nli);
    }
}

/*---------------------------------------------------------------------------
 * main()
 *--------------------------------------------------------------------------*/

static void printUsage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s --device <dev> [--baud <rate>]\n"
            "\n"
            "  --device <dev>   Serial device path  (e.g. /dev/ttyACM0)\n"
            "  --baud <rate>    Baud rate in bits/s  (default: %u)\n"
            "\n"
            "On startup two PTY slave paths are printed to stdout:\n"
            "  OT PTY:  /dev/pts/X  — connect otbr-agent here\n"
            "  BLE PTY: /dev/pts/Y  — connect BlueZ hciattach here\n",
            prog, (unsigned)HOST_MUX_UART_DEFAULT_BAUD);
}

int main(int argc, char *argv[])
{
    const char *device   = NULL;
    uint32_t    baudRate = HOST_MUX_UART_DEFAULT_BAUD;
    int         timerFd  = -1;
    int         sigFd    = -1;
    int         ret      = 0;
    sigset_t    mask;
    struct itimerspec its;
    struct pollfd fds[POLL_FD_COUNT];

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--device") == 0 && i + 1 < argc)
        {
            device = argv[++i];
        }
        else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc)
        {
            baudRate = (uint32_t)strtoul(argv[++i], NULL, 10);
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            printUsage(argv[0]);
            return 0;
        }
        else
        {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            printUsage(argv[0]);
            return 1;
        }
    }

    if (device == NULL)
    {
        fprintf(stderr, "Error: --device is required\n");
        printUsage(argv[0]);
        return 1;
    }

    /* ------------------------------------------------------------------ *
     * Initialise ring buffer
     * ------------------------------------------------------------------ */
    MuxBuf_init(&gRxBuf, gRxStorage, (uint16_t)sizeof(gRxStorage));

    /* ------------------------------------------------------------------ *
     * Open PTYs
     * ------------------------------------------------------------------ */
    if (openPty(&gBlePtyMasterFd, gBlePtySlavePath, sizeof(gBlePtySlavePath)) < 0)
    {
        fprintf(stderr, "[host_mux_ot_ble] failed to open BLE PTY\n");
        return 1;
    }
    if (openPty(&gOtPtyMasterFd, gOtPtySlavePath, sizeof(gOtPtySlavePath)) < 0)
    {
        fprintf(stderr, "[host_mux_ot_ble] failed to open OT PTY\n");
        (void)close(gBlePtyMasterFd);
        return 1;
    }

    printf("OT PTY:  %s\n", gOtPtySlavePath);
    printf("BLE PTY: %s\n", gBlePtySlavePath);
    fflush(stdout);

    /* ------------------------------------------------------------------ *
     * Open serial port
     * ------------------------------------------------------------------ */
    if (HostMuxUart_open(device, baudRate) != MUX_SUCCESS)
    {
        fprintf(stderr, "[host_mux_ot_ble] failed to open %s: %s\n",
                device, strerror(errno));
        (void)close(gBlePtyMasterFd);
        (void)close(gOtPtyMasterFd);
        return 1;
    }

    /* ------------------------------------------------------------------ *
     * Keepalive timerfd
     * ------------------------------------------------------------------ */
    timerFd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (timerFd < 0) { perror("timerfd_create"); ret = 1; goto cleanup; }

    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec    = HOST_MUX_KEEPALIVE_INTERVAL_SEC;
    its.it_interval.tv_sec = HOST_MUX_KEEPALIVE_INTERVAL_SEC;
    if (timerfd_settime(timerFd, 0, &its, NULL) < 0)
    {
        perror("timerfd_settime");
        ret = 1;
        goto cleanup;
    }

    /* ------------------------------------------------------------------ *
     * signalfd for SIGTERM / SIGINT
     * ------------------------------------------------------------------ */
    sigemptyset(&mask);
    sigaddset(&mask, SIGTERM);
    sigaddset(&mask, SIGINT);
    if (sigprocmask(SIG_BLOCK, &mask, NULL) < 0)
    {
        perror("sigprocmask");
        ret = 1;
        goto cleanup;
    }
    sigFd = signalfd(-1, &mask, SFD_CLOEXEC);
    if (sigFd < 0) { perror("signalfd"); ret = 1; goto cleanup; }

    /* ------------------------------------------------------------------ *
     * Build pollfd array
     * ------------------------------------------------------------------ */
    memset(fds, 0, sizeof(fds));
    fds[POLL_IDX_UART].fd     = HostMuxUart_getFd();
    fds[POLL_IDX_UART].events = POLLIN;
    fds[POLL_IDX_BLE].fd      = gBlePtyMasterFd;
    fds[POLL_IDX_BLE].events  = POLLIN;
    fds[POLL_IDX_OT].fd       = gOtPtyMasterFd;
    fds[POLL_IDX_OT].events   = POLLIN;
    fds[POLL_IDX_TIMER].fd    = timerFd;
    fds[POLL_IDX_TIMER].events = POLLIN;
    fds[POLL_IDX_SIG].fd      = sigFd;
    fds[POLL_IDX_SIG].events  = POLLIN;

    fprintf(stderr, "[host_mux_ot_ble] started — device %s  OT=%s  BLE=%s\n",
            device, gOtPtySlavePath, gBlePtySlavePath);

    /* ------------------------------------------------------------------ *
     * Main event loop
     * ------------------------------------------------------------------ */
    for (;;)
    {
        int nready = poll(fds, (nfds_t)POLL_FD_COUNT, -1);
        if (nready < 0)
        {
            if (errno == EINTR) { continue; }
            perror("poll");
            ret = 1;
            break;
        }

        if (fds[POLL_IDX_UART].revents & POLLIN)          { handleSerialRx(); }
        if (fds[POLL_IDX_UART].revents & (POLLERR | POLLHUP))
        {
            fprintf(stderr, "[host_mux_ot_ble] UART fd error — exiting\n");
            ret = 1;
            break;
        }

        if (fds[POLL_IDX_BLE].revents & POLLIN)
        {
            handlePtyRx(gBlePtyMasterFd, (uint8_t)MUX_NLI_BLE);
        }
        if (fds[POLL_IDX_OT].revents & POLLIN)
        {
            handlePtyRx(gOtPtyMasterFd, (uint8_t)MUX_NLI_OT);
        }

        if (fds[POLL_IDX_TIMER].revents & POLLIN)
        {
            uint64_t exp = 0U;
            (void)read(timerFd, &exp, sizeof(exp));

            if (gKeepaliveReceived)
            {
                gKeepaliveReceived = false;
                gKeepaliveWatchdog = 0;
            }
            else
            {
                gKeepaliveWatchdog++;
                fprintf(stderr, "[host_mux_ot_ble] keepalive miss %d/%d\n",
                        gKeepaliveWatchdog, HOST_MUX_KEEPALIVE_MISS_LIMIT);
                if (gKeepaliveWatchdog >= HOST_MUX_KEEPALIVE_MISS_LIMIT)
                {
                    fprintf(stderr, "[host_mux_ot_ble] device silent — watchdog expired\n");
                    ret = 2;
                    break;
                }
            }
        }

        if (fds[POLL_IDX_SIG].revents & POLLIN)
        {
            struct signalfd_siginfo sinfo;
            (void)read(sigFd, &sinfo, sizeof(sinfo));
            fprintf(stderr, "[host_mux_ot_ble] signal %u — shutting down\n",
                    sinfo.ssi_signo);
            ret = 0;
            break;
        }
    }

cleanup:
    if (sigFd >= 0)   { (void)close(sigFd); }
    if (timerFd >= 0) { (void)close(timerFd); }
    if (HostMuxUart_getFd() >= 0) { HostMuxUart_close(); }
    if (gBlePtyMasterFd >= 0) { (void)close(gBlePtyMasterFd); gBlePtyMasterFd = -1; }
    if (gOtPtyMasterFd  >= 0) { (void)close(gOtPtyMasterFd);  gOtPtyMasterFd  = -1; }

    return ret;
}
