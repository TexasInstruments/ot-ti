/*
 *  Copyright (c) 2016, Texas Instruments Incorporated
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

#include <stdbool.h>
#include <stdint.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/UART2.h>

#include <ti/drivers/AESECB.h>
#include <ti/drivers/ECDH.h>

#include <ti/drivers/ECDSA.h>
#include <ti/drivers/ECJPAKE.h>
#include <ti/drivers/SHA2.h>

/* POSIX Header files */
#include <mqueue.h>
#include <pthread.h>
#include <sched.h>

// The entry point for the application
extern int app_main(int argc, char *argv[]);
#define APP_STACK_SIZE (2048)

uint32_t appStack[APP_STACK_SIZE];

void *taskCode(void *arg0)
{
    (void)arg0;
    app_main(0, NULL);

    return NULL;
}

int main(void)
{
    Board_init();

    GPIO_init();

    NVS_init();

    ECDH_init();

    ECDSA_init();

    ECJPAKE_init();

    AESECB_init();

    SHA2_init();

    pthread_t           thread;
    pthread_attr_t      pAttrs;
    struct sched_param  priParam;
    int                 retc;

    retc = pthread_attr_init(&pAttrs);
    while(retc != 0);

    retc = pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    while(retc != 0);

    priParam.sched_priority = 1;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    while(retc != 0);

    retc = pthread_attr_setstack(&pAttrs, (void *)appStack,
                                 sizeof(appStack));
    while(retc != 0);

    retc = pthread_create(&thread, &pAttrs, taskCode, NULL);
    while(retc != 0);

    retc = pthread_attr_destroy(&pAttrs);
    while(retc != 0);

    BIOS_start();

    // Should never get here.
    while (1)
        ;
}
