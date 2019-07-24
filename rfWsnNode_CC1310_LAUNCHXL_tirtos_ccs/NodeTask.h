/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TASKS_NODETASK_H_
#define TASKS_NODETASK_H_

#include <ti/drivers/rf/RF.h>

#define  NODE_NOTIFICATION_TYPE_MOTION_DETECTED   0x81

typedef struct
{
    uint16_t    shortAddress;
    int16_t     power;
    uint32_t    frequency;
    uint32_t    maxPayloadLength;
    uint32_t    timeout;
}   NODETASK_CONFIG;

typedef struct
{
    uint32_t    frequency;
    int8_t      power;
    int8_t      rssi;
}   NODETASK_STATUS;

/* Initializes the Node Task and creates all TI-RTOS objects */
void NodeTask_init(void);

void NodeTask_dataOn(void);
bool NodeTask_dataTransfer(uint8_t* buffer, uint32_t length);
bool NodeTask_postTransfer(uint8_t* buffer, uint32_t length);

void NodeTask_testTransferStart(void);
void NodeTask_testTransferStop(void);

bool NodeTask_motionDetectionStart(void);
bool NodeTask_motionDetectionStop(void);

void NodeTask_scanStart(void);
void NodeTask_scanStop(void);

void    NodeTask_transferStart(void);
void    NodeTask_transferStop(void);
void    NodeTask_motionStart(void);
void    NodeTask_motionStop(void);

void NodeTask_wakeup(void);

void    NodeTask_getConfig(NODETASK_CONFIG* config);
bool    NodeTask_setConfig(NODETASK_CONFIG* config);

void    NodeTask_getRFStatus(NODETASK_STATUS* status);

uint32_t NodeTask_getQueueSize(void);

#endif /* TASKS_NODETASK_H_ */
