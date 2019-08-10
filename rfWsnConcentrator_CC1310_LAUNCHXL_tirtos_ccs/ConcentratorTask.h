/******************************************************************************

 @file ConcentratorTask.h

 @brief Easylink Concentrator Example Application Header

 Group: CMCU LPRF
 Target Device: cc13x0

 ******************************************************************************
 
 Copyright (c) 2015-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#ifndef TASKS_CONCENTRATORTASK_H_
#define TASKS_CONCENTRATORTASK_H_

/* Create the ConcentratorRadioTask and creates all TI-RTOS objects */
void ConcentratorTask_init(void);

bool ConcentratorTask_sendCommand(uint8_t _device_id, uint8_t cmd, uint8_t *_params, uint32_t _length);

bool ConcentratorTask_commandConfig(int argc, char *argv[]);
bool ConcentratorTask_commandStatus(int argc, char *argv[]);
bool ConcentratorTask_commandStart(int argc, char *argv[]);
bool ConcentratorTask_commandDownlink(int argc, char *argv[]);
bool ConcentratorTask_commandContract(int argc, char *argv[]);
bool ConcentratorTask_commandScan(int argc, char *argv[]);
bool ConcentratorTask_commandSleep(int argc, char *argv[]);
bool ConcentratorTask_commandDetect(int argc, char *argv[]);
bool ConcentratorTask_commandReset(int argc, char *argv[]);

#endif /* TASKS_CONCENTRATORTASK_H_ */
