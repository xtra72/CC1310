/******************************************************************************

 @file ConcentratorTask.c

 @brief Easylink Concentrator Example Application

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

/***** Includes *****/
/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

#include <strings.h>
/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "ConcentratorRadioTask.h"
#include "ConcentratorTask.h"
#include "ShellTask.h"
#include "RadioProtocol.h"

#include "DataQueue.h"
#include "Trace.h"
#include "rf.h"

/***** Defines *****/
#define CONCENTRATOR_TASK_STACK_SIZE 1024
#define CONCENTRATOR_TASK_PRIORITY   3

#define CONCENTRATOR_EVENT_ALL                      0xFFFFFFFF
#define CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE     (uint32_t)(1 << 0)
#define CONCENTRATOR_EVENT_NEW_RAW_DATA             (uint32_t)(1 << 1)
#define CONCENTRATOR_EVENT_TEST_RESET               (uint32_t)(1 << 2)
#define CONCENTRATOR_EVENT_ERROR                    (uint32_t)(1 << 3)

#define CONCENTRATOR_MAX_NODES 7

#define CONCENTRATOR_DISPLAY_LINES 8

/***** Type declarations *****/
struct AdcSensorNode {
    uint8_t address;
    uint16_t latestAdcValue;
    uint8_t button;
    int8_t latestRssi;
};


struct RawDataNode {
    uint32_t    time;
    uint8_t     address;
    uint8_t     length;
    uint8_t     data[128];
    int8_t      latestRssi;
};


/***** Variable declarations *****/
static Task_Params concentratorTaskParams;
Task_Struct concentratorTask;    /* not static so you can see in ROV */
static uint8_t concentratorTaskStack[CONCENTRATOR_TASK_STACK_SIZE];
Event_Struct concentratorEvent;  /* not static so you can see in ROV */
static Event_Handle concentratorEventHandle;
static struct AdcSensorNode latestActiveAdcSensorNode;
static struct RawDataNode   latestRawDataNode;
struct AdcSensorNode knownSensorNodes[CONCENTRATOR_MAX_NODES];
static struct AdcSensorNode* lastAddedSensorNode = knownSensorNodes;
static  char buffer[256] = {0,};

static DataQ        dataQ_;
static  uint32_t    previousReceivedTime = 0;

static  uint32_t    receivedPacket = 0;
static  uint32_t    receivedDataSize = 0;
static  uint32_t    successfullyReceivedPacket = 0;

static  uint32_t    totalReceivedPacket = 0;
static  uint32_t    totalReceivedDataSize = 0;
static  uint32_t    totalSuccessfullyReceivedPacket = 0;

/* Pin driver handle */

/* Clock for sensor stub */


/***** Prototypes *****/
static void ConcentratorTask_main(UArg arg0, UArg arg1);
static void ConcentratorTask_packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi);
static void ConcentratorTask_addNewNode(struct AdcSensorNode* node);
static void ConcentratorTask_updateNode(struct AdcSensorNode* node);
static uint8_t ConcentratorTask_isKnownNodeAddress(uint8_t address);


/***** Function definitions *****/
void ConcentratorTask_init(void)
{

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&concentratorEvent, &eventParam);
    concentratorEventHandle = Event_handle(&concentratorEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorTaskParams);
    concentratorTaskParams.stackSize = CONCENTRATOR_TASK_STACK_SIZE;
    concentratorTaskParams.priority = CONCENTRATOR_TASK_PRIORITY;
    concentratorTaskParams.stack = &concentratorTaskStack;
    Task_construct(&concentratorTask, ConcentratorTask_main, &concentratorTaskParams, NULL);
}

static void ConcentratorTask_main(UArg arg0, UArg arg1)
{
    ShellTask_output("+BOOT\n");

    DataQ_init(&dataQ_, 4);

    /* Register a packet received callback with the radio task */
    ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorTask_packetReceivedCallback);

    /* Enter main task loop */
    while(1)
    {
        uint32_t    currentReceivedTime;

        /* Wait for event */
        uint32_t events = Event_pend(concentratorEventHandle, 0, CONCENTRATOR_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we got a new ADC sensor value */
        if(events & CONCENTRATOR_EVENT_NEW_RAW_DATA)
        {
            uint32_t    i;

            /* If we knew this node from before, update the value */
            if(ConcentratorTask_isKnownNodeAddress(latestActiveAdcSensorNode.address))
            {
                ConcentratorTask_updateNode(&latestActiveAdcSensorNode);
            }
            else
            {
                /* Else add it */
                ConcentratorTask_addNewNode(&latestActiveAdcSensorNode);
            }

            receivedPacket++;
            receivedDataSize += latestRawDataNode.length;
            successfullyReceivedPacket++;
            totalReceivedPacket++;
            totalReceivedDataSize += latestRawDataNode.length;
            totalSuccessfullyReceivedPacket++;

            for(i = 0 ; i < latestRawDataNode.length ; i++)
            {
                uint8_t hi = (latestRawDataNode.data[i] >> 4) & 0x0F;
                uint8_t lo = (latestRawDataNode.data[i]     ) & 0x0F;
                if (hi < 10)
                {
                    buffer[i*2] ='0' + hi;
                }
                else
                {
                    buffer[i*2] ='A' + hi - 10;
                }

                if (lo < 10)
                {
                    buffer[i*2 + 1] ='0' + lo;
                }
                else
                {
                    buffer[i*2 + 1] ='A' + lo - 10;
                }
            }

            buffer[i*2]= 0;

            ShellTask_output("+DATA: %4d.%03d, %d, %s\n", latestRawDataNode.time / 1000, latestRawDataNode.time % 1000, latestRawDataNode.length, buffer);

        }
        else if(events & CONCENTRATOR_EVENT_TEST_RESET)
        {
            /* If we knew this node from before, update the value */
            if(ConcentratorTask_isKnownNodeAddress(latestActiveAdcSensorNode.address))
            {
                ConcentratorTask_updateNode(&latestActiveAdcSensorNode);
            }
            else
            {
                /* Else add it */
                ConcentratorTask_addNewNode(&latestActiveAdcSensorNode);
            }

            receivedPacket = 0;
            receivedDataSize = 0;
            successfullyReceivedPacket = 0;
            totalReceivedPacket = 0;
            totalReceivedDataSize = 0;
            totalSuccessfullyReceivedPacket = 0;
        }
        else if(events & CONCENTRATOR_EVENT_ERROR)
        {
            receivedPacket++;
            totalReceivedPacket++;
        }

        currentReceivedTime = (Clock_getTicks() * Clock_tickPeriod) / 1000000;

        if (currentReceivedTime != previousReceivedTime)
        {
            //clear screen, put cuser to beggining of terminal and print the header
            Trace_printf("%8d %8d %8d %3d %8d %8d %8d %3d\n",
                           receivedDataSize, successfullyReceivedPacket, receivedPacket, successfullyReceivedPacket * 100 / receivedPacket,
                           totalReceivedDataSize, totalSuccessfullyReceivedPacket, totalReceivedPacket, totalSuccessfullyReceivedPacket * 100 / totalReceivedPacket);
            receivedPacket =  0;
            receivedDataSize = 0;
            successfullyReceivedPacket = 0;

            previousReceivedTime = currentReceivedTime;
        }
    }
}

static void ConcentratorTask_packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi)
{
    /* If we recived an ADC sensor packet, for backward compatibility */
    if(packet->header.packetType == RADIO_PACKET_TYPE_RAW_DATA_PACKET)
    {
        /* Save the values */
        latestRawDataNode.time = (Clock_getTicks() * Clock_tickPeriod) / 1000;
        latestRawDataNode.address = packet->header.sourceAddress;
        latestRawDataNode.length = packet->header.length;
        memcpy(latestRawDataNode.data, packet->rawDataPacket.data, latestRawDataNode.length);
        latestRawDataNode.latestRssi = rssi;

        DataQ_push(&dataQ_, packet->rawDataPacket.data, packet->header.length);

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_RAW_DATA);
    }
    else if(packet->header.packetType == RADIO_PACKET_TYPE_TEST_RESET)
    {
        /* Save the values */
        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_TEST_RESET);
    }
    else
    {
        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_ERROR);
    }
}

static uint8_t ConcentratorTask_isKnownNodeAddress(uint8_t address)
{
    uint8_t found = 0;
    uint8_t i;
    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++)
    {
        if (knownSensorNodes[i].address == address)
        {
            found = 1;
            break;
        }
    }
    return found;
}

static void ConcentratorTask_updateNode(struct AdcSensorNode* node)
{
    uint8_t i;
    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++) {
        if (knownSensorNodes[i].address == node->address)
        {
            knownSensorNodes[i].latestAdcValue = node->latestAdcValue;
            knownSensorNodes[i].latestRssi = node->latestRssi;
            knownSensorNodes[i].button = node->button;
            break;
        }
    }
}

static void ConcentratorTask_addNewNode(struct AdcSensorNode* node)
{
    *lastAddedSensorNode = *node;

    /* Increment and wrap */
    lastAddedSensorNode++;
    if (lastAddedSensorNode > &knownSensorNodes[CONCENTRATOR_MAX_NODES-1])
    {
        lastAddedSensorNode = knownSensorNodes;
    }
}

bool ConcentratorTask_sendCommand(uint8_t _device_id, uint8_t cmd, uint8_t* _params, uint32_t _length)
{
    return  ConcentratorRadioTask_postCommand(_device_id, cmd, _params, _length);
}


bool    ConcentratorTask_commandConfig(int argc, char *argv[])
{
    if (argc == 1)
    {
        int8_t  power;

        EasyLink_getRfPower(&power);
        ShellTask_output("+%s:OK,FREQ=%d,POW=%d", argv[0],  EasyLink_getFrequency(), power);
    }
    else
    {
        int i;
        for(i = 1 ; i < argc ; i++)
        {
            char* name = strtok(argv[i], "=");
            if (strcasecmp(name, "FREQ") == 0)
            {
                char* value = strtok(NULL, " ");
                if (value != 0)
                {
                    uint32_t    frequency = strtoul(value, NULL, 10);
                    if ((RADIO_FREQUENCY_MIN <= frequency) && (frequency <= RADIO_FREQUENCY_MAX))
                    {
                        if (EasyLink_setFrequency(frequency) == EasyLink_Status_Success)
                        {
                            ShellTask_output("+%s:OK", argv[0]);
                        }
                        else
                        {
                            ShellTask_output("+%s:ERR", argv[0]);
                        }
                    }
                    else
                    {
                        ShellTask_output("+%s:ERR", argv[0]);
                    }
                }
            }
            else if (strcasecmp(name, "POW") == 0)
            {
                char* value = strtok(NULL, " ");
                if (value != 0)
                {
                    int32_t    power = strtol(value, NULL, 10);
                    if ((RADIO_POWER_MIN <= power) && (power <= RADIO_POWER_MAX))
                    {
                        if (EasyLink_setRfPower(power) == EasyLink_Status_Success)
                        {
                            ShellTask_output("+%s:OK", argv[0]);
                        }
                        else
                        {
                            ShellTask_output("+%s:ERR", argv[0]);
                        }
                    }
                    else
                    {
                        ShellTask_output("+%s:ERR", argv[0]);
                    }
                }
            }
            else
            {
                ShellTask_output("+%s:ERR", argv[0]);
                break;
            }
        }

    }

    return true;
}

bool    ConcentratorTask_commandStatus(int argc, char *argv[])
{
    if (argc == 1)
    {
        ShellTask_output("+%s:OK", argv[0]);
    }
    else
    {
        ShellTask_output("+%s:RSSI=%d", argv[0], ConcentratorRadioTask_getRssi());
    }

    return true;
}


bool    ConcentratorTask_commandStart(int argc, char *argv[])
{
    ConcentratorRadioTask_init(NULL);

    return  true;
}


bool    ConcentratorTask_commandDownlink(int argc, char *argv[])
{
    if (argc != 3)
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    uint8_t device_id = (uint8_t)strtoul(argv[1], NULL, 10);
    if (device_id == 0)
    {
        device_id = 2;
    }

    uint8_t length = 0;
    uint8_t i;
    for(i = 0 ; i < strlen(argv[2]) / 2 ; i++)
    {
        uint8_t value = 0;

        if ('0' <= argv[2][i*2] && argv[2][i*2] <= '9')
        {
            value = (argv[2][i*2] - '0');
        }
        else if ('a' <= argv[2][i*2] && argv[2][i*2] <= 'f')
        {
            value = (argv[2][i*2] - 'a' + 10);
        }
        else if ('A' <= argv[2][i*2] && argv[2][i*2] <= 'F')
        {
            value = (argv[2][i*2] - 'A' + 10);
        }

        if ('0' <= argv[2][i*2 + 1] && argv[2][i*2 + 1] <= '9')
        {
            value = (value << 4) + (argv[2][i*2 + 1] - '0');
        }
        else if ('a' <= argv[2][i*2] && argv[2][i*2 + 1] <= 'f')
        {
            value = (value << 4) + (argv[2][i*2 + 1] - 'a' + 10);
        }
        else if ('A' <= argv[2][i*2 + 1] && argv[2][i*2 + 1] <= 'F')
        {
            value = (value << 4) + (argv[2][i*2 + 1] - 'A' + 10);
        }

        buffer[length++] = value;
    }

   if (!ConcentratorTask_sendCommand(device_id, RF_DOWNLINK, (uint8_t *)buffer, length))
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    return true;
}

bool    ConcentratorTask_commandContract(int argc, char *argv[])
{
    if (argc != 3)
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    uint8_t device_id = (uint8_t)strtoul(argv[1], NULL, 10);
    if (device_id == 0)
    {
        device_id = 2;
    }

    uint32_t ts = (uint32_t)strtoul(argv[2], NULL, 10);
    uint8_t params[4];
    params[0] = (ts >> 24) & 0xFF;
    params[1] = (ts >> 16) & 0xFF;
    params[2] = (ts >>  8) & 0xFF;
    params[3] = (ts      ) & 0xFF;
   if (!ConcentratorTask_sendCommand(device_id, RF_REP_CONTRACT, params, sizeof(params)))
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    return true;
}

bool    ConcentratorTask_commandScan(int argc, char *argv[])
{
    if (argc != 3)
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    uint8_t device_id = (uint8_t)strtoul(argv[1], NULL, 10);
    if (device_id == 0)
    {
        device_id = 2;
    }

    if (strcasecmp(argv[2], "ON") == 0)
    {
        if (!ConcentratorTask_sendCommand(device_id, RF_REQ_SRV_SCAN_START, NULL, 0))
        {
            ShellTask_output("+%s:ERR", argv[0]);
            return  false;
        }
    }
    else
    {
        if (!ConcentratorTask_sendCommand(device_id, RF_REQ_SRV_SCAN_STOP, NULL, 0))
        {
            ShellTask_output("+%s:ERR", argv[0]);
            return  false;
        }
    }

    ShellTask_output("+%s:OK", argv[0]);
    return  true;
}

bool    ConcentratorTask_commandSleep(int argc, char *argv[])
{
    if (argc != 3)
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    uint16_t device_id = (uint16_t)strtoul(argv[1], NULL, 10);
    uint32_t sleep_time = (uint32_t)strtoul(argv[2], NULL, 10);
    uint8_t params[4];
    if (device_id == 0)
    {
        device_id = 2;
    }

    params[0] = (sleep_time >> 24) & 0xFF;
    params[1] = (sleep_time >> 16) & 0xFF;
    params[2] = (sleep_time >>  8) & 0xFF;
    params[3] = (sleep_time      ) & 0xFF;

    if (!ConcentratorTask_sendCommand(device_id, RF_REQ_SRV_SLEEP, params, 4))
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    ShellTask_output("+%s:OK", argv[0]);
    return  true;
}


bool    ConcentratorTask_commandDetect(int argc, char *argv[])
{
    if (argc != 3)
    {
        ShellTask_output("+%s:ERR", argv[0]);
        return  false;
    }

    uint8_t device_id = (uint8_t)strtoul(argv[1], NULL, 10);
    if (device_id == 0)
    {
        device_id = 2;
    }

    if (strcasecmp(argv[2], "ON") == 0)
    {
        if (!ConcentratorTask_sendCommand(device_id, RF_REQ_SRV_MOTION_DETECTION_START, NULL, 0))
        {
            ShellTask_output("+%s:ERR", argv[0]);
            return  false;
        }
    }
    else
    {
        if (!ConcentratorTask_sendCommand(device_id, RF_REQ_SRV_MOTION_DETECTION_STOP, NULL, 0))
        {
            ShellTask_output("+%s:ERR", argv[0]);
            return  false;
        }
    }

    ShellTask_output("+%s:OK", argv[0]);
    return  true;
}
