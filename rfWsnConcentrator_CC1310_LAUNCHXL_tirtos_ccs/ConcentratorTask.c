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

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "ConcentratorRadioTask.h"
#include "ConcentratorTask.h"
#include "RadioProtocol.h"

#include "DataQueue.h"
#include "Trace.h"

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

#define CONCENTRATOR_LED_BLINK_ON_DURATION_MS       100
#define CONCENTRATOR_LED_BLINK_OFF_DURATION_MS      400
#define CONCENTRATOR_LED_BLINK_TIMES                5

#define CONCENTRATOR_IDENTIFY_LED Board_PIN_LED1

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
static Display_Handle hDisplaySerial;

static  uint32_t    previousReceivedTime = 0;

static  uint32_t    receivedPacket = 0;
static  uint32_t    receivedDataSize = 0;
static  uint32_t    successfullyReceivedPacket = 0;

static  uint32_t    totalReceivedPacket = 0;
static  uint32_t    totalReceivedDataSize = 0;
static  uint32_t    totalSuccessfullyReceivedPacket = 0;

/* Pin driver handle */
static PIN_Handle identifyLedPinHandle;
static PIN_State identifyLedPinState;

/* Configure LED Pin */
PIN_Config identifyLedPinTable[] = {
        CONCENTRATOR_IDENTIFY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* Clock for sensor stub */
Clock_Struct ledBlinkClock;     /* Not static so you can see in ROV */
static Clock_Handle ledBlinkClockHandle;

static uint8_t ledBlinkCnt;

/***** Prototypes *****/
static void concentratorTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi);
static void addNewNode(struct AdcSensorNode* node);
static void updateNode(struct AdcSensorNode* node);
static uint8_t isKnownNodeAddress(uint8_t address);
static void ledBlinkClockCb(UArg arg0);

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
    Task_construct(&concentratorTask, concentratorTaskFunction, &concentratorTaskParams, NULL);

    /* Open Identify LED pin */
    identifyLedPinHandle = PIN_open(&identifyLedPinState, identifyLedPinTable);
    if (!identifyLedPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* Create Identify Clock to Blink LED */
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);

    clkParams.startFlag = FALSE;
    Clock_construct(&ledBlinkClock, ledBlinkClockCb, 1, &clkParams);
    ledBlinkClockHandle = Clock_handle(&ledBlinkClock);

    ledBlinkCnt = 0;
}

static void concentratorTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open UART types of display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplaySerial)
    {
        Trace_printf(hDisplaySerial, "Waiting for nodes...");
    }

    DataQ_init(0);

    /* Register a packet received callback with the radio task */
    ConcentratorRadioTask_registerPacketReceivedCallback(packetReceivedCallback);

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
            if(isKnownNodeAddress(latestActiveAdcSensorNode.address))
            {
                updateNode(&latestActiveAdcSensorNode);
            }
            else
            {
                /* Else add it */
                addNewNode(&latestActiveAdcSensorNode);
            }

            receivedPacket++;
            receivedDataSize += latestRawDataNode.length;
            successfullyReceivedPacket++;
            totalReceivedPacket++;
            totalReceivedDataSize += latestRawDataNode.length;
            totalSuccessfullyReceivedPacket++;

            static  char buffer[256] = {0,};
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

            Trace_printf(hDisplaySerial, "AT+RCVD: %4d.%03d, %d, %s", latestRawDataNode.time / 1000, latestRawDataNode.time % 1000, latestRawDataNode.length, buffer);

        }
        else if(events & CONCENTRATOR_EVENT_TEST_RESET)
        {
            /* If we knew this node from before, update the value */
            if(isKnownNodeAddress(latestActiveAdcSensorNode.address))
            {
                updateNode(&latestActiveAdcSensorNode);
            }
            else
            {
                /* Else add it */
                addNewNode(&latestActiveAdcSensorNode);
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
            Trace_printf(hDisplaySerial, "%8d %8d %8d %3d %8d %8d %8d %3d",
                           receivedDataSize, successfullyReceivedPacket, receivedPacket, successfullyReceivedPacket * 100 / receivedPacket,
                           totalReceivedDataSize, totalSuccessfullyReceivedPacket, totalReceivedPacket, totalSuccessfullyReceivedPacket * 100 / totalReceivedPacket);
            receivedPacket =  0;
            receivedDataSize = 0;
            successfullyReceivedPacket = 0;

            previousReceivedTime = currentReceivedTime;
        }
    }
}

static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi)
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

        DataQ_push(packet->rawDataPacket.data, packet->header.length);

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

static uint8_t isKnownNodeAddress(uint8_t address)
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

static void updateNode(struct AdcSensorNode* node)
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

static void addNewNode(struct AdcSensorNode* node)
{
    *lastAddedSensorNode = *node;

    /* Increment and wrap */
    lastAddedSensorNode++;
    if (lastAddedSensorNode > &knownSensorNodes[CONCENTRATOR_MAX_NODES-1])
    {
        lastAddedSensorNode = knownSensorNodes;
    }
}

static void ledBlinkClockCb(UArg arg0)
{
    if(ledBlinkCnt < CONCENTRATOR_LED_BLINK_TIMES)
    {
        uint32_t ledState = PIN_getOutputValue(CONCENTRATOR_IDENTIFY_LED);

        if(ledState)
        {
            ledBlinkCnt++;

            /* turn off LED */
            PIN_setOutputValue(identifyLedPinHandle, CONCENTRATOR_IDENTIFY_LED, 0);

            /* Setup timeout to turn LED on */
            Clock_setTimeout(ledBlinkClockHandle,
                CONCENTRATOR_LED_BLINK_OFF_DURATION_MS * 1000 / Clock_tickPeriod);

            /* Start sensor stub clock */
            Clock_start(ledBlinkClockHandle);
        }
        else
        {
            /* turn on LED */
            PIN_setOutputValue(identifyLedPinHandle, CONCENTRATOR_IDENTIFY_LED, 1);

            /* Setup timeout to turn LED off */
            Clock_setTimeout(ledBlinkClockHandle,
                CONCENTRATOR_LED_BLINK_ON_DURATION_MS * 1000 / Clock_tickPeriod);

            /* Start sensor stub clock */
            Clock_start(ledBlinkClockHandle);
        }
    }
    else
    {
        PIN_setOutputValue(identifyLedPinHandle, CONCENTRATOR_IDENTIFY_LED, 0);
        ledBlinkCnt = 0;
    }
}
