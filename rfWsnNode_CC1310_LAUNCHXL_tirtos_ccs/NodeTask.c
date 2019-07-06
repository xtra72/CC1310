/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "NodeTask.h"
#include "NodeRadioTask.h"
#include "DataQueue.h"
#include "Trace.h"
#include "mpu6050.h"

/***** Defines *****/
#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   3

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define NODE_EVENT_TRANSFER             (uint32_t)(1 << 1)
#define NODE_EVENT_TEST_RESET           (uint32_t)(1 << 2)
#define NODE_EVENT_AUTO_TRANSFER_START  (uint32_t)(1 << 3)
#define NODE_EVENT_AUTO_TRANSFER_STOP   (uint32_t)(1 << 4)
#define NODE_EVENT_OVERRUN_DETECTED     (uint32_t)(1 << 5)
#define NODE_EVENT_OVERRUN_RELEASED     (uint32_t)(1 << 6)
#define NODE_EVENT_DATA_ON              (uint32_t)(1 << 7)
#define NODE_EVENT_WAKEUP               (uint32_t)(1 << 8)
#define NODE_EVENT_SLEEP                (uint32_t)(1 << 9)
#define NODE_EVENT_MOTION_DETECTED      (uint32_t)(1 << 10)

#define NODE_ACTIVITY_LED           Board_PIN_LED0
#define NODE_MESSAGE_QUEUE_FULL_LED Board_PIN_LED1


/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* Not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* Not static so you can see in ROV */
static Event_Handle nodeEventHandle;

/* Clock for the transfer timeout */
Clock_Struct transferTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle transferTimeoutClockHandle;

/* Clock for the transfer timeout */
Clock_Struct messageTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle messageTimeoutClockHandle;

/* Clock for the motion detection timeout */
Clock_Struct motionDetectionTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle motionDetectionTimeoutClockHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
PIN_Config ledPinTable[] =
{
#if !defined Board_CC1350STK
    NODE_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    NODE_MESSAGE_QUEUE_FULL_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE
};

/* Display driver handles */
Display_Handle hDisplaySerial;


struct  TestConfig
{
    uint8_t     dataLength;
    uint32_t    period;
    uint8_t     loopCount;
};

static  struct TestConfig   testConfigs[5] =
{
 {  .dataLength = 4 + 16 * 1, .period = 2,  .loopCount = 100  },
 {  .dataLength = 4 + 16 * 2, .period = 2,  .loopCount = 100  },
 {  .dataLength = 4 + 16 * 3, .period = 3,  .loopCount = 100  },
 {  .dataLength = 4 + 16 * 4, .period = 3,  .loopCount = 100  },
 {  .dataLength = 4 + 16 * 5, .period = 4,  .loopCount = 100  }
};

static  uint8_t     configIndex = 0;
static  uint8_t     rawData[128];

static  uint32_t    sampleIndex = 0;

static  uint32_t    transferCount = 0;
static  uint32_t    transferSuccessCount = 0;
static  uint32_t    transferDataSize = 0;

static  uint32_t    totalTransferCount = 0;
static  uint32_t    totalTransferSuccessCount = 0;
static  uint32_t    totalTransferDataSize = 0;

static  uint32_t    previousTransferTime = 0;

static  Clock_Params    motionDetectedNotificationClockParams;
static  uint32_t    motionDetectedNotificationTryCount = 0;
static  uint32_t    motionDetectedNotificationMaxCount = 10;
static  uint32_t    motionDetectedNotificationPeriod = 10000;
static  uint32_t    messageGenerationOverrunSleep = 200;

static  bool        overrun = false;

#define msToClock(ms) ((ms) * 1000 / Clock_tickPeriod)

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);
static void transferTimeoutCallback(UArg arg0);
static void messageTimeoutCallback(UArg arg0);
static void motionDetectionTimeoutCallback(UArg arg0);

static void NodeTask_motionDetectFinishedCallback(PIN_Handle handle, PIN_Id pinId);


void NodeTask_notificationEvent(uint8_t eventType);

/***** Function definitions *****/
void NodeTask_init(void)
{

    DataQ_init(16);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    /* Create clock object which is used for fast report timeout */

//    Clock_construct(&transferTimeoutClock, transferTimeoutCallback, 1, &clkParams);
//    transferTimeoutClockHandle = Clock_handle(&transferTimeoutClock);

//    Clock_construct(&messageTimeoutClock, messageTimeoutCallback, 1, &clkParams);
//    messageTimeoutClockHandle = Clock_handle(&messageTimeoutClock);

    Clock_Params_init(&motionDetectedNotificationClockParams);
    motionDetectedNotificationClockParams.period = motionDetectedNotificationPeriod;
    motionDetectedNotificationClockParams.startFlag = FALSE;

    Clock_construct(&motionDetectionTimeoutClock, motionDetectionTimeoutCallback, 0, &motionDetectedNotificationClockParams);
    motionDetectionTimeoutClockHandle = Clock_handle(&motionDetectionTimeoutClock);

    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}

static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open both UART and LCD types of display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplaySerial)
    {
        Trace_printf(hDisplaySerial, "Waiting for SCE ADC reading...");
    }

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* setup timeout for fast report timeout */
//    Clock_setPeriod(transferTimeoutClockHandle, msToClock(transferPeriod));
//    Clock_setPeriod(messageTimeoutClockHandle, msToClock(testConfigs[configIndex].period));
    Clock_setPeriod(motionDetectionTimeoutClockHandle, msToClock(motionDetectedNotificationPeriod));

    MPU6050_init();

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If new ADC value, send this data */
        if (events & NODE_EVENT_AUTO_TRANSFER_START)
        {
            if (!Clock_isActive(transferTimeoutClockHandle))
            {
                Clock_start(transferTimeoutClockHandle);
                Clock_start(messageTimeoutClockHandle);
                Trace_printf(hDisplaySerial, "Transfer started");
            }

        }
        else if (events & NODE_EVENT_AUTO_TRANSFER_STOP)
        {
            if (Clock_isActive(transferTimeoutClockHandle))
            {
                Clock_stop(transferTimeoutClockHandle);
                Clock_stop(messageTimeoutClockHandle);
                Trace_printf(hDisplaySerial, "Transfer stopped");
            }
        }
        else if (events & NODE_EVENT_TEST_RESET)
        {
            uint32_t    currentTransferTime;
            /* Toggle activity LED */

#if !defined Board_CC1350STK
            PIN_setOutputValue(ledPinHandle, NODE_ACTIVITY_LED,!PIN_getOutputValue(NODE_ACTIVITY_LED));
#endif
            currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000000;

            Clock_stop(transferTimeoutClockHandle);
            Clock_stop(messageTimeoutClockHandle);

            if (NodeRadioTask_testReset() == NodeRadioStatus_Success)
            {
                configIndex = (configIndex + 1) % (sizeof(testConfigs) / sizeof(struct TestConfig));
                Trace_printf(hDisplaySerial, "Test Reset : %08d %08d", testConfigs[configIndex].dataLength, testConfigs[configIndex].loopCount);

                transferCount =  0;
                transferDataSize = 0;
                transferSuccessCount = 0;
                totalTransferCount =  0;
                totalTransferDataSize = 0;
                totalTransferSuccessCount = 0;
            }
            else
            {
                Trace_printf(hDisplaySerial, "Test Reset failed");
            }

            previousTransferTime = currentTransferTime;

        }
        else if (events & NODE_EVENT_TRANSFER)
        {
            /* Toggle activity LED */
            if (DataQ_count() != 0)
            {
                uint32_t    currentTransferTime;

                #if !defined Board_CC1350STK
                            PIN_setOutputValue(ledPinHandle, NODE_ACTIVITY_LED,!PIN_getOutputValue(NODE_ACTIVITY_LED));
                #endif

                uint32_t    length = 0;
                if (DataQ_front(rawData, sizeof(rawData), &length))
                {
                    transferCount++;
                    totalTransferCount++;
                    transferDataSize += length;
                    totalTransferDataSize += length;

                    currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000;

                    if (NodeRadioTask_sendRawData(rawData, length) == NodeRadioStatus_Success)
                    {
                        DataQ_pop(NULL, 0, &length);
                        transferSuccessCount++;
                        totalTransferSuccessCount++;
                    }
                    else
                    {
                        uint32_t    index = 0;

                        index = ((uint32_t)rawData[0] << 24) | ((uint32_t)rawData[1] << 16) | ((uint32_t)rawData[2] << 8) | (uint32_t)rawData[3];
                        Display_printf(hDisplaySerial, 0, 0, "Transfer error : %8x", index);
                    }
                }

                if (currentTransferTime / 1000 != previousTransferTime / 1000)
                {
                    Trace_printf(hDisplaySerial, "%8d %8d %8d %8d %8d %8d",
                                   transferCount, transferDataSize, transferSuccessCount,
                                   totalTransferCount, totalTransferDataSize, totalTransferSuccessCount);
                    transferCount =  0;
                    transferDataSize = 0;
                    transferSuccessCount = 0;
                }

                previousTransferTime = currentTransferTime;
            }
        }
        else
        {
            GPIO_write(Board_SPI_SLAVE_DATA_ON, !GPIO_read(Board_SPI_SLAVE_DATA_ON));
        }

        if( events & NODE_EVENT_OVERRUN_DETECTED)
        {
            overrun = true;
#if !defined Board_CC1350STK
            PIN_setOutputValue(ledPinHandle, NODE_MESSAGE_QUEUE_FULL_LED, 1);
#endif
            if (Clock_isActive(messageTimeoutClockHandle))
            {
                Clock_setPeriod(messageTimeoutClockHandle, msToClock(messageGenerationOverrunSleep));
                Clock_start(messageTimeoutClockHandle);
            }
            Trace_printf(hDisplaySerial, "Over Run detected");
        }
        else if( events & NODE_EVENT_OVERRUN_RELEASED)
        {
            overrun = false;
#if !defined Board_CC1350STK
            PIN_setOutputValue(ledPinHandle, NODE_MESSAGE_QUEUE_FULL_LED, 0);
#endif
            if (Clock_isActive(messageTimeoutClockHandle))
            {
                Clock_setPeriod(messageTimeoutClockHandle, msToClock(testConfigs[configIndex].period));
                Clock_start(messageTimeoutClockHandle);
            }
            Trace_printf(hDisplaySerial, "Over Run released");
        }

        if( events & NODE_EVENT_SLEEP)
        {
            uint32_t    i;

            for(i = 0 ; i < 10 ; i++)
            {
                Trace_printf(hDisplaySerial, "Start motion detect[%d]", i);
                if (MPU6050_startMotionDetect(0.4, false))
                {
                    Trace_printf(hDisplaySerial, "Motion detected");
                    NodeTask_motionDetected();
                    break;
                }
            }
        }

        if( events & NODE_EVENT_MOTION_DETECTED)
        {
            Clock_start(motionDetectionTimeoutClockHandle);
            motionDetectedNotificationTryCount = 0;

            NodeTask_notificationEvent(NODE_NOTI_MOTION_DETECTED);
        }

    }
}

/*
 *  ======== NodeTask_motionDetectFinishedCallback ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 */
static void NodeTask_motionDetectFinishedCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);

#if 0
    switch(pinId)
    {
    case    Board_PIN_BUTTON0:
        {
            if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
            {
                //start fast report and timeout
                Event_post(nodeEventHandle, NODE_EVENT_TEST_RESET);
            }
        }
        break;

    case    Board_PIN_BUTTON1:
        {

            if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
            {
                if (Clock_isActive(transferTimeoutClockHandle))
                {
                    Event_post(nodeEventHandle, NODE_EVENT_AUTO_TRANSFER_STOP);
                }
                else
                {
                    Event_post(nodeEventHandle, NODE_EVENT_AUTO_TRANSFER_START);
                }
            }
        }
        break;
    }
#endif
}

static void transferTimeoutCallback(UArg arg0)
{
    //stop fast report
    Event_post(nodeEventHandle, NODE_EVENT_TRANSFER);
}


static void messageTimeoutCallback(UArg arg0)
{
    //stop fast report
    static uint8_t buffer[128];
    uint8_t dataLength = 0;

    buffer[dataLength++] = (sampleIndex >> 24) & 0xFF;
    buffer[dataLength++] = (sampleIndex >> 16) & 0xFF;
    buffer[dataLength++] = (sampleIndex >>  8) & 0xFF;
    buffer[dataLength++] = (sampleIndex >>  0) & 0xFF;

    for(; dataLength < testConfigs[configIndex].dataLength ; )
    {
        buffer[dataLength++] = (uint8_t)(sampleIndex + dataLength);
    }

    if (DataQ_push(buffer, dataLength) == true)
    {
        if (overrun)
        {
            Event_post(nodeEventHandle, NODE_EVENT_OVERRUN_RELEASED);
        }
    }
    else
    {
        Event_post(nodeEventHandle, NODE_EVENT_OVERRUN_DETECTED);
    }
}

void motionDetectionTimeoutCallback(UArg arg0)
{
    motionDetectedNotificationTryCount++;

    if (motionDetectedNotificationTryCount < motionDetectedNotificationMaxCount)
    {
        NodeTask_notificationEvent(NODE_NOTI_MOTION_DETECTED);
    }
    else
    {
        NodeTask_sleep();
    }
}

void NodeTask_dataOn(void)
{
    //stop fast report
    Event_post(nodeEventHandle, NODE_EVENT_DATA_ON);
}

void NodeTask_startAutoTransfer(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_AUTO_TRANSFER_START);
}

void NodeTask_stopAutoTransfer(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_AUTO_TRANSFER_STOP);
}

bool NodeTask_dataTransfer(uint8_t* buffer, uint32_t length)
{
    if (DataQ_push(buffer, length))
    {
        Event_post(nodeEventHandle, NODE_EVENT_TRANSFER);

        return  true;
    }

    return  false;
}

void    NodeTask_sleep(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SLEEP);
}

void    NodeTask_wakeup(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_WAKEUP);
}

void    NodeTask_motionDetected(void)
{
    Trace_printf(hDisplaySerial, "Motion detected!");
    Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTED);
}

void NodeTask_notificationEvent(uint8_t eventType)
{
    //stop fast report
    uint8_t buffer[] = { 0, 0, 0, 0, 1, 0, 0, 4, 0, 0, 0, 0};

    memset(&buffer[8], eventType, 4);

    if (DataQ_push(buffer, sizeof(buffer)) == true)
    {
        Event_post(nodeEventHandle, NODE_EVENT_TRANSFER);
        if (overrun)
        {
            Event_post(nodeEventHandle, NODE_EVENT_OVERRUN_RELEASED);
        }
    }
    else
    {
        Event_post(nodeEventHandle, NODE_EVENT_OVERRUN_DETECTED);
    }
}

