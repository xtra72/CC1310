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

#include "easylink/EasyLink.h"

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "NodeTask.h"
#include "NodeRadioTask.h"
#include "DataQueue.h"
#include "Trace.h"
#include "mpu6050.h"
#include "SpiSlave.h"
#include "rf.h"

/***** Defines *****/
#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   3

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define NODE_EVENT_DATA_TRANSFER        (uint32_t)(1 << 1)
#define NODE_EVENT_POST_TRANSFER        (uint32_t)(1 << 2)
#define NODE_EVENT_TEST_RESET           (uint32_t)(1 << 3)
#define NODE_EVENT_TEST_TRANSFER_START  (uint32_t)(1 << 4)
#define NODE_EVENT_TEST_TRANSFER_STOP   (uint32_t)(1 << 5)
#define NODE_EVENT_OVERRUN_DETECTED     (uint32_t)(1 << 6)
#define NODE_EVENT_OVERRUN_RELEASED     (uint32_t)(1 << 7)
#define NODE_EVENT_DATA_ON              (uint32_t)(1 << 8)
#define NODE_EVENT_WAKEUP               (uint32_t)(1 << 9)
#define NODE_EVENT_MOTION_DETECTION_START   (uint32_t)(1 << 10)
#define NODE_EVENT_MOTION_DETECTION_RUN     (uint32_t)(1 << 11)
#define NODE_EVENT_MOTION_DETECTION_STOP    (uint32_t)(1 << 12)
#define NODE_EVENT_MOTION_DETECTED          (uint32_t)(1 << 13)

#define NODE_EVENT_SRV_NOTI_TRANSFER_START            (uint32_t)(1 << 14)
#define NODE_EVENT_SRV_NOTI_TRANSFER_STOP             (uint32_t)(1 << 15)
#define NODE_EVENT_SRV_NOTI_SCAN_START                (uint32_t)(1 << 16)
#define NODE_EVENT_SRV_NOTI_SCAN_STOP                 (uint32_t)(1 << 17)
#define NODE_EVENT_SRV_NOTI_MOTION_DETECTION_START    (uint32_t)(1 << 18)
#define NODE_EVENT_SRV_NOTI_MOTION_DETECTION_STOP     (uint32_t)(1 << 19)
#define NODE_EVENT_DOWNLINK                             (uint32_t)(1 << 20)

#define NODE_EVENT_MOTION_DETECTION_FINISHED            (uint32_t)(1 << 21)

#define TRANSFER_EVENT_ALL              0xFFFFFFFF
#define TRANSFER_EVENT_SUCCESS          (uint32_t)(1 << 1)
#define TRANSFER_EVENT_FAILED           (uint32_t)(1 << 2)

#define NODE_MESSAGE_QUEUE_FULL_LED Board_PIN_LED1



/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* Not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* Not static so you can see in ROV */
static Event_Handle nodeEventHandle;

Event_Struct transferEvent;  /* Not static so you can see in ROV */
static Event_Handle transferEventHandle;

/* Clock for the transfer timeout */
Clock_Struct transferTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle transferTimeoutClockHandle;

/* Clock for the transfer timeout */
Clock_Struct messageTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle messageTimeoutClockHandle;

/* Clock for the motion detection timeout */
Clock_Struct postMotionDetectedClock;     /* not static so you can see in ROV */
static Clock_Handle postMotionDetectedClockHandle;


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
static  uint32_t    transferMaxRetryCount = 10;
static  uint32_t    transferRetryCount = 0;

static  uint32_t    totalTransferCount = 0;
static  uint32_t    totalTransferSuccessCount = 0;
static  uint32_t    totalTransferDataSize = 0;

static  uint32_t    previousTransferTime = 0;

static  uint32_t    messageGenerationOverrunSleep = 200;

static  bool        overrun = false;

static  uint8_t     status_ = 0;

static  float       motionDetectionLimit_ = 0.4;
static  uint32_t    motionDetectionCountTry_ = 0;
static  uint32_t    motionDetectionCountMax_ = 0;

static  uint32_t    noitificationCountTry_ = 0;
static  uint32_t    noitificationCountMax_ = 10;

static  uint8_t     directTransferData[128];
static  uint32_t    directTransferDataLength = 0;

static  uint8_t     downlinkData[128];
static  uint8_t     downlinkLength = 0;

#define msToClock(ms) ((ms) * 1000 / Clock_tickPeriod)

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);
static void transferTimeoutCallback(UArg arg0);
static void messageTimeoutCallback(UArg arg0);
static void postMotionDetectedCallback(UArg arg0);

static void NodeTask_eventTestTransferStart(void);
static void NodeTask_eventTestTransferStop(void);
static void NodeTask_eventTestReset(void);

static void NodeTask_motionDetectionRun(void);
static void NodeTask_motionDetectionFinished(void);

static void NodeTask_eventDataTransfer(void);
static void NodeTask_eventPostTransfer(void);

static void NodeTask_postNotification(uint8_t _type);
static void NodeTask_postMotionDetected(void);

static void NodeTask_dataTransferSuccess(void);
static void NodeTask_dataTransferFailed(void);


/***** Function definitions *****/
void NodeTask_init(void)
{

    DataQ_init(16);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    Event_Params_init(&eventParam);
    Event_construct(&transferEvent, &eventParam);
    transferEventHandle = Event_handle(&transferEvent);

    /* Create clock object which is used for fast report timeout */
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);

    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&transferTimeoutClock, transferTimeoutCallback, 1, &clkParams);
    transferTimeoutClockHandle = Clock_handle(&transferTimeoutClock);

    Clock_construct(&messageTimeoutClock, messageTimeoutCallback, 1, &clkParams);
    messageTimeoutClockHandle = Clock_handle(&messageTimeoutClock);

    Clock_construct(&postMotionDetectedClock, postMotionDetectedCallback, 0, &clkParams);
    postMotionDetectedClockHandle = Clock_handle(&postMotionDetectedClock);

    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}

static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    MPU6050_init();

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, 100000);

        /* If new ADC value, send this data */
        if (events & NODE_EVENT_TEST_TRANSFER_START)
        {
            NodeTask_eventTestTransferStart();
        }
        else if (events & NODE_EVENT_TEST_TRANSFER_STOP)
        {
            NodeTask_eventTestTransferStop();
        }
        else if (events & NODE_EVENT_TEST_RESET)
        {
            NodeTask_eventTestReset();
        }
        else if (events & NODE_EVENT_DATA_TRANSFER)
        {
            NodeTask_eventDataTransfer();
        }
        else if (events & NODE_EVENT_POST_TRANSFER)
        {
            NodeTask_eventPostTransfer();
        }

        if( events & NODE_EVENT_OVERRUN_DETECTED)
        {
            overrun = true;
            if (Clock_isActive(messageTimeoutClockHandle))
            {
                Clock_setPeriod(messageTimeoutClockHandle, msToClock(messageGenerationOverrunSleep));
                Clock_start(messageTimeoutClockHandle);
            }
            Trace_printf("Over Run detected");
        }
        else if( events & NODE_EVENT_OVERRUN_RELEASED)
        {
            overrun = false;
            if (Clock_isActive(messageTimeoutClockHandle))
            {
                Clock_setPeriod(messageTimeoutClockHandle, msToClock(testConfigs[configIndex].period));
                Clock_start(messageTimeoutClockHandle);
            }
            Trace_printf("Over Run released");
        }

        if( events & NODE_EVENT_MOTION_DETECTION_START)
        {
            Trace_printf("Motion detection start.");
            motionDetectionCountTry_ = 0;
            NodeTask_motionDetectionRun();
            status_ &= ~NODE_STATUS_FLAG_MOTION_DETECTED;
        }
        if( events & NODE_EVENT_MOTION_DETECTION_RUN)
        {
            NodeTask_motionDetectionRun();
        }
        else if( events & NODE_EVENT_MOTION_DETECTION_FINISHED)
        {
            NodeTask_motionDetectionFinished();
        }
        else if( events & NODE_EVENT_MOTION_DETECTION_STOP)
        {
            MPU6050_stop();
            Clock_stop(postMotionDetectedClockHandle);
            status_ &= ~NODE_STATUS_FLAG_MOTION_DETECTED;
            Trace_printf("Motion detection stop.");
        }
        else if( events & NODE_EVENT_MOTION_DETECTED)
        {
            Trace_printf("Motion detected.");

            status_ |= NODE_STATUS_FLAG_MOTION_DETECTED;

            noitificationCountTry_ =  0;

            NodeTask_postMotionDetected();
            Clock_setPeriod(postMotionDetectedClockHandle, msToClock(10000));
            Clock_start(postMotionDetectedClockHandle);
        }
        else if( events & NODE_EVENT_DOWNLINK)
        {
            Trace_printf("Downlink : %d.", downlinkLength);

            SpiSlave_downlink(downlinkData, downlinkLength);
        }

        if( events & NODE_EVENT_SRV_NOTI_SCAN_START)
        {
            SpiSlave_setNotification(RF_REQ_SRV_SCAN_START);
        }
        else if( events & NODE_EVENT_SRV_NOTI_SCAN_STOP)
        {
            SpiSlave_setNotification( RF_REQ_SRV_SCAN_STOP);
        }
        else if( events & NODE_EVENT_SRV_NOTI_TRANSFER_START)
        {
            SpiSlave_setNotification( RF_REQ_SRV_TRANSFER_START);
        }
        else if( events & NODE_EVENT_SRV_NOTI_TRANSFER_STOP)
        {
            SpiSlave_setNotification( RF_REQ_SRV_TRANSFER_STOP);
        }
        else if( events & NODE_EVENT_SRV_NOTI_MOTION_DETECTION_START)
        {
            SpiSlave_setNotification( RF_REQ_SRV_MOTION_DETECTION_START);
        }
        else if( events & NODE_EVENT_SRV_NOTI_MOTION_DETECTION_STOP)
        {
            SpiSlave_setNotification( RF_REQ_SRV_MOTION_DETECTION_STOP);
        }
    }
}

static void transferTimeoutCallback(UArg arg0)
{
    //stop fast report
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

void postMotionDetectedCallback(UArg arg0)
{
    if (++noitificationCountTry_ <= noitificationCountMax_)
    {
        NodeTask_postMotionDetected();
    }
    else
    {
        Clock_stop(postMotionDetectedClockHandle);
        Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTION_START);
    }
}

uint8_t NodeTask_getStatus(void)
{
    return  status_;
}

void NodeTask_dataOn(void)
{
    //stop fast report
    Event_post(nodeEventHandle, NODE_EVENT_DATA_ON);
}

void NodeTask_testTransferStart(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_TEST_TRANSFER_START);
}

void NodeTask_testTransferStop(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_TEST_TRANSFER_STOP);
}

bool NodeTask_dataTransfer(uint8_t* buffer, uint32_t length)
{
    memcpy(directTransferData, buffer, length);
    directTransferDataLength = length;

    Event_post(nodeEventHandle, NODE_EVENT_DATA_TRANSFER);

    uint32_t events = Event_pend(transferEventHandle, 0, NODE_EVENT_ALL, 1000);
    if (events & TRANSFER_EVENT_SUCCESS)
    {
        return  true;
    }

    return  false;
}

bool NodeTask_postTransfer(uint8_t* buffer, uint32_t length)
{
    if (DataQ_push(buffer, length) == true)
    {
        Event_post(nodeEventHandle, NODE_EVENT_POST_TRANSFER);
        return  true;
    }

    return  false;
}

void    NodeTask_dataTransferSuccess(void)
{
    Event_post(transferEventHandle, TRANSFER_EVENT_SUCCESS);
}

void    NodeTask_dataTransferFailed(void)
{
    Event_post(transferEventHandle, TRANSFER_EVENT_FAILED);
}

void    NodeTask_wakeup(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_WAKEUP);
}

bool    NodeTask_motionDetectionStart(void)
{
    motionDetectionCountTry_ = 0;
    Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTION_START);

    return  true;
}

bool    NodeTask_motionDetectionStop(void)
{
    motionDetectionCountTry_ = motionDetectionCountMax_;
    Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTION_STOP);

    return  true;
}

void    NodeTask_postNotification(uint8_t _type)
{
    //stop fast report
    static uint8_t buffer[128];
    uint8_t dataLength = 0;

    buffer[dataLength++] = 0;
    buffer[dataLength++] = 0;
    buffer[dataLength++] = 0;
    buffer[dataLength++] = 1;
    buffer[dataLength++] = 1;   // Port
    buffer[dataLength++] = 0;   // Option
    buffer[dataLength++] = 0;   // Count
    buffer[dataLength++] = 1;   // Size
    buffer[dataLength++] = _type;

    if (DataQ_push(buffer, dataLength) == true)
    {
        Event_post(nodeEventHandle, NODE_EVENT_POST_TRANSFER);
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

void    NodeTask_postMotionDetected(void)
{
    Trace_printf("[NOTI] : Motion detected");
    SpiSlave_setCommand(RF_IO_NOTI_MOTION_DETECTED);
}

void    NodeTask_eventTestTransferStart(void)
{
    if (!Clock_isActive(transferTimeoutClockHandle))
    {
        Clock_start(transferTimeoutClockHandle);
        Clock_start(messageTimeoutClockHandle);
        Trace_printf("Transfer started");
    }
    else
    {
        Trace_printf("Transfer already started");
    }
}

void    NodeTask_eventTestTransferStop(void)
{
    if (Clock_isActive(transferTimeoutClockHandle))
    {
        Clock_stop(transferTimeoutClockHandle);
        Clock_stop(messageTimeoutClockHandle);
        Trace_printf("Transfer stopped");
    }
    else
    {
        Trace_printf("Transfer not started");
    }
}

void    NodeTask_eventTestReset(void)
{
    uint32_t    currentTransferTime;
    /* Toggle activity LED */

    currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000000;

    Clock_stop(transferTimeoutClockHandle);
    Clock_stop(messageTimeoutClockHandle);

    if (NodeRadioTask_testReset() == NodeRadioStatus_Success)
    {
        configIndex = (configIndex + 1) % (sizeof(testConfigs) / sizeof(struct TestConfig));
        Trace_printf("Test Reset : %08d %08d", testConfigs[configIndex].dataLength, testConfigs[configIndex].loopCount);

        transferCount =  0;
        transferDataSize = 0;
        transferSuccessCount = 0;
        totalTransferCount =  0;
        totalTransferDataSize = 0;
        totalTransferSuccessCount = 0;
    }
    else
    {
        Trace_printf("Test Reset failed");
    }

    previousTransferTime = currentTransferTime;
}

void    NodeTask_motionDetectionRun(void)
{
    ++motionDetectionCountTry_;
    MPU6050_start();
    Trace_printf("motion detection Run[%d]", motionDetectionCountTry_);
}

void    NodeTask_motionDetectionFinished(void)
{
    float   value;

    MPU6050_stop();

    value = MPU6050_getOscillationValue();
    if (value >= motionDetectionLimit_)
    {
        Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTED);
    }
    else
    {
        if ((motionDetectionCountMax_ == 0) || (motionDetectionCountTry_ < motionDetectionCountMax_))
        {
            Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTION_RUN);
        }
    }
}

void    NodeTask_notifyMotionDetectionFinished(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_MOTION_DETECTION_FINISHED);
}

void    NodeTask_scanStart(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SRV_NOTI_SCAN_START);
}

void    NodeTask_scanStop(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SRV_NOTI_SCAN_STOP);
}

void    NodeTask_transferStart(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SRV_NOTI_TRANSFER_START);
}

void    NodeTask_transferStop(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SRV_NOTI_TRANSFER_STOP);
}

void    NodeTask_motionStart(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SRV_NOTI_MOTION_DETECTION_START);
}

void    NodeTask_motionStop(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_SRV_NOTI_MOTION_DETECTION_STOP);
}

void    NodeTask_downlink(uint8_t* data, uint8_t length)
{
    memcpy(downlinkData, data, length);
    downlinkLength = length;

    Event_post(nodeEventHandle, NODE_EVENT_DOWNLINK);
}

void    NodeTask_eventDataTransfer(void)
{
    if (NodeRadioTask_sendRawData(directTransferData, directTransferDataLength) == NodeRadioStatus_Success)
    {
        NodeTask_dataTransferSuccess();
    }
    else
    {
        NodeTask_dataTransferFailed();
    }
}

void    NodeTask_eventPostTransfer(void)
{
    /* Toggle activity LED */
    if (DataQ_count() != 0)
    {
        uint32_t    currentTransferTime;

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
                transferRetryCount = 0;
                transferSuccessCount++;
                totalTransferSuccessCount++;
            }
            else
            {
                uint32_t    index = 0;
                index = ((uint32_t)rawData[0] << 24) | ((uint32_t)rawData[1] << 16) | ((uint32_t)rawData[2] << 8) | (uint32_t)rawData[3];
                Trace_printf("Transfer error : %8x, %d", index, transferRetryCount++);
                if (transferMaxRetryCount <= transferRetryCount)
                {
                    DataQ_pop(NULL, 0, &length);
                    Trace_printf("Packet drop : %8x", index);
                }
            }
        }

        if (currentTransferTime / 1000 != previousTransferTime / 1000)
        {
            Trace_printf("%8d %8d %8d %8d %8d %8d",
                           transferCount, transferDataSize, transferSuccessCount,
                           totalTransferCount, totalTransferDataSize, totalTransferSuccessCount);
            transferCount =  0;
            transferDataSize = 0;
            transferSuccessCount = 0;
        }

        previousTransferTime = currentTransferTime;
    }

    if (DataQ_count() != 0)
    {
        Event_post(nodeEventHandle, NODE_EVENT_POST_TRANSFER);
    }

}

void    NodeTask_getRFStatus(NODETASK_STATUS* status)
{
    status->frequency = EasyLink_getFrequency();
    EasyLink_getRfPower(&status->power);
    EasyLink_getRssi(&status->rssi);
}


void    NodeTask_getConfig(NODETASK_CONFIG* config)
{
    config->shortAddress = nodeRadioTask_getNodeAddr();
    int8_t power;
    EasyLink_getRfPower(&power);
    config->power = power;
    config->frequency = EasyLink_getFrequency();
    config->timeout = nodeRadioTask_getAckTimeout();
}


bool    NodeTask_setConfig(NODETASK_CONFIG* config)
{
    EasyLink_setFrequency(config->frequency);

    return  true;
}

uint32_t NodeTask_getQueueSize(void)
{
    return  (DataQ_maxCount() -  DataQ_count());
}
