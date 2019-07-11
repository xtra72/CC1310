/******************************************************************************

 @file ConcentratorRadioTask.c

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

#include "ConcentratorRadioTask.h"

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "RadioProtocol.h"
#include "DataQueue.h"
#include "crc16.h"

/***** Defines *****/
#define CONCENTRATORRADIO_TASK_STACK_SIZE 1024
#define CONCENTRATORRADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                  0xFFFFFFFF
#define RADIO_EVENT_VALID_PACKET_RECEIVED      (uint32_t)(1 << 0)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED (uint32_t)(1 << 1)
#define RADIO_EVENT_SEND_PACKET_READY       (uint32_t)(2 << 0)

#define CONCENTRATORRADIO_MAX_RETRIES 2

//Delay is needed to account for DMM TX<->RX switch time in us  
#define CONCENTRATORRADIO_ACK_DELAY  250

#define CONCENTRATOR_ACTIVITY_LED Board_PIN_LED0 

/***** Type declarations *****/
typedef struct
{
    uint8_t destAddr;
    uint8_t length;
    uint8_t payload[EASYLINK_MAX_DATA_LENGTH];
}   ConsentratorRadioTaskPostPacket;


/***** Prototypes *****/
static void concentratorRadioTask_main(UArg arg0, UArg arg1);
static void ConcentratorRadioTask_rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void ConcentratorRadioTask_notifyPacketReceived(union ConcentratorPacket* latestRxPacket);
static bool ConsentratorRadioTask_send(uint8_t _destAddr, uint8_t* _payload, uint32_t _len, uint32_t _timeout);
static bool ConsentratorRadioTask_sendAck(uint8_t latestSourceAddress, uint32_t _timeout);

/***** Variable declarations *****/
static Task_Params concentratorRadioTaskParams;
Task_Struct concentratorRadioTask; /* not static so you can see in ROV */
static uint8_t concentratorRadioTaskStack[CONCENTRATORRADIO_TASK_STACK_SIZE];
Event_Struct radioOperationEvent;  /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;



static ConcentratorRadio_PacketReceivedCallback packetReceivedCallback;
static union ConcentratorPacket latestRxPacket;
static EasyLink_TxPacket txPacket;
static struct AckPacket ackPacket;
static uint8_t concentratorAddress;
static int8_t latestRssi;

static  Semaphore_Struct    sendSem;  /* not static so you can see in ROV */
static  Semaphore_Handle    sendSemHandle;

static  DataQ   dataQ_;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Configure LED Pin */
PIN_Config ledPinTable[] = {
        CONCENTRATOR_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Function definitions *****/
void ConcentratorRadioTask_init(void) {

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
	if (!ledPinHandle)
	{
        System_abort("Error initializing board 3.3V domain pins\n");
    }

	DataQ_init(&dataQ_, 4);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&sendSem, 1, &semParam);
    sendSemHandle = Semaphore_handle(&sendSem);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorRadioTaskParams);
    concentratorRadioTaskParams.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
    concentratorRadioTaskParams.priority = CONCENTRATORRADIO_TASK_PRIORITY;
    concentratorRadioTaskParams.stack = &concentratorRadioTaskStack;
    Task_construct(&concentratorRadioTask, concentratorRadioTask_main, &concentratorRadioTaskParams, NULL);
}

void ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorRadio_PacketReceivedCallback callback) {
    packetReceivedCallback = callback;
}

static void concentratorRadioTask_main(UArg arg0, UArg arg1)
{
    // Initialize the EasyLink parameters to their default values
	EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

#ifdef DEFINED_RADIO_EASYLINK_MODULATION
    // Change the modulation from the default found in easylink_config.h
	easyLink_params.ui32ModType = DEFINED_RADIO_EASYLINK_MODULATION;
#endif
	
    /* Initialize EasyLink */
	if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success){ 
		System_abort("EasyLink_init failed");
	}	

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Set concentrator address */;
    concentratorAddress = RADIO_CONCENTRATOR_ADDRESS;
    EasyLink_enableRxAddrFilter(&concentratorAddress, 1, 1);

    /* Set up Ack packet */
    ackPacket.header.sourceAddress = concentratorAddress;
    ackPacket.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;

    /* Enter receive */
    if(EasyLink_receiveAsync(ConcentratorRadioTask_rxDoneCallback, 0) != EasyLink_Status_Success) {
        System_abort("EasyLink_receiveAsync failed");
    }

    while (1) {
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If valid packet received */
        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED) {

            /* Send ack packet */
            ConsentratorRadioTask_sendAck(latestRxPacket.header.sourceAddress, BIOS_WAIT_FOREVER);

            /* Call packet received callback */
            ConcentratorRadioTask_notifyPacketReceived(&latestRxPacket);

            /* Go back to RX */
            if(EasyLink_receiveAsync(ConcentratorRadioTask_rxDoneCallback, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }


            /* toggle Activity LED */
            PIN_setOutputValue(ledPinHandle, CONCENTRATOR_ACTIVITY_LED, !PIN_getOutputValue(CONCENTRATOR_ACTIVITY_LED));
        }

        /* If invalid packet received */
        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED) {
            /* Go back to RX */
            if(EasyLink_receiveAsync(ConcentratorRadioTask_rxDoneCallback, 0) != EasyLink_Status_Success) {
                //System_abort("EasyLink_receiveAsync failed");
            }
        }

        if(events & RADIO_EVENT_SEND_PACKET_READY)
        {
            while(DataQ_count(&dataQ_) != 0)
            {
                DataQ_lock(&dataQ_, BIOS_WAIT_FOREVER);

                DataQItem*  item = DataQ_front(&dataQ_);

                ConsentratorRadioTaskPostPacket* packet = (ConsentratorRadioTaskPostPacket*)item->data;
                ConsentratorRadioTask_send(packet->destAddr, packet->payload, packet->length, BIOS_WAIT_FOREVER);
                DataQ_unlock(&dataQ_);

                DataQ_pop(&dataQ_, NULL, 0, 0, BIOS_WAIT_FOREVER);
            }
        }
    }
}


bool    ConcentratorRadioTask_post(uint8_t _address, uint8_t* _payload, uint32_t _len)
{
    DataQItem*  item = DataQ_lazyPushBegin(&dataQ_);
    if (!item)
    {
        return  false;
    }

    ((ConsentratorRadioTaskPostPacket*)item->data)->destAddr = _address;
    ((ConsentratorRadioTaskPostPacket*)item->data)->length = _len;
    memcpy(((ConsentratorRadioTaskPostPacket*)item->data)->payload, _payload, _len);

    DataQ_lazyPushEnd(&dataQ_, false);
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_PACKET_READY);


    return  true;

}

bool ConsentratorRadioTask_send(uint8_t _destAddr, uint8_t* _payload, uint32_t _len, uint32_t _timeout)
{
    bool    ret = false;
    uint32_t absTime;

    if (Semaphore_pend(sendSemHandle, _timeout))
    {
            /* Set destinationAdress, but use EasyLink layers destination adress capability */
        txPacket.dstAddr[0] = _destAddr;

        /* Copy ACK packet to payload, skipping the destination adress byte.
         * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
        union ConcentratorPacket   *packet = (union ConcentratorPacket *)txPacket.payload;
        packet->header.sourceAddress = concentratorAddress;
        packet->header.packetType = RADIO_PACKET_TYPE_RAW_DATA_PACKET;
        packet->header.options = RADIO_PACKET_OPTIONS_CRC;
        packet->header.length = _len;

        memcpy(packet->rawDataPacket.data, _payload, _len);
        txPacket.len = sizeof(struct PacketHeader) + sizeof(uint16_t) + _len;

#if 0
        if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
        {
            // Problem getting absolute time
            // Still send ACK
            txPacket.absTime = 0;
        }
        else
        {
            txPacket.absTime = absTime + EasyLink_us_To_RadioTime(CONCENTRATORRADIO_ACK_DELAY);
        }
#else
        txPacket.absTime = 0;
#endif

        if (EasyLink_transmit(&txPacket) == EasyLink_Status_Success)
        {
            ret = true;
        }

        Semaphore_post(sendSemHandle);
    }

    return  ret;
}

bool ConsentratorRadioTask_sendAck(uint8_t latestSourceAddress, uint32_t _timeout)
{
    bool    ret = false;

    if (Semaphore_pend(sendSemHandle, _timeout))
    {
        uint32_t absTime;

        /* Set destinationAdress, but use EasyLink layers destination adress capability */
        txPacket.dstAddr[0] = latestSourceAddress;

        /* Copy ACK packet to payload, skipping the destination adress byte.
         * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
        memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
        txPacket.len = sizeof(ackPacket);

        if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
        {
            // Problem getting absolute time
            // Still send ACK
            txPacket.absTime = 0;
        }
        else
        {
            txPacket.absTime = absTime + EasyLink_us_To_RadioTime(CONCENTRATORRADIO_ACK_DELAY);
        }

        if (EasyLink_transmit(&txPacket) == EasyLink_Status_Success)
        {
            ret = true;
        }

        Semaphore_post(sendSemHandle);
    }

    return  ret;
}

static void ConcentratorRadioTask_notifyPacketReceived(union ConcentratorPacket* latestRxPacket)
{
    if (packetReceivedCallback)
    {
        packetReceivedCallback(latestRxPacket, latestRssi);
    }
}

static void ConcentratorRadioTask_rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    union ConcentratorPacket* tmpRxPacket;

    /* If we received a packet successfully */
    if (status == EasyLink_Status_Success)
    {
        /* Save the latest RSSI, which is later sent to the receive callback */
        latestRssi = (int8_t)rxPacket->rssi;

        /* Check that this is a valid packet */
        tmpRxPacket = (union ConcentratorPacket*)(rxPacket->payload);

        /* If this is a known packet */
        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_RAW_DATA_PACKET)
        {
            uint32_t i;
            uint32_t offset = 0;

            /* Save packet */
            latestRxPacket.header.sourceAddress = rxPacket->payload[offset++];
            latestRxPacket.header.packetType = rxPacket->payload[offset++];
            latestRxPacket.header.options = rxPacket->payload[offset++];
            latestRxPacket.header.length = rxPacket->payload[offset++];

            if (latestRxPacket.header.options & RADIO_PACKET_OPTIONS_CRC)
            {
                uint16_t    crc;
                uint16_t    receivedCRC;

                receivedCRC = (uint16_t)rxPacket->payload[offset++] << 8;
                receivedCRC |= rxPacket->payload[offset++];

                crc = CRC16_calc(&rxPacket->payload[offset], latestRxPacket.header.length);
                if (receivedCRC != crc)
                {
                    /* Signal invalid packet received */
                    Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
                    return;
                }
            }

            if (latestRxPacket.header.length < sizeof(latestRxPacket.rawDataPacket.data))
            {
                for(i = 0; i < latestRxPacket.header.length  ; i++)
                {
                    latestRxPacket.rawDataPacket.data[i] = rxPacket->payload[offset++];
                }

                /* Signal packet received */
                Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
            }
            else
            {
                /* Signal invalid packet received */
                Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
            }

        }
        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_TEST_RESET)
        {
            uint32_t offset = 0;

            /* Save packet */
            latestRxPacket.header.sourceAddress = rxPacket->payload[offset++];
            latestRxPacket.header.packetType = rxPacket->payload[offset++];
            latestRxPacket.header.options = rxPacket->payload[offset++];
            latestRxPacket.header.length = rxPacket->payload[offset++];

            /* Signal packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else
        {
            /* Signal invalid packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
        }
    }
    else
    {
        /* Signal invalid packet received */
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }
}


uint8_t  ConcentratorRadioTask_getAddress(void)
{
    return  concentratorAddress;
}

int8_t  ConcentratorRadioTask_getRssi(void)
{
    return  latestRssi;
}
