#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Types.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Log.h>
#include <Board.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include "encoder.h"

GPTimerCC26XX_Handle    timerChannelA;
GPTimerCC26XX_Handle    timerChannelB;
PIN_Handle              timerPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State        timerPinState;

int32_t    countA = 0;
int32_t     offsetA = 0;
bool        directionUp = true;
bool        reverse_ = false;
int32_t     match_ = 10;

PIN_Config GptPinInitTable[] = {
    IOID_22   | PIN_INPUT_EN | PIN_PULLUP,
    IOID_24   | PIN_INPUT_EN | PIN_PULLUP,
    PIN_TERMINATE
};

ENCODER encoder_ =
{
     .head = 0,
     .tail = 0,
     .index = 0,
     .records = {{0,}}
};

void captureCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    if ((!reverse_ && (PIN_getInputValue(IOID_22) == PIN_getInputValue(IOID_24))) || (reverse_ && (PIN_getInputValue(IOID_22) != PIN_getInputValue(IOID_24))))
    {
        if (!directionUp)
        {
            directionUp = true;
        }
        else
        {
            countA++;
        }
    }
    else
    {
        if (directionUp)
        {
            directionUp = false;
        }
        else
        {
            countA--;
        }
    }
}

void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    static  int loop = 0;

    if (++loop >= 10)
    {
        if (encoder_.index >= 10)
        {

            if (((encoder_.head + 1) % 20) == encoder_.tail)
            {
                encoder_.tail = (encoder_.tail + 1) % 20;
            }
            encoder_.head = (encoder_.head + 1) % 20;
            encoder_.index = 0;
        }

        if (encoder_.index == 0)
        {
            encoder_.records[encoder_.head].time = (Clock_getTicks() / (1000 / Clock_tickPeriod));
        }
        encoder_.records[encoder_.head].value[encoder_.index++] = Encoder_getCount();

        loop = 0;
    }
  //  while(1);
}


void  Encoder_start(void)
{
    GPTimerCC26XX_start(timerChannelA);
    GPTimerCC26XX_start(timerChannelB);
}


void  Encoder_stop(void)
{
    GPTimerCC26XX_stop(timerChannelA);
    GPTimerCC26XX_stop(timerChannelB);
}


void    Encoder_init(void)
{
    GPTimerCC26XX_PinMux pinMux;
    GPTimerCC26XX_Params timerParams;
    GPTimerCC26XX_Params_init(&timerParams);
    timerParams.width          = GPT_CONFIG_16BIT;
    timerParams.mode           = GPT_MODE_EDGE_COUNT_UP;
    timerChannelA = GPTimerCC26XX_open(Board_GPTIMER0A, &timerParams);
    if(timerChannelA == NULL)
    {
      Log_error0("Failed to open GPTimer");
      Task_exit();
    }

    GPTimerCC26XX_setCaptureEdge(timerChannelA, GPTimerCC26XX_POS_EDGE);
    /* Register interrupt when capture happens */
    GPTimerCC26XX_registerInterrupt(timerChannelA, captureCallback, GPT_INT_CAPTURE_MATCH);
    /* Open pin handle and route pin to timer */
    timerPinHandle = PIN_open(&timerPinState, GptPinInitTable);
    pinMux = GPTimerCC26XX_getPinMux(timerChannelA);
    PINCC26XX_setMux(timerPinHandle, PIN_ID(IOID_22), pinMux);

    GPTimerCC26XX_setMatchValue(timerChannelA, match_);
    GPTimerCC26XX_setLoadValue(timerChannelA, match_*2);


    timerParams.width          = GPT_CONFIG_16BIT;
    timerParams.mode           = GPT_MODE_PERIODIC_UP;
    timerParams.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    timerChannelB = GPTimerCC26XX_open(Board_GPTIMER1A, &timerParams);
    if(timerChannelB == NULL)
    {
      Log_error0("Failed to open GPTimer");
      Task_exit();
    }

    Types_FreqHz  freq;
    BIOS_getCpuFreq(&freq);
    GPTimerCC26XX_Value loadVal = freq.lo / 1000 - 1; //47999
    GPTimerCC26XX_setLoadValue(timerChannelB, loadVal);
    GPTimerCC26XX_registerInterrupt(timerChannelB, timerCallback, GPT_INT_TIMEOUT);
}

void    Encoder_setCount(int32_t count)
{
    directionUp = true;

    if(count >= 0)
    {
        countA = count / match_;
        offsetA = GPTimerCC26XX_getValue(timerChannelA) - (count % match_);
    }
    else
    {
        countA = count / match_;
        offsetA = - (match_ - GPTimerCC26XX_getValue(timerChannelA)) - (count % match_);
    }
}

int32_t    Encoder_getCount(void)
{
    if (timerChannelA)
    {
        if (directionUp)
        {
            if (countA >= 0)
            {
                return (countA * match_) +  GPTimerCC26XX_getValue(timerChannelA) - offsetA;
            }
            else
            {
                return (countA * match_) -  (match_ - GPTimerCC26XX_getValue(timerChannelA)) - offsetA;
            }
        }
        else
        {
            if (countA >= 0)
            {
                return (countA * match_) +  (match_ - GPTimerCC26XX_getValue(timerChannelA)) - offsetA;
            }
            else
            {
                return (countA * match_) -  GPTimerCC26XX_getValue(timerChannelA) - offsetA;
            }
        }
    }

    return  0;
}


bool    Encoder_isUp(void)
{
    return  directionUp;
}

bool    Encoder_reset(void)
{
    countA = 0;

    offsetA = GPTimerCC26XX_getValue(timerChannelA);

    return  true;
}

bool    Encoder_setReverse(bool _reverse)
{
    reverse_ = _reverse;

    return  true;
}

bool    Encoder_getRecord(ENCODER_RECORD* record, bool _clear)
{
    if (encoder_.head == encoder_.tail)
    {
        return  false;
    }

    memcpy(record, &encoder_.records[encoder_.tail], sizeof(ENCODER_RECORD));
    if (_clear)
    {
        encoder_.tail = (encoder_.tail + 1) % 20;
    }

    return  true;
}

bool        Encoder_setScale(uint32_t scale)
{
    if ((scale == 0) || (2000 < scale))
    {
        return  false;
    }

    match_ = scale;

    GPTimerCC26XX_setMatchValue(timerChannelA, match_);
    GPTimerCC26XX_setLoadValue(timerChannelA, match_*2);

    return  true;
}
