#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Types.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Log.h>
#include <Board.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

GPTimerCC26XX_Handle    timerChannelA;
GPTimerCC26XX_Handle    timerChannelB;
PIN_Handle              timerPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State        timerPinState;

int32_t    countA = 0;
int32_t     offsetA = 0;
bool        directionUp = true;
bool        reverse_ = false;

PIN_Config GptPinInitTable[] = {
    IOID_22   | PIN_INPUT_EN | PIN_PULLUP,
    IOID_24   | PIN_INPUT_EN | PIN_PULLUP,
    PIN_TERMINATE
};

void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    if ((!reverse_ && (PIN_getInputValue(IOID_22) == PIN_getInputValue(IOID_24))) || (reverse_ && (PIN_getInputValue(IOID_22) != PIN_getInputValue(IOID_24))))
    {
        countA++;
        if (countA == 0)
        {
            offsetA = 0;
        }
        directionUp = true;
    }
    else
    {
        if (countA >= 0)
        {
            countA--;
        }
        directionUp = false;
    }
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
    timerParams.width          = GPT_CFG_CFG_16BIT_TIMER;
    timerParams.mode           = GPT_MODE_EDGE_COUNT_UP;
    timerChannelA = GPTimerCC26XX_open(Board_GPTIMER0A, &timerParams);
    if(timerChannelA == NULL)
    {
      Log_error0("Failed to open GPTimer");
      Task_exit();
    }

    timerChannelB = GPTimerCC26XX_open(Board_GPTIMER1A, &timerParams);
    if(timerChannelB == NULL)
    {
      Log_error0("Failed to open GPTimer");
      Task_exit();
    }
    GPTimerCC26XX_setCaptureEdge(timerChannelA, GPTimerCC26XX_POS_EDGE);
    GPTimerCC26XX_setCaptureEdge(timerChannelB, GPTimerCC26XX_POS_EDGE);

    /* Register interrupt when capture happens */
    GPTimerCC26XX_registerInterrupt(timerChannelA, timerCallback, GPT_INT_CAPTURE_MATCH);

    /* Open pin handle and route pin to timer */
    timerPinHandle = PIN_open(&timerPinState, GptPinInitTable);

    pinMux = GPTimerCC26XX_getPinMux(timerChannelA);
    PINCC26XX_setMux(timerPinHandle, PIN_ID(IOID_22), pinMux);
    pinMux = GPTimerCC26XX_getPinMux(timerChannelB);
    PINCC26XX_setMux(timerPinHandle, PIN_ID(IOID_24), pinMux);

    GPTimerCC26XX_setMatchValue(timerChannelA, 100);

    GPTimerCC26XX_setLoadValue(timerChannelA, 200);
    GPTimerCC26XX_setLoadValue(timerChannelB, 200);
}

void    Encoder_setCount(uint32_t count)
{
    countA = count / 100;
    offsetA = GPTimerCC26XX_getValue(timerChannelA) - (count % 100);
}

uint32_t    Encoder_getCount(void)
{
    if (timerChannelA)
    {
        if (countA >= 0)
        {
            if ((countA * 100) +  GPTimerCC26XX_getValue(timerChannelA) - offsetA > 0)
            {
                return (countA * 100) +  GPTimerCC26XX_getValue(timerChannelA) - offsetA;
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

