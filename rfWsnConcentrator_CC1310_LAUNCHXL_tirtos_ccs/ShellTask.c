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
#include <ti/drivers/UART.h>

#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "ConcentratorTask.h"
#include "ShellTask.h"
#include "RadioProtocol.h"

#include "DataQueue.h"
#include "Trace.h"


/***** Defines *****/
#define SHELL_TASK_STACK_SIZE 512
#define SHELL_TASK_PRIORITY   3

#define SHELL_IO_TASK_STACK_SIZE 256
#define SHELL_IO_TASK_PRIORITY   3


#define SHELL_TASK_BUFFER_SIZE  512
#define SHELL_EVENT_ALL                      0xFFFFFFFF
#define SHELL_EVENT_DATA_IN                  (uint32_t)(1 << 0)
#define SHELL_EVENT_NEW_RAW_DATA             (uint32_t)(1 << 1)
#define SHELL_EVENT_TEST_RESET               (uint32_t)(1 << 2)
#define SHELL_EVENT_ERROR                    (uint32_t)(1 << 3)


/***** Variable declarations *****/
static  Task_Struct ShellTask;    /* not static so you can see in ROV */
static  Task_Struct ShellIOTask;    /* not static so you can see in ROV */

static  UART_Handle uart;
static  uint8_t     rxBuffer_[SHELL_TASK_BUFFER_SIZE + 1];
static  uint32_t    rxLen_ = 0;
static  uint8_t     txBuffer_[SHELL_TASK_BUFFER_SIZE + 1];
static  uint32_t    txLen_ = 0;
static  bool        echo_ = true;

static  DataQ       dataQ_;
static  Semaphore_Struct    readSem;  /* not static so you can see in ROV */
static  Semaphore_Handle    readSemHandle;
static  Semaphore_Struct    writeSem;  /* not static so you can see in ROV */
static  Semaphore_Handle    writeSemHandle;



/***** Prototypes *****/
static void ShellTask_main(UArg arg0, UArg arg1);
static void ShellIOTask_main(UArg arg0, UArg arg1);

void    ShellTask_readCallback(UART_Handle handle, void *buf, size_t count);
void    ShellTask_writeCallback(UART_Handle handle, void *buf, size_t count);

void    ShellTask_output(char *fmt, ...);
bool    ShellTask_write(char *_string, uint32_t _length, uint32_t _timeout);

/***** Function definitions *****/
void ShellTask_init(void)
{
    static  uint8_t stack[SHELL_TASK_STACK_SIZE];
    static  uint8_t IOStack[SHELL_IO_TASK_STACK_SIZE];

    Task_Params     params;
    /* Create the SHELL radio protocol task */
    Task_Params_init(&params);
    params.stackSize = SHELL_TASK_STACK_SIZE;
    params.priority = SHELL_TASK_PRIORITY;
    params.stack = &stack;
    Task_construct(&ShellTask, ShellTask_main, &params, NULL);

    /* Create the SHELL radio protocol task */
    Task_Params_init(&params);
    params.stackSize = SHELL_IO_TASK_STACK_SIZE;
    params.priority = SHELL_IO_TASK_PRIORITY;
    params.stack = &IOStack;
    Task_construct(&ShellIOTask, ShellIOTask_main, &params, NULL);

    DataQ_init(&dataQ_, 4);

    /* Create a UART with data processing off. */
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeMode = UART_MODE_CALLBACK;
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.writeCallback = ShellTask_writeCallback;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readCallback = ShellTask_readCallback;
    uartParams.baudRate = 921600;

    uart = UART_open(Board_UART0, &uartParams);

    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&readSem, 1, &semParam);
    readSemHandle = Semaphore_handle(&readSem);

    Semaphore_Params_init(&semParam);
    Semaphore_construct(&writeSem, 1, &semParam);
    writeSemHandle = Semaphore_handle(&writeSem);

}

static void ShellTask_main(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open UART types of display. */

    /* Enter main task loop */
    while(1)
    {
        txLen_ = 0;
        if (DataQ_pop(&dataQ_, txBuffer_, sizeof(txBuffer_), &txLen_, 1000))
        {
            char* token = strtok((char *)txBuffer_, "+");
            if ((token != NULL) && (strcasecmp(token, "AT") == 0))
            {
                char* cmd = strtok(NULL, ":");
                if (cmd != NULL)
                {
                    bool    ret = false;

                    if (strcasecmp(cmd, "start") == 0)
                    {
                        token = strtok(NULL, "");
                        if (token != NULL)
                        {
                            uint16_t device_id = (uint16_t)strtoul(token, NULL, 10);

                            ret = ConcentratorTask_sendCommand(device_id, CONCENTRATOR_COMMAND_DEVICE_START);
                        }
                    }
                    else if (strcasecmp(cmd, "stop") == 0)
                    {
                        token = strtok(NULL, "");
                        if (token != NULL)
                        {
                            uint16_t device_id = (uint16_t)strtoul(token, NULL, 10);

                            ret = ConcentratorTask_sendCommand(device_id, CONCENTRATOR_COMMAND_DEVICE_STOP);
                        }
                    }

                    if (ret)
                    {
                        ShellTask_output("+%s:OK\n",cmd);
                    }
                    else
                    {
                        ShellTask_output("+%s:ERR\n",cmd);
                    }
                }
            }
        }
    }
}



static void ShellIOTask_main(UArg arg0, UArg arg1)
{
    static  char    rxBuffer;

    /* Enter main task loop */
    while(1)
    {
        /* Get access semaphore */
         Semaphore_pend(readSemHandle, BIOS_WAIT_FOREVER);

         UART_read(uart, &rxBuffer, 1);
    }
}

void ShellTask_readCallback(UART_Handle handle, void *buf, size_t count)
{
    int32_t i;

    if (SHELL_TASK_BUFFER_SIZE < rxLen_ + count)
    {
        count = SHELL_TASK_BUFFER_SIZE - rxLen_;
    }

    char *ch = (char *)buf;

    for(i = 0 ; i < count ; i++)
    {
        if ((ch[i] == '\n') || (ch[i] == '\r'))
        {
            if (rxLen_ != 0)
            {
                rxBuffer_[rxLen_++] = '\0';
                DataQ_push(&dataQ_, rxBuffer_, rxLen_);
                rxLen_=0;
            }
        }
        else
        {
            rxBuffer_[rxLen_++] = ch[i];
        }

        if (echo_)
        {
            Semaphore_pend(writeSemHandle, BIOS_WAIT_FOREVER);
            UART_write(uart, &ch[i], 1);
        }
    }

    Semaphore_post(readSemHandle);
}


void ShellTask_writeCallback(UART_Handle handle, void *buf, size_t count)
{
    Semaphore_post(writeSemHandle);
}

static  char    newFmt[512];


void  ShellTask_output(char *fmt, ...)
{
    if (NULL != uart)
    {
        Semaphore_pend(writeSemHandle, BIOS_WAIT_FOREVER);
        va_list va;
        va_start(va, fmt);

        uint32_t len = vsnprintf(newFmt, sizeof(newFmt), fmt, va);
        UART_write(uart, newFmt, len);

        va_end(va);
    }
}

bool    ShellTask_write(char *_string, uint32_t _length, uint32_t _timeout)
{
    Semaphore_pend(writeSemHandle, _timeout);
    UART_write(uart, _string, _length);

    return  true;
}

/*
 * By default disable both asserts and log for this module.
 * This must be done before DebugP.h is included.
 */
#ifndef DebugP_ASSERT_ENABLED
#define DebugP_ASSERT_ENABLED 0
#endif
#ifndef DebugP_LOG_ENABLED
#define DebugP_LOG_ENABLED 0
#endif

#include <ti/drivers/dpl/DebugP.h>

void  Trace_printf(char *fmt, ...)
{
    if (NULL != uart)
    {
        Semaphore_pend(writeSemHandle, BIOS_WAIT_FOREVER);

        uint32_t    currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000;

        uint32_t    len = sprintf(newFmt, "+LOG:%4d.%03d,", currentTransferTime / 1000, currentTransferTime % 1000);
        strcpy(&newFmt[len], fmt);

        va_list va;
        va_start(va, fmt);

        len += vsnprintf(&newFmt[len], sizeof(newFmt) - len, fmt, va);
        UART_write(uart, newFmt, len);

        va_end(va);
    }
}

