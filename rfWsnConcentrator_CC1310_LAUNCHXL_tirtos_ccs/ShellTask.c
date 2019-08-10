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
#include <strings.h>

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


#define SHELL_TASK_BUFFER_SIZE  256
#define SHELL_EVENT_ALL                      0xFFFFFFFF
#define SHELL_EVENT_DATA_IN                  (uint32_t)(1 << 0)
#define SHELL_EVENT_NEW_RAW_DATA             (uint32_t)(1 << 1)
#define SHELL_EVENT_TEST_RESET               (uint32_t)(1 << 2)
#define SHELL_EVENT_ERROR                    (uint32_t)(1 << 3)


/***** Prototypes *****/
static void ShellTask_main(UArg arg0, UArg arg1);

void    ShellTask_readCallback(UART_Handle handle, void *buf, size_t count);
void    ShellTask_writeCallback(UART_Handle handle, void *buf, size_t count);

void    ShellTask_output(char *fmt, ...);
bool    ShellTask_write(char *_string, uint32_t _length, uint32_t _timeout);

/***** Variable declarations *****/
static  Task_Struct ShellTask;    /* not static so you can see in ROV */
static  Task_Struct ShellIOTask;    /* not static so you can see in ROV */

static  UART_Handle uart;
static  uint8_t     rxBuffer_[SHELL_TASK_BUFFER_SIZE + 1];
static  uint32_t    rxLen_ = 0;
static  uint8_t     txBuffer_[SHELL_TASK_BUFFER_SIZE + 1];
static  uint32_t    txLen_ = 0;
static  bool        echo_ = false;

static  DataQ       dataQ_;
static  Semaphore_Struct    readSem_;  /* not static so you can see in ROV */
static  Semaphore_Handle    readSemHandle_;
static  Semaphore_Struct    writeSem_;  /* not static so you can see in ROV */
static  Semaphore_Handle    writeSemHandle_;

static  uint32_t            commandCount_ = 0;
static  ShellTaskCommand    *commandList_ = NULL;

/***** Function definitions *****/
void ShellTask_init(ShellTaskCommand _commandList[], uint32_t _count)
{
    static  uint8_t stack[SHELL_TASK_STACK_SIZE];
//    static  uint8_t IOStack[SHELL_IO_TASK_STACK_SIZE];

    commandList_ = _commandList;
    commandCount_ = _count;

    Task_Params     params;
    /* Create the SHELL radio protocol task */
    Task_Params_init(&params);
    params.stackSize = SHELL_TASK_STACK_SIZE;
    params.priority = SHELL_TASK_PRIORITY;
    params.stack = &stack;
    Task_construct(&ShellTask, ShellTask_main, &params, NULL);

    /* Create the SHELL radio protocol task */
#if 0
    Task_Params_init(&params);
    params.stackSize = SHELL_IO_TASK_STACK_SIZE;
    params.priority = SHELL_IO_TASK_PRIORITY;
    params.stack = &IOStack;
    Task_construct(&ShellIOTask, ShellIOTask_main, &params, NULL);
#endif
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
    uartParams.readEcho = UART_ECHO_ON;
    uartParams.readCallback = ShellTask_readCallback;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&readSem_, 1, &semParam);
    readSemHandle_ = Semaphore_handle(&readSem_);

    Semaphore_Params_init(&semParam);
    Semaphore_construct(&writeSem_, 1, &semParam);
    writeSemHandle_ = Semaphore_handle(&writeSem_);

}

static void ShellTask_main(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open UART types of display. */

    /* Enter main task loop */
    while(1)
    {
        txLen_ = 0;
        if (Semaphore_pend(readSemHandle_, 1))
        {
            static  char    rxBuffer;
            UART_read(uart, &rxBuffer, 1);
        }

        if (DataQ_pop(&dataQ_, txBuffer_, sizeof(txBuffer_), &txLen_, 100))
        {
            char* token = strtok((char *)txBuffer_, "+");
            if ((token != NULL) && (strcasecmp(token, "AT") == 0))
            {
                int i;
                char* cmd = strtok(NULL, ":");
                if (cmd != NULL)
                {
                    bool    ret = false;
                    char*   argv[8];
                    int     argc = 0;

                    argv[argc++] = cmd;
                    while((token = strtok(NULL, ",")) != NULL)
                    {
                        argv[argc++] = token;
                    }

                    for(i = 0 ; i < commandCount_ ; i++)
                    {
                        if (strcasecmp(cmd, commandList_[i].name) == 0)
                        {
                            ret = commandList_[i].command(argc, argv);
                            break;
                        }
                    }

                    if (!ret)
                    {
                        ShellTask_output("+%s:ERR\n",cmd);
                    }
                }
            }
        }
    }
}


#if 0
static void ShellIOTask_main(UArg arg0, UArg arg1)
{
    static  char    rxBuffer;

    /* Enter main task loop */
    while(1)
    {
        /* Get access semaphore */
         Semaphore_pend(readSemHandle_, BIOS_WAIT_FOREVER);

         UART_read(uart, &rxBuffer, 1);
    }
}
#endif

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
        else if (isprint(ch[i]))
        {
            rxBuffer_[rxLen_++] = ch[i];
        }

        if (echo_)
        {
            Semaphore_pend(writeSemHandle_, BIOS_WAIT_FOREVER);
            UART_write(uart, &ch[i], 1);
        }
    }

    Semaphore_post(readSemHandle_);
}


void ShellTask_writeCallback(UART_Handle handle, void *buf, size_t count)
{
    Semaphore_post(writeSemHandle_);
}

static  char    newFmt[512];


void  ShellTask_output(char *fmt, ...)
{
    if (NULL != uart)
    {
        Semaphore_pend(writeSemHandle_, BIOS_WAIT_FOREVER);
        va_list va;
        va_start(va, fmt);

        uint32_t len = vsnprintf(newFmt, sizeof(newFmt), fmt, va);
        UART_write(uart, newFmt, len);

        va_end(va);
    }
}

bool    ShellTask_write(char *_string, uint32_t _length, uint32_t _timeout)
{
    Semaphore_pend(writeSemHandle_, _timeout);
    UART_write(uart, _string, _length);

    return  true;
}

static  bool    trace_ = false;

void  Trace_enable(void)
{
    trace_ = true;
}

void  Trace_disable(void)
{
    trace_ = false;
}

bool  Trace_isEnable(void)
{
    return  trace_;
}

void  Trace_printf(char *fmt, ...)
{
    if (trace_ && (NULL != uart))
    {
        Semaphore_pend(writeSemHandle_, BIOS_WAIT_FOREVER);

        uint32_t    currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000;

        uint32_t    len = sprintf(newFmt, "+LOG:%4d.%03d,", currentTransferTime / 1000, currentTransferTime % 1000);

        va_list va;
        va_start(va, fmt);

        len += vsnprintf(&newFmt[len], sizeof(newFmt) - len, fmt, va);
        UART_write(uart, newFmt, len);

        va_end(va);
    }
}
