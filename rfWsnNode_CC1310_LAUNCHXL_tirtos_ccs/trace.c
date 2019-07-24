/*
 * trace.c
 *
 *  Created on: 2019. 5. 26.
 *      Author: xtra7
 */
#include <ti/sysbios/knl/Clock.h>
#include <ti/display/Display.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

/* Display driver handles */
Display_Handle hDisplaySerial = NULL;

#include <ti/drivers/dpl/DebugP.h>

void    Trace_init(void)
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
}



void  Trace_printf(char *fmt, ...)
{
    static  char    newFmt[512];

    if (NULL == hDisplaySerial)
    {
        DebugP_log0("Trying to use NULL-handle.");
        return;
    }

    uint32_t    currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000;

    uint32_t    len = sprintf(newFmt, "[%4d.%03d] : ", currentTransferTime / 1000, currentTransferTime % 1000);
    strcpy(&newFmt[len], fmt);

    va_list va;
    va_start(va, fmt);

    hDisplaySerial->fxnTablePtr->vprintfFxn(hDisplaySerial, 0, 0, newFmt, va);

    va_end(va);
}

