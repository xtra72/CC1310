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

#include <ti/drivers/dpl/DebugP.h>

void  Trace_printf(Display_Handle handle, char *fmt, ...)
{
    static  char    newFmt[512];

    if (NULL == handle)
    {
        DebugP_log0("Trying to use NULL-handle.");
        return;
    }

    uint32_t    currentTransferTime = (Clock_getTicks() * Clock_tickPeriod) / 1000;

    uint32_t    len = sprintf(newFmt, "[%4d.%03d] : ", currentTransferTime / 1000, currentTransferTime % 1000);
    strcpy(&newFmt[len], fmt);

    va_list va;
    va_start(va, fmt);

    handle->fxnTablePtr->vprintfFxn(handle, 0, 0, newFmt, va);

    va_end(va);
}

