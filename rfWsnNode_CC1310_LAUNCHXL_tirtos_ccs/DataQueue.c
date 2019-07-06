/*
 * MessageQueue.c
 *
 *  Created on: 2019. 5. 26.
 *      Author: xtra7
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <stdbool.h>
#include "DataQueue.h"

#define DATA_QUEUE_MAX_SIZE 16

static  Semaphore_Struct    accessSem;  /* not static so you can see in ROV */
static Semaphore_Handle     accessSemHandle;

static  struct  DataQueueElement    pool[DATA_QUEUE_MAX_SIZE];
static  uint32_t            headIndex = 0;
static  uint32_t            tailIndex = 0;
static  uint32_t            poolSize = DATA_QUEUE_MAX_SIZE;

void    DataQ_init(uint32_t size)
{
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&accessSem, 1, &semParam);
    accessSemHandle = Semaphore_handle(&accessSem);

    if (size < DATA_QUEUE_MAX_SIZE)
    {
        poolSize = size;
    }
}

uint32_t    DataQ_count(void)
{
    uint32_t    count = 0;

    /* Get access semaphore */
    Semaphore_pend(accessSemHandle, BIOS_WAIT_FOREVER);

    if (headIndex <= tailIndex)
    {
        count = tailIndex - headIndex;
    }
    else
    {
        count = (poolSize + tailIndex - headIndex);
    }

    /* Return access semaphore */
    Semaphore_post(accessSemHandle);

    return  count;
}

bool    DataQ_push(uint8_t* data, uint32_t length)
{
    if (poolSize - 1 <= DataQ_count())
    {
        return  false;
    }

    if (sizeof(pool[0].data) < length)
    {
        return  false;
    }

    /* Get access semaphore */
    Semaphore_pend(accessSemHandle, BIOS_WAIT_FOREVER);

    pool[tailIndex].length = length;
    memcpy(pool[tailIndex].data, data, length);
    tailIndex = (tailIndex + 1) % poolSize;

    /* Return access semaphore */
    Semaphore_post(accessSemHandle);

    return  true;
}

bool    DataQ_pop(uint8_t* buffer, uint32_t maxLength, uint32_t* length)
{
    if (DataQ_count() == 0)
    {
        return  false;
    }

    if ((maxLength != 0) && (maxLength < pool[headIndex].length))
    {
        return  false;
    }

    /* Get access semaphore */
    Semaphore_pend(accessSemHandle, BIOS_WAIT_FOREVER);

    if (length != NULL)
    {
        *length = pool[headIndex].length;
    }

    if (maxLength != 0)
    {
        memcpy(buffer, pool[headIndex].data, pool[headIndex].length);
    }
    headIndex = (headIndex + 1) % poolSize;

    /* Return access semaphore */
    Semaphore_post(accessSemHandle);

    return  true;
}

bool    DataQ_front(uint8_t* buffer, uint32_t maxLength, uint32_t* length)
{
    if (DataQ_count() == 0)
    {
        return  false;
    }

    if (maxLength < pool[headIndex].length)
    {
        return  false;
    }

    /* Get access semaphore */
    Semaphore_pend(accessSemHandle, BIOS_WAIT_FOREVER);

    *length = pool[headIndex].length;
    if ((buffer != NULL) && (maxLength != 0))
    {
        memcpy(buffer, pool[headIndex].data, pool[headIndex].length);
    }

    /* Return access semaphore */
    Semaphore_post(accessSemHandle);

    return  true;
}
