/*
 * DataQueue.h
 *
 *  Created on: 2019. 5. 26.
 *      Author: xtra7
 */

#ifndef DATAQUEUE_H_
#define DATAQUEUE_H_

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <stdbool.h>

#include <stdint.h>

#define DATA_QUEUE_MAX_SIZE 4
#define DATA_QUEUE_MAX_DATA_SIZE 64

typedef struct
{
    uint8_t length;
    uint8_t data[DATA_QUEUE_MAX_DATA_SIZE];
}   DataQItem;

typedef struct
{
    Semaphore_Struct    accessSem;  /* not static so you can see in ROV */
    Semaphore_Handle    accessSemHandle;

    Semaphore_Struct    dataSem;  /* not static so you can see in ROV */
    Semaphore_Handle    dataSemHandle;

    DataQItem           pool[DATA_QUEUE_MAX_SIZE];
    uint32_t            headIndex;
    uint32_t            tailIndex;
    uint32_t            poolSize;

}   DataQ;

void        DataQ_init(DataQ* _dataQ, uint32_t size);
uint32_t    DataQ_count(DataQ* _dataQ);
DataQItem*  DataQ_front(DataQ* _dataQ);
DataQItem*  DataQ_rear(DataQ* _dataQ);
bool        DataQ_push(DataQ* _dataQ, uint8_t* data, uint32_t length);
bool        DataQ_pop(DataQ* _dataQ, uint8_t* buffer, uint32_t maxLength, uint32_t* length, uint32_t _timeout);

bool    DataQ_lock(DataQ* _dataQ, uint32_t _timeout);
void    DataQ_unlock(DataQ* _dataQ);

DataQItem*  DataQ_lazyPushBegin(DataQ* _dataQ);
void        DataQ_lazyPushEnd(DataQ* _dataQ, bool _cancel);

#endif /* DATAQUEUE_H_ */
