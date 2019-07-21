/*
 * MessageQueue.c
 *
 *  Created on: 2019. 5. 26.
 *      Author: xtra7
 */

#include "DataQueue.h"


void    DataQ_init(DataQ* _dataQ, uint32_t _size)
{
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&_dataQ->accessSem, 1, &semParam);
    _dataQ->accessSemHandle = Semaphore_handle(&_dataQ->accessSem);

    Semaphore_construct(&_dataQ->dataSem, 0, &semParam);
    _dataQ->dataSemHandle = Semaphore_handle(&_dataQ->dataSem);

    _dataQ->headIndex = 0;
    _dataQ->tailIndex = 0;

    if ((_size != 0) && (_size < DATA_QUEUE_MAX_SIZE))
    {
        _dataQ->poolSize = _size;
    }
    else
    {
        _dataQ->poolSize = DATA_QUEUE_MAX_SIZE;
    }
}

uint32_t    DataQ_count(DataQ* _dataQ)
{
    uint32_t    count = 0;

    /* Get access semaphore */
    Semaphore_pend(_dataQ->accessSemHandle, BIOS_WAIT_FOREVER);

    if (_dataQ->headIndex <= _dataQ->tailIndex)
    {
        count = _dataQ->tailIndex - _dataQ->headIndex;
    }
    else
    {
        count = (_dataQ->poolSize + _dataQ->tailIndex - _dataQ->headIndex);
    }

    /* Return access semaphore */
    Semaphore_post(_dataQ->accessSemHandle);

    return  count;
}

bool    DataQ_push(DataQ* _dataQ, uint8_t* data, uint32_t length)
{
    if (_dataQ->poolSize - 1 <= DataQ_count(_dataQ))
    {
        return  false;
    }

    if (sizeof(_dataQ->pool[0].data) < length)
    {
        return  false;
    }

    /* Get access semaphore */
    Semaphore_pend(_dataQ->accessSemHandle, BIOS_WAIT_FOREVER);

    _dataQ->pool[_dataQ->tailIndex].length = length;
    memcpy(_dataQ->pool[_dataQ->tailIndex].data, data, length);
    _dataQ->tailIndex = (_dataQ->tailIndex + 1) % _dataQ->poolSize;

    /* Return access semaphore */
    Semaphore_post(_dataQ->accessSemHandle);

    Semaphore_post(_dataQ->dataSemHandle);

    return  true;
}

bool    DataQ_pop(DataQ* _dataQ, uint8_t* buffer, uint32_t maxLength, uint32_t* length, uint32_t _timeout)
{
    bool    ret = true;

    if (!Semaphore_pend(_dataQ->dataSemHandle, _timeout))
    {
        return  false;
    }

    /* Get access semaphore */
    Semaphore_pend(_dataQ->accessSemHandle, BIOS_WAIT_FOREVER);

    if (!buffer)
    {
        _dataQ->headIndex = (_dataQ->headIndex + 1) % _dataQ->poolSize;
    }
    else
    {
        if (maxLength < _dataQ->pool[_dataQ->headIndex].length)
        {
            Semaphore_post(_dataQ->dataSemHandle);
            ret = false;
        }
        else
        {
            *length = _dataQ->pool[_dataQ->headIndex].length;
            memcpy(buffer, _dataQ->pool[_dataQ->headIndex].data, _dataQ->pool[_dataQ->headIndex].length);
            _dataQ->headIndex = (_dataQ->headIndex + 1) % _dataQ->poolSize;
        }
    }

    Semaphore_post(_dataQ->accessSemHandle);

    return  ret;
}


bool    DataQ_front(DataQ* _dataQ, uint8_t* buffer, uint32_t maxLength, uint32_t* length, uint32_t _timeout)
{
    bool    ret = true;

    if (!Semaphore_pend(_dataQ->dataSemHandle, _timeout))
    {
        return  false;
    }

    /* Get access semaphore */
    Semaphore_pend(_dataQ->accessSemHandle, BIOS_WAIT_FOREVER);

    if (maxLength < _dataQ->pool[_dataQ->headIndex].length)
    {
        ret = false;
    }
    else
    {
        *length = _dataQ->pool[_dataQ->headIndex].length;
        memcpy(buffer, _dataQ->pool[_dataQ->headIndex].data, _dataQ->pool[_dataQ->headIndex].length);
    }
    Semaphore_post(_dataQ->dataSemHandle);
    Semaphore_post(_dataQ->accessSemHandle);

    return  ret;
}
