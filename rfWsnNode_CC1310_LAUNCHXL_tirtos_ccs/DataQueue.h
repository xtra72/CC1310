/*
 * DataQueue.h
 *
 *  Created on: 2019. 5. 26.
 *      Author: xtra72
 */

#ifndef DATAQUEUE_H_
#define DATAQUEUE_H_

#include <stdint.h>

struct DataQueueElement {
    uint8_t length;
    uint8_t data[128];
};

void        DataQ_init(uint32_t size);
uint32_t    DataQ_count(void);
bool        DataQ_push(uint8_t* data, uint32_t length);
bool        DataQ_pop(uint8_t* buffer, uint32_t maxLength, uint32_t* length);
bool        DataQ_front(uint8_t* buffer, uint32_t maxLength, uint32_t* length);

#endif /* DATAQUEUE_H_ */
