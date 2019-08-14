/*
 * encoder.h
 *
 *  Created on: 2019. 8. 11.
 *      Author: xtra7
 */

#ifndef ENCODER_H_
#define ENCODER_H_


typedef struct
{
    uint32_t    time;
    int32_t    value[10];
}   ENCODER_RECORD;

typedef struct
{
    uint32_t    head;
    uint32_t    tail;
    uint32_t    index;
    ENCODER_RECORD records[20];
}   ENCODER;

void    Encoder_init(void);

void    Encoder_start(void);
void    Encoder_stop(void);

void    Encoder_setCount(int32_t count);
int32_t    Encoder_getCount(void);

bool        Encoder_setScale(uint32_t scale);

bool        Encoder_isUp(void);
bool        Encoder_reset(void);
bool        Encoder_setReverse(bool _reverse);
bool        Encoder_getRecord(ENCODER_RECORD* record, bool _clear);

#endif /* ENCODER_H_ */
