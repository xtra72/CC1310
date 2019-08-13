/*
 * encoder.h
 *
 *  Created on: 2019. 8. 11.
 *      Author: xtra7
 */

#ifndef ENCODER_H_
#define ENCODER_H_


void    Encoder_init(void);

void    Encoder_start(void);
void    Encoder_stop(void);

void    Encoder_setCount(uint32_t count);
uint32_t    Encoder_getCount(void);
bool        Encoder_isUp(void);
bool        Encoder_reset(void);
bool        Encoder_setReverse(bool _reverse);

#endif /* ENCODER_H_ */
