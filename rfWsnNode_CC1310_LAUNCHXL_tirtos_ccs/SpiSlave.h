/*
 * SpiSlave.h
 *
 *  Created on: 2019. 5. 27.
 *      Author: xtra7
 */

#ifndef SPISLAVE_H_
#define SPISLAVE_H_


bool    SpiSlave_init(void);

bool    SpiSlave_setCommandForMaster(uint8_t cmd);
#endif /* SPISLAVE_H_ */
