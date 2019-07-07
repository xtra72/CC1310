/*
 * rf.h
 *
 *  Created on: 2019. 7. 7.
 *      Author: xtra7
 */

#ifndef RF_H_
#define RF_H_

#define RF_SPI_CMD_DATA_TRANSFER            0x01

#define RF_SPI_CMD_GET_CONFIG               0x41
#define RF_SPI_CMD_SET_CONFIG               0x42

#define RF_SPI_CMD_START_AUTO_TRANSFER      0x81
#define RF_SPI_CMD_STOP_AUTO_TRANSFER       0x82
#define RF_SPI_CMD_START_MOTION_DETECTION   0x83
#define RF_SPI_CMD_STOP_MOTION_DETECTION    0x84

#define RF_SPI_CMD_DUMMY                    0x5A

#endif /* RF_H_ */
