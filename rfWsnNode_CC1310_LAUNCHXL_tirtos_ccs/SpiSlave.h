/*
 * SpiSlave.h
 *
 *  Created on: 2019. 5. 27.
 *      Author: xtra7
 */

#ifndef SPISLAVE_H_
#define SPISLAVE_H_


#define RF_IO_TX_DATA                   0x01

#define RF_IO_REQ_GET_CONFIG            0x11
#define RF_IO_REQ_SET_CONFIG            0x12

#define RF_IO_REQ_MOTION_DETECT_START   0x21
#define RF_IO_REQ_MOTION_DETECT_STOP    0x22
#define RF_IO_REQ_SLEEP                 0x23

#define RF_IO_KEEP_ALIVE                0x6F

#define RF_IO_REP_ACK                   0x81
#define RF_IO_REP_NACK                  0x82

#define RF_IO_REP_GET_CONFIG            0x91
#define RF_IO_REP_SET_CONFIG            0x92

#define RF_IO_REP_MOTION_DETECT_STARTED 0xA1
#define RF_IO_REP_MOTION_DETECT_STOPPED 0xA2
#define RF_IO_NOTI_MOTION_DETECTED      0xA3
#define RF_IO_NOTI_FROM_SERVER          0xA4

#define RF_IO_STATUS                    0xEF

typedef struct
{
    uint32_t    mode;
    uint32_t    time;
}   RF_SPI_REQ_MOTION_DETECT_PARAMS;

typedef struct
{
    uint32_t    cmd;
}   RF_SPI_REQ_FROM_SLAVE_PARAMS;


bool    SpiSlave_init(void);

bool    SpiSlave_setCommand(uint8_t cmd);
bool    SpiSlave_setNotification(uint8_t cmd);
#endif /* SPISLAVE_H_ */
