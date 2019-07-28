/*
 * rf.h
 *
 *  Created on: 2019. 7. 7.
 *      Author: xtra7
 */

#ifndef RF_H_
#define RF_H_


#define RF_CMD                              0x00
#define RF_REQ_PING                         (RF_CMD + 3)

#define RF_REPLY_TO_SERVER                  (0x20)
#define RF_REP_SRV_MOTION_DETECTION_START   (RF_REPLY_TO_SERVER + 1)
#define RF_REP_SRV_MOTION_DETECTION_STOP    (RF_REPLY_TO_SERVER + 2)
#define RF_REP_SRV_SCAN_START               (RF_REPLY_TO_SERVER + 3)
#define RF_REP_SRV_SCAN_STOP                (RF_REPLY_TO_SERVER + 4)
#define RF_REP_SRV_TRANSFER_START           (RF_REPLY_TO_SERVER + 5)
#define RF_REP_SRV_TRANSFER_STOP            (RF_REPLY_TO_SERVER + 6)
#define RF_REP_SRV_DATA_COUNT               (RF_REPLY_TO_SERVER + 7)

#define RF_REPLY                            (0x80)
#define RF_REP_ACK                          (RF_REPLY + 1)
#define RF_REP_NACK                         (RF_REPLY + 2)
#define RF_REP_PING                         (RF_REPLY + 3)
#define RF_DOWNLINK                         (RF_REPLY + 5)

#define RF_NOTIFY                           (0x90)
#define RF_NOTI_MOTION_DETECTED             (RF_NOTIFY + 1)

#define RF_REQUEST_FROM_SERVER              (0xA0)
#define RF_REQ_SRV_MOTION_DETECTION_START   (RF_REQUEST_FROM_SERVER + 1)
#define RF_REQ_SRV_MOTION_DETECTION_STOP    (RF_REQUEST_FROM_SERVER + 2)
#define RF_REQ_SRV_SCAN_START               (RF_REQUEST_FROM_SERVER + 3)
#define RF_REQ_SRV_SCAN_STOP                (RF_REQUEST_FROM_SERVER + 4)
#define RF_REQ_SRV_TRANSFER_START           (RF_REQUEST_FROM_SERVER + 5)
#define RF_REQ_SRV_TRANSFER_STOP            (RF_REQUEST_FROM_SERVER + 6)
#define RF_REQ_SRV_REQ_DATA_COUNT           (RF_REQUEST_FROM_SERVER + 7)
#define RF_REQ_SRV_SLEEP                    (RF_REQUEST_FROM_SERVER + 8)

#endif /* RF_H_ */
