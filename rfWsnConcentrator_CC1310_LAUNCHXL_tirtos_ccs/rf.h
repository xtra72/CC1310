/*
 * rf.h
 *
 *  Created on: 2019. 7. 7.
 *      Author: xtra7
 */

#ifndef RF_H_
#define RF_H_

#define RF_REQ_HOST_SET_CONFIG              (RF_REQUEST_FROM_HOST + 1)
#define RF_REQ_HOST_GET_CONFIG              (RF_REQUEST_FROM_HOST + 2)
#define RF_REQ_HOST_MOTION_DETECTION_START  (RF_REQUEST_FROM_HOST + 3)
#define RF_REQ_HOST_MOTION_DETECTION_STOP   (RF_REQUEST_FROM_HOST + 4)

#define RF_REPLY_TO_SERVER                  (0x20)
#define RF_REP_HOST_DATA_COUNT               (RF_REPLY_TO_SERVER + 7)

#define RF_NOTI_TO_SERVER                   (0x30)
#define RF_NOTI_HOST_MOTION_DETECTED        (RF_NOTI_TO_SERVER + 1)

#define RF_REQUEST_FROM_SERVER              (0xA0)
#define RF_REQ_SRV_MOTION_DETECTION_START   (RF_REQUEST_FROM_SERVER + 1)
#define RF_REQ_SRV_MOTION_DETECTION_STOP    (RF_REQUEST_FROM_SERVER + 2)
#define RF_REQ_SRV_SCAN_START               (RF_REQUEST_FROM_SERVER + 3)
#define RF_REQ_SRV_SCAN_STOP                (RF_REQUEST_FROM_SERVER + 4)
#define RF_REQ_SRV_TRANSFER_START           (RF_REQUEST_FROM_SERVER + 5)
#define RF_REQ_SRV_TRANSFER_STOP            (RF_REQUEST_FROM_SERVER + 6)
#define RF_REQ_SRV_REQ_DATA_COUNT           (RF_REQUEST_FROM_SERVER + 7)
#define RF_REQ_SRV_SLEEP                    (RF_REQUEST_FROM_SERVER + 8)
#define RF_REQ_SRV_SCAN_TRANS_START         (RF_REQUEST_FROM_SERVER + 9)
#define RF_REQ_SRV_SCAN_TRANS_STOP          (RF_REQUEST_FROM_SERVER + 10)


#endif /* RF_H_ */