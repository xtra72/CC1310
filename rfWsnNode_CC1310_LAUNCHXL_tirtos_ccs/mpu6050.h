/*
 * mpu6050.h
 *
 *  Created on: 2019. 6. 8.
 *      Author: xtra7
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_VALUE_ACCL_X    0x3B
#define MPU6050_VALUE_ACCL_Y    0x3D
#define MPU6050_VALUE_ACCL_Z    0x3F
#define MPU6050_VALUE_TEMP      0x41
#define MPU6050_VALUE_GYRO_X    0x43
#define MPU6050_VALUE_GYRO_Y    0x45
#define MPU6050_VALUE_GYRO_Z    0x47

#define MPU6050_INT_CONFIG      0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A

#define MPU6050_USER_CTRL       0x6A

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C

#define MPU6050_FIFO_CTRL       0x23
#define MPU6050_I2C_MST_CTRL    0x24
#define MPU6050_FIFO_COUNT      0x72
#define MPU6050_FIFO_VALUE      0x74

#define MPU6050_PWR_MGMT_RESET  (1 << 7)
#define MPU6050_PWR_MGMT_SLEEP  (1 << 6)
#define MPU6050_PWR_MGMT_CYCLE  (1 << 5)
#define MPU6050_PWR_MGMT_TEMP_DISABLE   (1 << 3)

#define MPU6050_PWR_MGMT_LP_WAKE_CTRL_MASK  (3 << 6)
#define MPU6050_PWR_MGMT_LP_WAKE_CTRL_0     (0 << 6)
#define MPU6050_PWR_MGMT_LP_WAKE_CTRL_1     (1 << 6)
#define MPU6050_PWR_MGMT_LP_WAKE_CTRL_2     (2 << 6)
#define MPU6050_PWR_MGMT_LP_WAKE_CTRL_3     (3 << 6)
#define MPU6050_PWR_MGMT_STBY_ACCL_X        (1 << 5)
#define MPU6050_PWR_MGMT_STBY_ACCL_Y        (1 << 4)
#define MPU6050_PWR_MGMT_STBY_ACCL_Z        (1 << 3)
#define MPU6050_PWR_MGMT_STBY_GYLO_X        (1 << 2)
#define MPU6050_PWR_MGMT_STBY_GYLO_Y        (1 << 1)
#define MPU6050_PWR_MGMT_STBY_GYLO_Z        (1 << 0)

#define MPU6050_INT_CONFIG_LEVEL_LOW        (1 << 7)
#define MPU6050_INT_CONFIG_OPEN_DRAIN       (1 << 6)
#define MPU6050_INT_CONFIG_LATCH            (1 << 5)

#define MPU6050_INT_ENABLE_FIFO_OVERFLOW    (1 << 4)

#define MPU6050_FIFO_CTRL_TEMP_EN       (1 << 7)
#define MPU6050_FIFO_CTRL_GYLO_X        (1 << 6)
#define MPU6050_FIFO_CTRL_GYLO_Y        (1 << 5)
#define MPU6050_FIFO_CTRL_GYLO_Z        (1 << 4)
#define MPU6050_FIFO_CTRL_ACCL          (1 << 3)

#define MPU6050_USER_CTRL_FIFO_EN       (1 << 6)
#define MPU6050_USER_CTRL_FIFO_RESET    (1 << 2)

bool    MPU6050_init(void);
void    MPU6050_final(void);

bool    MPU6050_readValue(uint8_t type, double* value);
bool    MPU6050_startMotionDetect(float amplitude, bool _async);


#endif /* MPU6050_H_ */
