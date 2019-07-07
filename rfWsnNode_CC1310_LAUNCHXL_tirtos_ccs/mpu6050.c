/*
 * MPU6050.c
 *
 *  Created on: 2019. 5. 27.
 *      Author: xtra7
 */

/*
 *  ======== MPU6050.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"
#include "mpu6050.h"
#include "Trace.h"
#include "NodeTask.h"

#define MPU6050_BUFFER_LENGTH_MAX   128

#define WAKEUP  0

/* Pin driver handles */
static PIN_Handle interruptHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State interruptPinState;

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config interruptPinTable[] = {
#if WAKEUP
    CC1310_LAUNCHXL_DIO1  | PIN_INPUT_EN | PIN_PULLDOWN | PINCC26XX_WAKEUP_POSEDGE,//PIN_IRQ_POSEDGE,
#else
    CC1310_LAUNCHXL_DIO1  | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
#endif
    PIN_TERMINATE
};

extern  Display_Handle hDisplaySerial;

static  bool            stop_ = false;
static  uint8_t         slaveId_ = 0x68;
static  I2C_Handle      i2c_;
static  uint8_t         txBuffer[MPU6050_BUFFER_LENGTH_MAX];
static  uint8_t         rxBuffer[MPU6050_BUFFER_LENGTH_MAX];
static  I2C_Transaction i2cTransaction;

bool    MPU6050_write8(uint8_t address, uint8_t value);
bool    MPU6050_write16(uint8_t address, uint16_t value);
void    MPU6050_showRegister(uint8_t address, char *name);
float    MPU6050_getOscillationValue(void);

/* Semaphore to block slave until transfer is complete */
sem_t lock_;
sem_t fifoFull_;

/*
 *  ======== buttonCallbackFxn ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 *  If Board_PIN_LED3 and Board_PIN_LED4 are defined, then we'll add them to the PIN
 *  callback function.
 */
void MPU6050_interruptCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == CC1310_LAUNCHXL_DIO1)
    {
//        currVal =  PIN_getInputValue(pinId);

//        Trace_printf(hDisplaySerial, "Interrupt occurred.");
        PIN_setInterrupt(interruptHandle, PIN_IRQ_DIS);

        sem_post(&fifoFull_);
    }
}

bool    MPU6050_write8(uint8_t address, uint8_t value)
{
    bool    ret = true;

    sem_wait(&lock_);

    ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
    ((uint8_t *)i2cTransaction.writeBuf)[1] = value;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;

    if (!I2C_transfer(i2c_, &i2cTransaction))
    {
        sem_post(&lock_);
        Trace_printf(hDisplaySerial, "I2C Bus fault.");
        ret = false;
    }
    else
    {
        ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readCount = 1;

        if (!I2C_transfer(i2c_, &i2cTransaction))
        {
            sem_post(&lock_);
            Trace_printf(hDisplaySerial, "I2C Bus fault.");
            ret = false;
        }
        else
        {
#if 0
            Trace_printf(hDisplaySerial, "%02x : %d|%d|%d|%d|%d|%d|%d|%d",  ((uint8_t *)i2cTransaction.readBuf)[0],
                 (((uint8_t *)i2cTransaction.readBuf)[0] >> 7) & 1, (((uint8_t *)i2cTransaction.readBuf)[0] >> 6) & 1, (((uint8_t *)i2cTransaction.readBuf)[0] >> 5) & 1, (((uint8_t *)i2cTransaction.readBuf)[0] >> 4) & 1,
                 (((uint8_t *)i2cTransaction.readBuf)[0] >> 3) & 1, (((uint8_t *)i2cTransaction.readBuf)[0] >> 2) & 1, (((uint8_t *)i2cTransaction.readBuf)[0] >> 1) & 1, (((uint8_t *)i2cTransaction.readBuf)[0] >> 0) & 1);
#endif
        }
    }

    sem_post(&lock_);

    return  ret;
}

bool    MPU6050_read8(uint8_t address, uint8_t *value)
{
    bool    ret = true;

    sem_wait(&lock_);

    ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount = 1;

    if (!I2C_transfer(i2c_, &i2cTransaction))
    {
        Trace_printf(hDisplaySerial, "I2C Bus fault.");
        ret = false;
    }
    else
    {
        *value = ((uint8_t *)i2cTransaction.readBuf)[0];
    }

    sem_post(&lock_);

    return  ret;
}

bool    MPU6050_write16(uint8_t address, uint16_t value)
{
    bool    ret = true;

    sem_wait(&lock_);

    ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
    ((uint8_t *)i2cTransaction.writeBuf)[1] = (value >> 8) & 0xFF;
    ((uint8_t *)i2cTransaction.writeBuf)[2] = value & 0xFF;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readCount = 0;

    if (!I2C_transfer(i2c_, &i2cTransaction))
    {
        Trace_printf(hDisplaySerial, "I2C Bus fault.");
        ret = false;
    }

    sem_post(&lock_);

    return  ret;
}

bool    MPU6050_read16(uint8_t address, uint16_t *value)
{
    bool    ret = true;

    sem_wait(&lock_);

    ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount = 2;

    if (!I2C_transfer(i2c_, &i2cTransaction))
    {
        Trace_printf(hDisplaySerial, "I2C Bus fault.");
        ret = false;
    }
    else
    {
        *value = (((uint16_t)((uint8_t *)i2cTransaction.readBuf)[0] << 8) | ((uint8_t *)i2cTransaction.readBuf)[1]);
    }

    sem_post(&lock_);

    return  ret;
}

bool    MPU6050_writeArray(uint8_t address, uint8_t *values, uint8_t length)
{
    bool    ret = true;

    sem_wait(&lock_);

    ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
    memcpy(&((uint8_t *)i2cTransaction.writeBuf)[1], values, length);
    i2cTransaction.writeCount = length+1;
    i2cTransaction.readCount = 0;

    if (!I2C_transfer(i2c_, &i2cTransaction))
    {
        sem_post(&lock_);
        Trace_printf(hDisplaySerial, "I2C Bus fault.");
        ret = false;
    }

    sem_post(&lock_);

    return  ret;
}

bool    MPU6050_readArray(uint8_t address, uint8_t *values, uint8_t length)
{
    bool    ret = true;

    sem_wait(&lock_);

    ((uint8_t *)i2cTransaction.writeBuf)[0] = address;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readCount = length;

    if (!I2C_transfer(i2c_, &i2cTransaction))
    {
        Trace_printf(hDisplaySerial, "I2C Bus fault.");
        ret = false;
    }
    else
    {
        memcpy(values, i2cTransaction.readBuf, length);
    }

    sem_post(&lock_);

    return  ret;
}

bool    MPU6050_readValue(uint8_t type, double* value)
{
    int16_t tempValue;

    switch(type)
    {
    case    MPU6050_VALUE_ACCL_X:
        if (!MPU6050_read16(0x3B, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 16384.0;
        break;

    case    MPU6050_VALUE_ACCL_Y:
        if (!MPU6050_read16(0x3D, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 16384.0;
        break;

    case    MPU6050_VALUE_ACCL_Z:
        if (!MPU6050_read16(0x3F, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 16384.0;
        break;

    case    MPU6050_VALUE_TEMP:
        if (!MPU6050_read16(0x41, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 340.0 + 136.53;
        break;

    case    MPU6050_VALUE_GYRO_X:
        if (!MPU6050_read16(0x43, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 131.0;
        break;

    case    MPU6050_VALUE_GYRO_Y:
        if (!MPU6050_read16(0x45, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 131.0;
        break;

    case    MPU6050_VALUE_GYRO_Z:
        if (!MPU6050_read16(0x47, (uint16_t *)&tempValue))
        {
            return  false;
        }
        *value = tempValue / 131.0;
        break;

    default:
        return  false;
    }

    return  true;
}


bool    MPU6050_fifoCount(uint16_t* count)
{
    uint8_t value;

    if (!MPU6050_read8(MPU6050_FIFO_COUNT, &value))
    {
        return  false;
    }
    *count = value;

    if (!MPU6050_read8(MPU6050_FIFO_COUNT + 1, &value))
    {
        return  false;
    }

    *count = ((*count) << 8) | value;

    return  true;
}

bool    MPU6050_popValue(uint8_t type, double* value)
{
    uint16_t value16;

#if 0
    uint8_t *value8 = (uint8_t *)&value16;

    if (!MPU6050_read8(MPU6050_FIFO_VALUE, &value8[1]))
    {
        return  false;
    }

    if (!MPU6050_read8(MPU6050_FIFO_VALUE, &value8[0]))
    {
        return  false;
    }
#else
    if (!MPU6050_read16(MPU6050_FIFO_VALUE, &value16))
    {
        return  false;
    }

#endif

    switch(type)
    {
    case    MPU6050_VALUE_ACCL_X:
    case    MPU6050_VALUE_ACCL_Y:
    case    MPU6050_VALUE_ACCL_Z:
        *value = value16 / 16384.0;
        break;

    case    MPU6050_VALUE_TEMP:
        *value = value16 / 340.0 + 136.53;
        break;

    case    MPU6050_VALUE_GYRO_X:
    case    MPU6050_VALUE_GYRO_Y:
    case    MPU6050_VALUE_GYRO_Z:
        *value = value16 / 131.0;
        break;

    default:
        return  false;
    }

    return  true;
}

/*
 * ======== MPU6050_thread ========
 */

bool    MPU6050_startMotionDetection(float _limit)
{
    Trace_printf(hDisplaySerial, "Start Motion Detection");
    MPU6050_write8(MPU6050_FIFO_CTRL, MPU6050_FIFO_CTRL_ACCL);
    MPU6050_write8(MPU6050_INT_ENABLE, MPU6050_INT_ENABLE_FIFO_OVERFLOW);
    MPU6050_write8(MPU6050_USER_CTRL, MPU6050_USER_CTRL_FIFO_RESET);
    CPUdelay(10000*12);
    MPU6050_write8(MPU6050_USER_CTRL, MPU6050_USER_CTRL_FIFO_EN);
    MPU6050_write8(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_CYCLE | MPU6050_PWR_MGMT_TEMP_DISABLE);
    MPU6050_write8(MPU6050_PWR_MGMT_2, MPU6050_PWR_MGMT_LP_WAKE_CTRL_3 | MPU6050_PWR_MGMT_STBY_ACCL_X | MPU6050_PWR_MGMT_STBY_ACCL_Y | MPU6050_PWR_MGMT_STBY_GYLO_X | MPU6050_PWR_MGMT_STBY_GYLO_Y | MPU6050_PWR_MGMT_STBY_GYLO_Z);

#if WAKEUP
    PINCC26XX_setWakeup(interruptPinTable);

    /* Go to shutdown */
    Power_shutdown(0, 0);

#else
    PIN_setInterrupt(interruptHandle, PIN_IRQ_POSEDGE);
    sem_wait(&fifoFull_);
    PIN_setInterrupt(interruptHandle, PIN_IRQ_DIS);
#endif

    MPU6050_write8(MPU6050_INT_ENABLE, 0);
    MPU6050_write8(MPU6050_PWR_MGMT_2, MPU6050_PWR_MGMT_LP_WAKE_CTRL_3 | MPU6050_PWR_MGMT_STBY_ACCL_X | MPU6050_PWR_MGMT_STBY_ACCL_Y | MPU6050_PWR_MGMT_STBY_ACCL_Z | MPU6050_PWR_MGMT_STBY_GYLO_X | MPU6050_PWR_MGMT_STBY_GYLO_Y | MPU6050_PWR_MGMT_STBY_GYLO_Z);
    MPU6050_write8(MPU6050_PWR_MGMT_1,  MPU6050_PWR_MGMT_TEMP_DISABLE);

    return  MPU6050_getOscillationValue() > _limit;
}

void    MPU6050_showRegister(uint8_t address, char *name)
{
    uint8_t value8;

    if (MPU6050_read8(MPU6050_INT_STATUS, &value8))
    {
        Trace_printf(hDisplaySerial, "%s : %d|%d|%d|%d|%d|%d|%d|%d", name,
             (value8 >> 7) & 1, (value8 >> 6) & 1, (value8 >> 5) & 1, (value8 >> 4) & 1,
             (value8 >> 3) & 1, (value8 >> 2) & 1, (value8 >> 1) & 1, (value8 >> 0) & 1);
    }
}

float    MPU6050_getOscillationValue(void)
{
    int i;
    float min = 10;
    float max = 0;
    uint16_t    fifoCount = 0;

    if (!MPU6050_fifoCount(&fifoCount))
    {
        Trace_printf(hDisplaySerial, "Failed to get fifo count.");
        return  0;
    }

    for(i = 0 ;  i < fifoCount / 2; i++)
    {
        double  value;

        MPU6050_popValue(MPU6050_VALUE_ACCL_Z, &value);
        if (value != 0)
        {
            if (value < min)
                min = value;

            if (value > max)
                max = value;
        }
    }
    Trace_printf(hDisplaySerial, "Min, Max : %f, %f", min, max);

    return  (max - min);
}

/*
 *  ======== mainThread ========
 */
bool    MPU6050_init(void)
{

    I2C_Params      i2cParams;
    int32_t         status;

    I2C_init();

    i2cTransaction.slaveAddress = slaveId_;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;

    if (sem_init(&lock_, 0, 1) != 0)
    {
        Trace_printf(hDisplaySerial, "Error creating lock_\n");
        return  false;
    }

    if (sem_init(&fifoFull_, 0, 0) != 0)
    {
        Trace_printf(hDisplaySerial, "Error creating fifoFull_\n");
        return  false;
    }

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    i2c_ = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c_ == NULL)
    {
        Trace_printf(hDisplaySerial, "Error Initializing I2C\n");
        return  false;
    }

    interruptHandle = PIN_open(&interruptPinState, interruptPinTable);
    if(!interruptHandle)
    {
        /* Error initializing button pins */
        return  false;
    }

    if (PIN_registerIntCb(interruptHandle, &MPU6050_interruptCallbackFxn) != 0)
    {
        /* Error registering button callback function */
        return  false;
    }

    MPU6050_write8(MPU6050_PWR_MGMT_1, 0x80);
    CPUdelay(10000*12);
    MPU6050_write8(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_TEMP_DISABLE);
    MPU6050_write8(MPU6050_PWR_MGMT_2, MPU6050_PWR_MGMT_LP_WAKE_CTRL_3 | MPU6050_PWR_MGMT_STBY_ACCL_X | MPU6050_PWR_MGMT_STBY_ACCL_Y | MPU6050_PWR_MGMT_STBY_ACCL_Z | MPU6050_PWR_MGMT_STBY_GYLO_X | MPU6050_PWR_MGMT_STBY_GYLO_Y | MPU6050_PWR_MGMT_STBY_GYLO_Z);

    return   true;
}
