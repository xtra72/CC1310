/*
 * SpiSlave.c
 *
 *  Created on: 2019. 5. 27.
 *      Author: xtra7
 */

/*
 *  ======== spislave.c ========
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
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"
#include "Trace.h"
#include "crc16.h"
#include "NodeTask.h"
#include "SpiSlave.h"
#include "DataQueue.h"
#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (80)
#define MAX_LOOP        (10)

typedef struct
{
    uint8_t     cmd;
    uint8_t     len;
    uint16_t    crc;
    uint8_t     result;
    uint8_t     status;
    uint8_t     qsize;
    uint8_t     reserved;
    uint8_t     payload[SPI_MSG_LENGTH - 8];
}   SPI_Frame;

typedef union
{
    SPI_Frame   frame;
    uint8_t     raw[sizeof(SPI_Frame)];
}   SPI_Buffer;

static  uint8_t commandForMaster = 0;

bool SPI_isValidFrame(SPI_Frame* frame);

extern  Display_Handle hDisplaySerial;

/* Pin driver handle */
#if 0
static PIN_Handle slaveStatusHandle;
static PIN_State slaveStatusState;
PIN_Config slaveStatusTable[] =
{
     CC1310_LAUNCHXL_PIN_SPI_SLAVE_READY | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
     CC1310_LAUNCHXL_PIN_SPI_SLAVE_DATA_ON | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
     PIN_TERMINATE
};
#endif

SPI_Buffer  rxBuffer;
SPI_Buffer  txBuffer;

/* Semaphore to block slave until transfer is complete */
static  sem_t slaveSem;

static  bool    stop_ = true;
/*
 *  ======== transferCompleteFxn ========
 *  Callback function for SPI_transfer().
 */
void transferCompleteFxn(SPI_Handle handle, SPI_Transaction *transaction)
{
    sem_post(&slaveSem);
}

/*
 * ======== slaveThread ========
 *  Slave SPI sends a message to master while simultaneously receiving a
 *  message from the master.
 */
void *slaveThread(void *arg0)
{
    SPI_Handle      slaveSpi;
    SPI_Params      spiParams;
    SPI_Transaction transaction;
    bool            transferOK;
    int32_t         status;

    /*
     * Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY are GPIO pins connected
     * between the master & slave.  These pins are used to synchronize
     * the master & slave applications via a small 'handshake'.  The pins
     * are later used to synchronize transfers & ensure the master will not
     * start a transfer until the slave is ready.  These pins behave
     * differently between spimaster & spislave examples:
     *
     * spislave example:
     *     * Board_SPI_MASTER_READY is configured as an input pin.  During the
     *       'handshake' this pin is read & a high value will indicate the
     *       master is ready to run the application.  Afterwards, the pin is
     *       read to determine if the master has already opened its SPI pins.
     *       The master will pull this pin low when it has opened its SPI.
     *
     *     * Board_SPI_SLAVE_READY is configured as an output pin.  During the
     *       'handshake' this pin is changed from low to high output.  This
     *       notifies the master the slave is ready to run the application.
     *       Afterwards, the pin is used by the slave to notify the master it
     *       is ready for a transfer.  When ready for a transfer, this pin will
     *       be pulled low.
     *
     * Below we set Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY initial
     * conditions for the 'handshake'.
     */
    GPIO_setConfig(Board_SPI_SLAVE_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH);
    GPIO_setConfig(Board_SPI_MASTER_READY, GPIO_CFG_INPUT);
    /* Open LED pins */
#if 0
    slaveStatusHandle = PIN_open(&slaveStatusState, slaveStatusTable);
    if (!slaveStatusHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }
#endif
    /*
     * Handshake - Set Board_SPI_SLAVE_READY high to indicate slave is ready
     * to run.  Wait for Board_SPI_MASTER_READY to be high.
     */
    GPIO_write(Board_SPI_SLAVE_READY, 1);
//    while (GPIO_read(Board_SPI_MASTER_READY) == 0) {}

    /*
     * Create synchronization semaphore; this semaphore will block the slave
     * until a transfer is complete.  The slave is configured in callback mode
     * to allow us to configure the SPI transfer & then notify the master the
     * slave is ready.  However, we must still wait for the current transfer
     * to be complete before setting up the next.  Thus, we wait on slaveSem;
     * once the transfer is complete the callback function will unblock the
     * slave.
     */
    status = sem_init(&slaveSem, 0, 0);
    if (status != 0)
    {
        Trace_printf(hDisplaySerial, "Error creating slaveSem\n");

        while(1);
    }

    /*
     * Wait until master SPI is open.  When the master is configuring SPI pins
     * the clock may toggle from low to high (or high to low depending on
     * polarity).  If using 3-pin SPI & the slave has been opened before the
     * master, clock transitions may cause the slave to shift bits out assuming
     * it is an actual transfer.  We can prevent this behavior by opening the
     * master first & then opening the slave.
     */
    //while (GPIO_read(Board_SPI_MASTER_READY)) {}

    /*
     * Open SPI as slave in callback mode; callback mode is used to allow us to
     * configure the transfer & then set Board_SPI_SLAVE_READY high.
     */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL1_PHA1;
    spiParams.mode = SPI_SLAVE;
    spiParams.transferCallbackFxn = transferCompleteFxn;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    slaveSpi = SPI_open(Board_SPI_SLAVE, &spiParams);
    if (slaveSpi == NULL)
    {
        Trace_printf(hDisplaySerial, "Error initializing slave SPI\n");
        while (1);
    }
    else
    {
        Trace_printf(hDisplaySerial, "Slave SPI initialized\n");
    }

    Trace_printf(hDisplaySerial, "Sizeof (SPI_Frame) = %d\n", sizeof(SPI_Frame));
    /* Copy message to transmit buffer */

    memset(txBuffer.raw, 0, sizeof(txBuffer));

    stop_ = false;
    while(!stop_)
    {
        /* Initialize slave SPI transaction structure */
        transaction.count = sizeof(txBuffer);
        transaction.txBuf = (void *) txBuffer.raw;
        transaction.rxBuf = (void *) rxBuffer.raw;

        transferOK = SPI_transfer(slaveSpi, &transaction);
        if (transferOK)
        {
//            while(GPIO_read(Board_SPI_MASTER_READY))
//            {
//                CPUdelay(10);
//            }

            Trace_printf(hDisplaySerial, "Waiting transfer");
//            PIN_setOutputValue(slaveStatusHandle, CC1310_LAUNCHXL_PIN_SPI_SLAVE_READY, 0);
            GPIO_write(Board_SPI_SLAVE_READY, 0);

           /* Wait until transfer has completed */
            sem_wait(&slaveSem);

//            while(!GPIO_read(Board_SPI_MASTER_READY))
//            {
//                CPUdelay(10);
//            }

//            PIN_setOutputValue(slaveStatusHandle, CC1310_LAUNCHXL_PIN_SPI_SLAVE_READY, 1);
            GPIO_write(Board_SPI_SLAVE_READY, 1);
            if (SPI_isValidFrame(&rxBuffer.frame))
            {
                memset(txBuffer.raw, 0, sizeof(txBuffer.raw));

                switch(rxBuffer.frame.cmd)
                {
                case    RF_IO_TX_DATA:
                    {
                        if (!NodeTask_postTransfer(rxBuffer.frame.payload, rxBuffer.frame.len))
                        {
                            Trace_printf(hDisplaySerial, "Data Tranfer Failed!");
                        }
                    }
                    break;

                case    RF_IO_REQ_GET_CONFIG:
                    {
                        Trace_printf(hDisplaySerial, "Request get RF config!");

                        NODETASK_CONFIG   config;

                        NodeTask_getConfig(&config);

                        txBuffer.frame.cmd = RF_IO_REP_GET_CONFIG;
                        txBuffer.frame.len = sizeof(NODETASK_CONFIG);
                        memcpy(txBuffer.frame.payload, &config, sizeof(NODETASK_CONFIG));
                    }
                    break;

                case    RF_IO_REQ_SET_CONFIG:
                    {
                        Trace_printf(hDisplaySerial, "Request set RF config!");

                        if (rxBuffer.frame.len == sizeof(NODETASK_CONFIG))
                        {
                            if (NodeTask_setConfig((NODETASK_CONFIG *)rxBuffer.frame.payload))
                            {
                                NODETASK_CONFIG   config;

                                NodeTask_getConfig(&config);

                                txBuffer.frame.cmd = RF_IO_REP_SET_CONFIG;
                                txBuffer.frame.len = sizeof(NODETASK_CONFIG);
                                memcpy(txBuffer.frame.payload, &config, sizeof(NODETASK_CONFIG));
                            }
                            else
                            {
                                txBuffer.frame.cmd = RF_IO_REP_SET_CONFIG;
                                txBuffer.frame.len = 1;
                                txBuffer.frame.payload[0] = 1;
                            }
                        }
                    }
                    break;

                case    RF_IO_REQ_MOTION_DETECT_START:
                    {
                        if (NodeTask_motionDetectionStart())
                        {
                            txBuffer.frame.cmd = RF_IO_REP_MOTION_DETECT_STARTED;
                        }
                    }
                    break;

                case    RF_IO_REQ_MOTION_DETECT_STOP:
                    {
                        if (NodeTask_motionDetectionStop())
                        {
                            txBuffer.frame.cmd = RF_IO_REP_MOTION_DETECT_STOPPED;
                        }
                    }
                    break;

                case    RF_IO_KEEP_ALIVE:
                    {
                        if (!NodeTask_postTransfer(NULL, 0))
                        {
                            Trace_printf(hDisplaySerial, "Data Tranfer Failed!");
                        }
                    }
                    break;

                default:
                    {
                        Trace_printf(hDisplaySerial, "Unknown ata received : cmd - %d, len = %d", rxBuffer.frame.cmd, rxBuffer.frame.len);
                    }
                }

                if (txBuffer.frame.cmd == 0)
                {
                    if  (commandForMaster != 0)
                    {
                        txBuffer.frame.cmd = RF_IO_NOTI_FROM_SERVER;
                        txBuffer.frame.len = sizeof(RF_SPI_REQ_FROM_SLAVE_PARAMS);
                        ((RF_SPI_REQ_FROM_SLAVE_PARAMS *)txBuffer.frame.payload)->cmd = commandForMaster;
                        commandForMaster = 0;
                    }
                }
                else
                {
                    txBuffer.frame.cmd = RF_IO_STATUS;
                    txBuffer.frame.len = 0;
                }
                txBuffer.frame.qsize = DataQ_maxCount() - DataQ_count();

                if (txBuffer.frame.len != 0)
                {
                    txBuffer.frame.crc =   CRC16_calc(txBuffer.frame.payload, txBuffer.frame.len);
                }
            }
            else
            {
                Trace_printf(hDisplaySerial, "Invalid frame");
            }
        }
        else
        {
            Trace_printf(hDisplaySerial, "Unsuccessful slave SPI transfer");
        }

    }

    SPI_close(slaveSpi);

    /* Example complete - set pins to a known state */
    GPIO_setConfig(Board_SPI_MASTER_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_write(Board_SPI_SLAVE_READY, 0);

    Trace_printf(hDisplaySerial, "\nDone");

    return (NULL);
}

bool SPI_isValidFrame(SPI_Frame* frame)
{
    if (frame->len > 60)
    {
        return  false;
    }

    uint16_t crc = CRC16_calc(frame->payload, frame->len);
    if (crc != frame->crc)
    {
        Trace_printf(hDisplaySerial, "CRC invalid![%02x, %d, %04x]\n", frame->cmd, frame->len, frame->crc);
        return  false;
    }

    return  true;
}
/*
 *  ======== mainThread ========
 */
bool    SpiSlave_init(void)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    SPI_init();

    /* Create application thread */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0)
    {
        return  false;
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0)
    {
        return  false;
    }

    /* Create slave thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, slaveThread, NULL);
    if (retc != 0)
    {
        return  false;
    }

    return true;
}

bool    SpiSlave_setNotification(uint8_t cmd)
{
    commandForMaster = cmd;

    return  true;
}
