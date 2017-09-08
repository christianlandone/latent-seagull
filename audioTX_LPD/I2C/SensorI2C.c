#include <unistd.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#include "Board.h"
#include "SensorUtil.h"
#include "SensorI2C.h"


#define I2C_TIMEOUT 500


static I2C_Handle i2cHandle;
static I2C_Params i2cParams;
static Semaphore_Struct mutex;
static const I2CCC26XX_I2CPinCfg pinCfg1 =
{
    // Pin configuration for I2C interface 1
    .pinSDA = Board_I2C0_SDA0,
    .pinSCL = Board_I2C0_SCL0
};

/* Module state */
static volatile uint8_t interface;
static volatile uint8_t slaveAddr;
static uint8_t buffer[32];

bool SensorI2C_write(uint8_t *data, uint8_t len)
{
    I2C_Transaction masterTransaction;

    masterTransaction.writeCount   = len;
    masterTransaction.writeBuf     = data;
    masterTransaction.readCount    = 0;
    masterTransaction.readBuf      = NULL;
    masterTransaction.slaveAddress = slaveAddr;

    return I2C_transfer(i2cHandle, &masterTransaction) == TRUE;
}


bool SensorI2C_writeSingle(uint8_t data)
{
    uint8_t d;

    d = data;

    return SensorI2C_write(&d, 1);
}


bool SensorI2C_read(uint8_t *data, uint8_t len)
{
    I2C_Transaction masterTransaction;

    masterTransaction.writeCount = 0;
    masterTransaction.writeBuf = NULL;
    masterTransaction.readCount = len;
    masterTransaction.readBuf = data;
    masterTransaction.slaveAddress = slaveAddr;

    return I2C_transfer(i2cHandle, &masterTransaction) == TRUE;
}


bool SensorI2C_writeRead(uint8_t *wdata, uint8_t wlen, uint8_t *rdata, uint8_t rlen)
{
    I2C_Transaction masterTransaction;

    masterTransaction.writeCount = wlen;
    masterTransaction.writeBuf = wdata;
    masterTransaction.readCount = rlen;
    masterTransaction.readBuf = rdata;
    masterTransaction.slaveAddress = slaveAddr;

    return I2C_transfer(i2cHandle, &masterTransaction) == TRUE;
}


bool SensorI2C_readReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
    usleep(1000);
    return SensorI2C_writeRead(&addr,1,pBuf,nBytes);
}


bool SensorI2C_writeReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
    uint8_t i;
    uint8_t *p = buffer;

    /* Copy address and data to local buffer for burst write */
    *p++ = addr;
    for (i = 0; i < nBytes; i++)
    {
        *p++ = *pBuf++;
    }
    nBytes++;

    usleep(1000);
    /* Send data */
    return SensorI2C_write(buffer,nBytes);
}

/*******************************************************************************
* @fn          SensorI2C_select
*
* @brief       Select an I2C interface and slave
*
* @param       newInterface - selected interface
* @param       address - slave address
*
* @return      true if success
*/
bool SensorI2C_select(uint8_t newInterface, uint8_t address)
{
    // Acquire I2C resource
    if (!Semaphore_pend(Semaphore_handle(&mutex),MS_2_TICKS(I2C_TIMEOUT)))
    {
        return false;
    }

    // Store new slave address
    slaveAddr = address;

    // Interface changed ?
    if (newInterface != interface)
    {
        // Store new interface
        interface = newInterface;

        // Shut down RTOS driver
        I2C_close(i2cHandle);

        // Sets custom to NULL, selects I2C interface 0
        I2C_Params_init(&i2cParams);

        // Assign I2C data/clock pins according to selected I2C interface 1
        if (interface == SENSOR_I2C_1)
        {
            i2cParams.custom = (void *)&pinCfg1;
        }

        // Re-open RTOS driver with new bus pin assignment
        i2cHandle = I2C_open(Board_I2C0, &i2cParams);
    }

    return i2cHandle != NULL;
}

/*******************************************************************************
* @fn          SensorI2C_deselect
*
* @brief       Allow other tasks to access the I2C driver
*
* @param       none
*
* @return      none
*/
void SensorI2C_deselect(void)
{
    // Release I2C resource
    Semaphore_post(Semaphore_handle(&mutex));
}

/*******************************************************************************
* @fn          SensorI2C_open
*
* @brief       Initialize the RTOS I2C driver (must be called only once)
*
* @param       none
*
* @return      true if I2C_open succeeds
*/
bool SensorI2C_open(void)
{
    Semaphore_Params semParamsMutex;

    // Create protection semaphore
    Semaphore_Params_init(&semParamsMutex);
    semParamsMutex.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&mutex, 1, &semParamsMutex);

    // Initialize I2C bus
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2cHandle = I2C_open(Board_I2C0, &i2cParams);

    // Initialize local variables
    slaveAddr = 0xFF;
    interface = SENSOR_I2C_0;

    return i2cHandle != NULL;
}

/*******************************************************************************
* @fn          SensorI2C_close
*
* @brief       Close the I2C interface and release the data lines
*
* @param       none
*
* @return      true if I2C_open succeeds
*/
void SensorI2C_close(void)
{
    if (i2cHandle != NULL)
    {
        I2C_close(i2cHandle);
    }
}
