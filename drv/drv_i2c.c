#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"

#include "drv_i2c.h"

#define  BUFFER_SIZE   64U

i2c_master_handle_t g_m_handle;
i2c_master_transfer_t masterXfer;

volatile bool g_MasterCompletionFlag = false;
volatile bool nakFlag = false;

static uint8_t g_i2cTxBuff[BUFFER_SIZE];
static uint8_t g_i2cRxBuff[BUFFER_SIZE];

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }

    if (status == kStatus_I2C_Nak || status == kStatus_I2C_Addr_Nak)
    {
    	nakFlag = true;
    }
}

#if 0
void i2c_pins_init(void)
{
    const uint32_t port0_pin26_config = (
        IOCON_PIO_FUNC1 |               /* Pin is configured as FC2_RXD_SDA_MOSI_DATA */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    const uint32_t port0_pin27_config = (
        IOCON_PIO_FUNC1 |               /* Pin is configured as FC2_TXD_SCL_MISO_WS */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    CLOCK_EnableClock(kCLOCK_Iocon);

    IOCON_PinMuxSet(IOCON, 0U, 26U, port0_pin26_config);
    IOCON_PinMuxSet(IOCON, 0U, 27U, port0_pin27_config);
}
#endif

void i2c_init(void)
{
	i2c_master_config_t i2c_config = {0};

	/* Init GPIOs */
//    i2c_pins_init();
    
    /* attach 12 MHz clock to SPI2 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);
	/*
	 * masterConfig.baudRate_Bps = 100000U;
	 * masterConfig.enableStopHold = false;
	 * masterConfig.glitchFilterWidth = 0U;
	 * masterConfig.enableMaster = true;
	 */
    I2C_MasterGetDefaultConfig(&i2c_config);

    i2c_config.baudRate_Bps = I2C_BAUDRATE;
    i2c_config.enableMaster = true;

    I2C_MasterInit(EXAMPLE_I2C_MASTER, &i2c_config, I2C_MASTER_CLOCK_FREQUENCY);


    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER, &g_m_handle, i2c_master_callback, NULL);
}

uint8_t i2c_write_register(uint8_t reg_addr, uint8_t* value, uint32_t len)
{
	status_t reVal        = kStatus_Fail;

	memcpy(g_i2cTxBuff, value, len);

    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = g_i2cTxBuff;
    masterXfer.dataSize = len;
    masterXfer.flags = kI2C_TransferDefaultFlag;
	I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
	/*  Wait for transfer completed. */
	while (!g_MasterCompletionFlag && !nakFlag)
	{
	}
	reVal = (nakFlag == false);
    g_MasterCompletionFlag = false;
    nakFlag = false;

	return reVal;
}

uint8_t i2c_read_registers(uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	status_t reVal        = kStatus_Fail;

	masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = g_i2cRxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;
	I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
	/*  Wait for transfer completed. */
	while (!g_MasterCompletionFlag && !nakFlag)
	{
	}

	g_MasterCompletionFlag = false;
	nakFlag = false;
	reVal = kStatus_Success;
	memcpy(rxBuff, g_i2cRxBuff, rxSize);

	return kStatus_Success;
}

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
