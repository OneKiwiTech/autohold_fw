#include "drv_spi.h"
#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"


void spi_pins_init(void)
{
    const uint32_t port0_pin0_config = (
        IOCON_PIO_FUNC2 |               /* Pin is configured as FC3_SCK */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    const uint32_t port0_pin2_config = (
        IOCON_PIO_FUNC1 |               /* Pin is configured as FC3_TXD_SCL_MISO_WS */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    const uint32_t port0_pin3_config = (
        IOCON_PIO_FUNC1 |               /* Pin is configured as FC3_RXD_SDA_MOSI_DATA */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    
//    IOCON_PinMuxSet(IOCON, 0U, 0U, port0_pin0_config);
//    IOCON_PinMuxSet(IOCON, 0U, 2U, port0_pin2_config);
//    IOCON_PinMuxSet(IOCON, 0U, 3U, port0_pin3_config);

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t spi_cs_config = {
        kGPIO_DigitalOutput, 1,
    };
    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO, SPI_CS_PORT, SPI_CS_PIN, &spi_cs_config);
    GPIO_PinWrite(GPIO, SPI_CS_PORT, SPI_CS_PIN, 1);
}

void spi_init(void)
{
    spi_master_config_t spi_config = {0};

    spi_pins_init();
    
    /* attach 12 MHz clock to SPI2 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

    /* Init SPI master */
    /*
     * masterConfig.enableLoopback = false;
     * masterConfig.polarity = flexSPI_ClockPolarityActiveHigh;
     * masterConfig.phase = flexSPI_ClockPhaseFirstEdge;
     * masterConfig.direction = flexSPI_MsbFirst;
     * masterConfig.baudRate_Bps = 500000U;
     * masterConfig.dataWidth = flexSPI_Data8Bits;
     * config->sselNum = flexSPI_Ssel0;
     */
    SPI_MasterGetDefaultConfig(&spi_config);
    spi_config.dataWidth = kSPI_Data8Bits;
    spi_config.baudRate_Bps = 5000000U;
    spi_config.direction = kSPI_MsbFirst;
    spi_config.polarity = kSPI_ClockPolarityActiveLow;
    spi_config.phase = kSPI_ClockPhaseSecondEdge;
    SPI_MasterInit(DRV_SPI, &spi_config, DRV_SPI_CLK_FREQ);
}

status_t spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint8_t size)
{
	spi_transfer_t spi_config = {0};

	spi_config.txData   = tx_data;
	spi_config.rxData   = rx_data;
	spi_config.dataSize = size;
	spi_config.configFlags = kSPI_FrameAssert;

	return SPI_MasterTransferBlocking(DRV_SPI, &spi_config);
}

uint8_t spi_write_register(uint8_t reg, uint8_t value)
{
	uint8_t tx_data[2] = {reg & 0x1f, value};

	SPI_CS_CLR();

	spi_transfer(tx_data, NULL, sizeof(tx_data));

	SPI_CS_SET();

	return 0x01U;
}

uint8_t spi_read_register(uint8_t reg)
{
	uint8_t rx_data = 0;
	uint8_t tx_data = reg | 0x80 ;

	SPI_CS_CLR();

	spi_transfer(&tx_data, NULL, 1);

	spi_transfer(NULL, &rx_data, 1);
    
	SPI_CS_SET();

	return rx_data;
}

/////

//RESET INSTRUCTION & REQUEST-TO-SEND (RTS) INSTRUCTION
uint8_t spi_can_rst(uint8_t instruction)
{
	uint8_t tx_data = instruction;
	SPI_CS_CLR();
	spi_transfer(&tx_data, NULL, 1);
	SPI_CS_SET();

	return 0x01U;
}

// READ INSTRUCTION
uint8_t spi_can_read(uint8_t instruction, uint8_t address)
{
	uint8_t rx_data = 0;
	SPI_CS_CLR();

	uint8_t tx_data[2] = {instruction, address};
	spi_transfer(tx_data, NULL, sizeof(tx_data));

	spi_transfer(NULL, &rx_data, 1);

	SPI_CS_SET();

	return rx_data;
}

// READ RX BUFFER INSTRUCTION
uint8_t spi_can_read_buffer(uint8_t instruction)
{
	uint8_t cmd = instruction;
	uint8_t rx_buf = 0;

	SPI_CS_CLR();

	spi_transfer(&cmd, NULL, 1);
	spi_transfer(NULL, &rx_buf, 1);

	SPI_CS_SET();
	return rx_buf;
}

// BYTE WRITE INSTRUCTION
uint8_t spi_can_write(uint8_t instruction, uint8_t address, uint8_t data)
{
	uint8_t tx_data[3] = {instruction, address, data};

    SPI_CS_CLR();
    
	spi_transfer(tx_data, NULL, sizeof(tx_data));

    SPI_CS_SET();

	return 0x01U;
}


// BIT MODIFY INSTRUCTION
uint8_t spi_can_bit_modify(uint8_t instruction, uint8_t address, uint8_t mask, uint8_t data)
{
	uint8_t tx_data[4] = {instruction, address, mask, data};

    SPI_CS_CLR();
    spi_transfer(tx_data, NULL, sizeof(tx_data));
    SPI_CS_SET();

	return 0x01U;
}

// READ STATUS INSTRUCTION & RX STATUS INSTRUCTION
uint8_t spi_can_status(uint8_t instruction)
{
	uint8_t tx_data = instruction;
	uint8_t rx_data = 0;

    SPI_CS_CLR();

    spi_transfer(&tx_data, NULL, 1);
    spi_transfer(NULL, &rx_data, 1);
    
    SPI_CS_SET();

	return rx_data;
}



// LOAD TX BUFFER INSTRUCTION
uint8_t spi_can_load_buffer(uint8_t instruction, uint8_t* data, uint8_t data_len)
{
    uint8_t tx_data  = instruction;
    SPI_CS_CLR();
    spi_transfer(&tx_data, NULL, 1);
    spi_transfer(data, NULL, data_len);
    SPI_CS_SET();

    return 0x01U;
}

uint8_t spi_can_read_rx_buffer(uint8_t instruction, uint8_t* data, uint8_t data_len)
{
    uint8_t tx_data  = instruction;
    SPI_CS_CLR();
    spi_transfer(&tx_data, NULL, 1);
    spi_transfer(NULL, data, data_len);
    SPI_CS_SET();

    return 0x01U;
}
/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
