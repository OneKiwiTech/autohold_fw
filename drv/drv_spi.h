#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#include "drv_com.h"

#define SPI_CS_PORT         0U
#define SPI_CS_PIN          4U
#define SPI_CS_SET()        GPIO_PinWrite(GPIO, SPI_CS_PORT, SPI_CS_PIN, 1)
#define SPI_CS_CLR()        GPIO_PinWrite(GPIO, SPI_CS_PORT, SPI_CS_PIN, 0)

#define DRV_SPI             SPI3
#define DRV_SPI_IRQ         FLEXCOMM3_IRQn
#define DRV_SPI_CLK_FREQ    CLOCK_GetFlexCommClkFreq(3U)

#ifdef __cplusplus
 extern "C" {
#endif

 void spi_init(void);
uint8_t spi_write(uint8_t value);
uint8_t spi_write_register(uint8_t reg, uint8_t value);
uint8_t spi_read_register(uint8_t reg);

uint8_t spi_can_rst(uint8_t instruction);
uint8_t spi_can_read(uint8_t instruction, uint8_t address);
uint8_t spi_can_read_buffer(uint8_t instruction);
uint8_t spi_can_write(uint8_t instruction, uint8_t address, uint8_t data);
// uint8_t spi_can_load_buffer(uint8_t instruction, uint8_t* iReg, uint8_t dlc, uint8_t* data);
uint8_t spi_can_load_buffer(uint8_t instruction, uint8_t* data, uint8_t data_len);
uint8_t spi_can_bit_modify(uint8_t instruction, uint8_t address, uint8_t mask, uint8_t data);
uint8_t spi_can_status(uint8_t instruction);
uint8_t spi_can_read_rx_buffer(uint8_t instruction, uint8_t* data, uint8_t data_len);
// https://github.com/Magicoe/LPC54114_SmartBanlanceCar/blob/91542cf4f4/code/applications/mpu9250_spi_int_kalmanfilter/driver_spi_mpu9250.c

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SPI_H */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
