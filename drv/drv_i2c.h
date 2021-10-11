#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#include "drv_com.h"

#define EXAMPLE_I2C_MASTER_BASE    (I2C0_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY (12000000UL)
#define WAIT_TIME                   10U
#define EXAMPLE_I2C_MASTER 			((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define I2C_MASTER_SLAVE_ADDR_7BIT  (0x45U)

#define I2C_BAUDRATE               100000U

#ifdef __cplusplus
 extern "C" {
#endif

 void i2c_init(void);
 uint8_t i2c_write_register(uint8_t reg_addr, uint8_t* value, uint32_t len);
 uint8_t i2c_read_registers(uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

#ifdef __cplusplus
}
#endif

#endif /* ___DRV_I2C_H */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
