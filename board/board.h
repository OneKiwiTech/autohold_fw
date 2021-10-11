/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_reset.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME "LPCXpresso55S28"

/*! @brief The UART to use for debug messages. */
/* TODO: rename UART to USART */
#define BOARD_DEBUG_UART_TYPE       kSerialPort_Uart
#define BOARD_DEBUG_UART_BASEADDR   (uint32_t) USART0
#define BOARD_DEBUG_UART_INSTANCE   0U
#define BOARD_DEBUG_UART_CLK_FREQ   12000000U
#define BOARD_DEBUG_UART_CLK_ATTACH kFRO12M_to_FLEXCOMM0
#define BOARD_DEBUG_UART_RST        kFC0_RST_SHIFT_RSTn
#define BOARD_DEBUG_UART_CLKSRC     kCLOCK_Flexcomm0
#define BOARD_UART_IRQ_HANDLER      FLEXCOMM0_IRQHandler
#define BOARD_UART_IRQ              FLEXCOMM0_IRQn

#define BOARD_ACCEL_I2C_BASEADDR   I2C4
#define BOARD_ACCEL_I2C_CLOCK_FREQ 12000000

#define BOARD_DEBUG_UART_TYPE_CORE1       kSerialPort_Uart
#define BOARD_DEBUG_UART_BASEADDR_CORE1   (uint32_t) USART1
#define BOARD_DEBUG_UART_INSTANCE_CORE1   1U
#define BOARD_DEBUG_UART_CLK_FREQ_CORE1   12000000U
#define BOARD_DEBUG_UART_CLK_ATTACH_CORE1 kFRO12M_to_FLEXCOMM1
#define BOARD_DEBUG_UART_RST_CORE1        kFC1_RST_SHIFT_RSTn
#define BOARD_DEBUG_UART_CLKSRC_CORE1     kCLOCK_Flexcomm1
#define BOARD_UART_IRQ_HANDLER_CORE1      FLEXCOMM1_IRQHandler
#define BOARD_UART_IRQ_CORE1              FLEXCOMM1_IRQn

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200U
#endif /* BOARD_DEBUG_UART_BAUDRATE */

#ifndef BOARD_DEBUG_UART_BAUDRATE_CORE1
#define BOARD_DEBUG_UART_BAUDRATE_CORE1 115200U
#endif /* BOARD_DEBUG_UART_BAUDRATE_CORE1 */


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);
void BOARD_InitDebugConsole_Core1(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/