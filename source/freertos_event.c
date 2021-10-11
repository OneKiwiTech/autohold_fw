/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_power.h"

#include "sensor_task.h"

#include "drv_mcp2515.h"
#include "drv_can.h"
#include "drv_ina219.h"
#include "drv_delay.h"

#include "sensor_task.h"
#include "autohold.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Globals
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Init driver subsystem */
    drv_input_init();
    delay_init();

    /* CAN bus*/
    mcp2515_init();
    can_init();

    /* Current sensor */
    ina219_init();

#if DRIVER_CAN_BUS_TEST
    can_loopback_test();
#endif

#if DRIVER_CURRENT_SENSOR_TEST
    drv_ina219_test();
#endif
    /*TEST stuff */
    PRINTF("Hello world\r\n");
    /* Init all sub system task */
    sensor_task_init();
    autohold_task_init();
    
    /* Start scheduling. */
    vTaskStartScheduler();
    for (;;);
}
