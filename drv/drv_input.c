/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "drv_input.h"
#include "fsl_gint.h"

#include "FreeRTOS.h"
#include "semphr.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SW_AUTOHOLD_GINT0_PORT kGINT_Port0

/* Select one input, active low for GINT0 */
#define SW_AUTOHOLD_GINT0_POL_MASK ~(1U << BOARD_INITPINS_SW_AUTOHOLD_PIN )
#define SW_AUTOHOLD_GINT0_ENA_MASK (1U << BOARD_INITPINS_SW_AUTOHOLD_PIN )

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
SemaphoreHandle_t xAutoHoldSemaphore = NULL;
static  volatile BaseType_t xHigherPriorityTaskWoken = pdFALSE;;
static  volatile uint32_t  lastAutoTick = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Call back for GINT0 event
 */
void gint0_sw_autohold_callback(void)
{
	uint32_t  curTick = xTaskGetTickCount();
	//if ( drv_input_is_autohold_valid() )
	{
		if ( curTick - lastAutoTick >= pdMS_TO_TICKS(1000) )
		{
			xSemaphoreGiveFromISR( xAutoHoldSemaphore, &xHigherPriorityTaskWoken );
		}
	}
	lastAutoTick = curTick;

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*!
 * @brief Main function
 */
void drv_input_init(void)
{
	/* Nothing to do*/
	GINT_Init(GINT0);
   /* Setup GINT0 for edge trigger, "OR" mode */
    GINT_SetCtrl(GINT0, kGINT_CombineOr, kGINT_TrigEdge, gint0_sw_autohold_callback);
   /* Select pins & polarity for GINT0 */
    GINT_ConfigPins(GINT0, BOARD_INITPINS_SW_AUTOHOLD_PORT, SW_AUTOHOLD_GINT0_POL_MASK, SW_AUTOHOLD_GINT0_ENA_MASK);
    /* Enable callbacks for GINT0 & GINT1 */
    GINT_EnableCallback(GINT0);

	/* Creat semaphore to notify main fsm*/
	xAutoHoldSemaphore = xSemaphoreCreateBinary();
}

bool drv_input_is_autohold_active()
{
	return (xSemaphoreTake( xAutoHoldSemaphore, ( TickType_t ) 10 ) == pdTRUE);
}
