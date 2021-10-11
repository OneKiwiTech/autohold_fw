#include "drv_motor.h"
#include "fsl_gpio.h"
#include "sensor_task.h"
#include "drv_delay.h"

#include "FreeRTOS.h"
#include "task.h"

#include "proj_config.h"



#define SPI_CS_PORT         0U
#define SPI_CS_PIN          4U

#define  MOTOR_BREAKING_PERIOD       300   // 10 * 100ms = 1 second
#define  MOTOR_DEBREAKING_PERIOD     2000   // 30 * 100ms = 3 second


typedef enum
{
	MOTOR_IN_BREAKING_STATE = 0x00,
	MOTOR_IN_DEBREAKING_STATE,
	MOTOR_IN_INVALID_STATE
} MOTOR_CONTROL_STATE;

static volatile MOTOR_CONTROL_STATE  motor_state = MOTOR_IN_INVALID_STATE;

void drv_motor_init()
{
	
}



void drv_motor_breaking()
{
	uint32_t i = 0;

	if (motor_state == MOTOR_IN_BREAKING_STATE) {return;}
	sensor_stop_current_sense();
	drv_motor_run_fwd();

	/* Flush inrush current */
	sensor_flush_current_check();

	/* Waiting for timeout or over current occurs */
	for (i = 0; i < MOTOR_BREAKING_PERIOD; i++)
	{
#if USE_CURRENT_SENSOR  > 0
		if ( sensor_is_overcurrent() )
		{
			/* Stop motor immediately*/
			drv_motor_run_stop();
		}
#endif
		delay_rtos_ms(10);
	}
	drv_motor_run_stop();
	
	sensor_start_current_sense();
	motor_state = MOTOR_IN_BREAKING_STATE;
}

void drv_motor_debreaking()
{
	if (motor_state == MOTOR_IN_DEBREAKING_STATE) {return;}

	drv_motor_run_rev();

	delay_rtos_ms(MOTOR_DEBREAKING_PERIOD);

	drv_motor_run_stop();

	motor_state = MOTOR_IN_DEBREAKING_STATE;
}

