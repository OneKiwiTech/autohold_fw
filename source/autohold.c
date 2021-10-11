#include <stdbool.h>

#include "autohold.h"
#include "drv_input.h"
#include "drv_motor.h"
#include "drv_delay.h"
#include "sensor_task.h"

#include "FreeRTOS.h"
#include "timers.h"

static volatile FsmStateEnum_t  fsm_state = FSM_MANUAL_MODE;
static  TimerHandle_t   xBlinkyTimer;

static void autohold_fsm_task(void *pvParameters);
static void autohold_init_timer();

void vBlinkyTimerCallback( TimerHandle_t xTimer )
{
	uint32_t ulCount;

	drv_led_autohold_toggle();

   /* The number of times this timer has expired is saved as the
	timer's ID.  Obtain the count. */
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	vTimerSetTimerID( xTimer, ( void * ) ulCount );
}

void  sensor_task_init()
{
	autohold_init_timer();
	/* Create sensor task */
	xTaskCreate(autohold_fsm_task, "AUTOHOLD FSM TASK",
				configMINIMAL_STACK_SIZE + 512, NULL,
				tskIDLE_PRIORITY + 1, NULL);
}

static void autohold_init_timer()
{
	xBlinkyTimer = xTimerCreate
	                   ( /* Just a text name, not used by the RTOS
	                     kernel. */
	                     "LED Timer",
	                     /* The timer period in ticks, must be
	                     greater than 0. */
	                     ( pdMS_TO_TICKS(500) ),
	                     /* The timers will auto-reload themselves
	                     when they expire. */
	                     pdTRUE,
	                     /* The ID is used to store a count of the
	                     number of times the timer has expired, which
	                     is initialised to 0. */
	                     ( void * ) 0,
	                     /* Each timer calls the same callback when
	                     it expires. */
	                     vBlinkyTimerCallback
	                   );
}

static void autohold_do_manual_operation()
{
	/* Checking manual break request */
	if ( drv_input_is_break_active() )
	{
		/* Show LED Indicator */
		drv_led_breaking_on();
		/* Do breaking */
		drv_motor_breaking();
	}else if ( drv_input_is_debreak_active() && drv_input_is_pedal_break_active() ){
		/* Show LED Indicator */
		drv_led_breaking_off();
		/* Do breaking */
		drv_motor_debreaking();
	}
}


static void autohold_enter_auto_mode()
{
	drv_led_autohold_on();
}

static void autohold_exit_auto_mode()
{
	xTimerStop( xBlinkyTimer, 0 );
	drv_led_autohold_off();
}

static bool autohold_error_check()
{
	bool xRet = false;

#if USE_CURRENT_SENSOR  > 0
		if ( sensor_is_overcurrent() )
		{
			xRet = true;
		}
#endif

	return xRet;
}

static void autohold_error_handler()
{
	/* Stop motor immediately*/
	drv_motor_run_stop();

	/* Show Error LED */
	drv_led_autohold_on();
	drv_led_breaking_on();
}

static void autohold_fsm_task(void *pvParameters)
{
  while(1)
  {
	if ( autohold_error_check() == true)
	{
		autohold_error_handler();
		goto SKIP_FSM;
	}

	switch(fsm_state)
	{
	    case FSM_MANUAL_MODE:
		{
			/* Check ACC Switch */
			if ( drv_input_is_acc_active() )
			{
				/* Engine start state */
				fsm_state = FSM_ENGINE_START_MODE;
			}else
			{
				autohold_do_manual_operation();
			}

			/* Condition exit this mode */
			if ( drv_input_is_autohold_active() && sensor_is_seatbelt_active() )
			{
				autohold_enter_auto_mode();
				fsm_state = FMS_AUTOHOLD_MODE;

			}
		}
		break;
	   case FSM_ENGINE_START_MODE:
			if ( sensor_is_gear_p_active() )
			{
				/* Show LED Indicator */
				drv_led_breaking_on();
				/* Do breaking */
				drv_motor_breaking();
			} else
			{
				/* Check Acc changes? */
				if ( sensor_is_acc_change() )
				{
					drv_led_breaking_off();
					/*Un-breaking */
					drv_motor_debreaking();
				}
			}

			if ( sensor_is_vehicle_stop() )
			{
				autohold_do_manual_operation();
			}

			/* Check condition exit Engine Start Mode */
			if ( drv_input_is_autohold_active() && sensor_is_seatbelt_active() )
			{
				/* Start LED Blinky Timer */
				autohold_enter_auto_mode();
				fsm_state = FMS_AUTOHOLD_MODE;
			}

			if ( !drv_input_is_acc_active() )
			{
				fsm_state = FSM_MANUAL_MODE;
			}

			break;

		case FMS_AUTOHOLD_MODE:
		{
			/* Check seatbelt status */
			if ( sensor_is_seatbelt_active() )
			{
				/* Check BREAK SWITCH active in 3 second */
				if ( drv_input_is_pedal_break_active() )
				{
					delay_rtos_ms(3000);
					if ( drv_input_is_pedal_break_active() )
					{
						/* Check if BREAK is Active at least 3 seconds*/
						if ( sensor_is_vehicle_stop() && sensor_is_gear_d_active() )
						{
							xTimerStart( xBlinkyTimer, 0 );
							/* Show LED Indicator */
							drv_led_breaking_on();
							/* Breaking */
							drv_motor_breaking();
						}
					}
				}
			}

			/* Check Acc changes? */
			if ( sensor_is_acc_change() )
			{
				/* Turn of blinky LED*/
				xTimerStop( xBlinkyTimer, 0 );
				drv_led_autohold_on();

				/*Un-breaking */
				drv_led_breaking_off();
				drv_motor_debreaking();
			}

			/* Exit state condition */
			if (drv_input_is_autohold_active())
			{
				autohold_exit_auto_mode();
				fsm_state = FSM_MANUAL_MODE;
			}
		}
		break;
		default:
		{
			/*Un-breaking */
			drv_led_breaking_on();
			drv_motor_breaking();
		}
		break;
	}

SKIP_FSM:	
	delay_rtos_ms(20);
  }
}
