#include "drv_can.h"
#include "drv_mcp2515.h"
#include "drv_delay.h"
#include "drv_ina219.h"
#include "sensor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "proj_config.h"

#include <stdbool.h>


/* CAN BUS related variables*/
static CAN_Bus_Status_t   CANBusStatus = {0};
static uCAN_MSG 		  rxMessage = {0};

/* Current sensor related variables */
#ifndef DRV_MOTOR_CURRENT_THRESHOLD
	#define DRV_MOTOR_CURRENT_THRESHOLD     1200UL // 1500mA
#endif

static  TimerHandle_t     xCurrentReadingTimer;
static  SemaphoreHandle_t xOverCurrentSemaphore = NULL;

static volatile int16_t current_value = 0;

void  sensor_can_frame_speed_process(uCAN_MSG* can_msg);
void  sensor_can_frame_acc_process(uCAN_MSG* can_msg);
void  sensor_can_frame_gear_process(uCAN_MSG* can_msg);
void  sensor_can_frame_sealbelt_process(uCAN_MSG* can_msg);

static void sensor_canbus_task(void *pvParameters);
/********************************PUBLIC FUNCTION********************************/
bool sensor_is_seatbelt_active()
{
	return CANBusStatus.can_bus_flag.vehicle_seatbelt;
}

bool sensor_is_gear_p_active()
{
	return CANBusStatus.can_bus_flag.vehicle_gear_p;
}

void sensor_clear_gear_p_bit()
{
	CANBusStatus.can_bus_flag.vehicle_gear_p = 0;
}

bool sensor_is_gear_d_active()
{
	return CANBusStatus.can_bus_flag.vehicle_gear_d;
}

bool sensor_is_acc_change()
{
	return CANBusStatus.can_bus_flag.vehicle_acc_change;
}

bool sensor_is_vehicle_stop()
{
	return CANBusStatus.can_bus_flag.vehicle_stop;
}

void vCurrentReadingTimerCallback( TimerHandle_t xTimer )
{
	uint32_t ulCount;

    current_value = getShuntVoltage_raw();
    if (current_value > DRV_MOTOR_CURRENT_THRESHOLD)
    {
        xSemaphoreGive( xOverCurrentSemaphore);	
    }

   /* The number of times this timer has expired is saved as the
	timer's ID.  Obtain the count. */
	ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
	vTimerSetTimerID( xTimer, ( void * ) ulCount );
}

bool sensor_is_overcurrent()
{
    #if 0
	 return ( xSemaphoreTake( xOverCurrentSemaphore, ( TickType_t ) 10 ) == pdTRUE );
    #endif
    current_value = getShuntVoltage_raw();
    if (current_value > DRV_MOTOR_CURRENT_THRESHOLD)
    {
        return true;
    }

    return false;
}

void sensor_flush_current_check()
{
	// uint32_t cnt = 0;
	uint32_t  begin_tick = xTaskGetTickCount();
	do
	{
		getShuntVoltage_raw();
	}while ( xTaskGetTickCount() - begin_tick < pdMS_TO_TICKS(20) );
}


void sensor_start_current_sense()
{
  //  xTimerStart( xCurrentReadingTimer, 0 );
}

void sensor_stop_current_sense()
{
  //  xTimerStop( xCurrentReadingTimer, 0 );
}

void  autohold_task_init()
{
	CANBusStatus.can_bus_status = 0x00;
	/* Create sensor task */
	xTaskCreate(sensor_canbus_task, "CAN BUS TASK",
				configMINIMAL_STACK_SIZE + 512, NULL,
				tskIDLE_PRIORITY + 1, NULL);

    /* Create Timer task to read current sensor */
    xOverCurrentSemaphore = xSemaphoreCreateBinary();  

	xCurrentReadingTimer = xTimerCreate
	                   ( /* Just a text name, not used by the RTOS
	                     kernel. */
	                     "Current Sensor Timer",
	                     /* The timer period in ticks, must be
	                     greater than 0. */
	                     ( pdMS_TO_TICKS(100) ),
	                     /* The timers will auto-reload themselves
	                     when they expire. */
	                     pdTRUE,
	                     /* The ID is used to store a count of the
	                     number of times the timer has expired, which
	                     is initialised to 0. */
	                     ( void * ) 0,
	                     /* Each timer calls the same callback when
	                     it expires. */
	                     vCurrentReadingTimerCallback
	                   );    
                       
    sensor_start_current_sense();
}

/********************************PRIVATE FUNCTION********************************/
static void sensor_canbus_task(void *pvParameters)
{
    while (1)
    {
        if(can_receive(&rxMessage))
        {
            switch (rxMessage.frame.id)
            {
                case CAN_FRAME_SPEED:
                    sensor_can_frame_speed_process(&rxMessage);
                    break;
                case CAN_FRAME_GEAR:
                    sensor_can_frame_gear_process(&rxMessage);
                    break;
                case CAN_FRAME_SEATBELT:
                    sensor_can_frame_sealbelt_process(&rxMessage);
                    break;
                case CAN_FRAME_ACC:
                    sensor_can_frame_acc_process(&rxMessage);
                    break;
                default:
                	break;
            }
        }
        delay_rtos_ms(10);
    }
}

void  sensor_can_frame_speed_process(uCAN_MSG* can_msg)
{
	if ( can_msg->frame.data1 == 0 )
	{
		CANBusStatus.can_bus_flag.vehicle_stop = 1;
		CANBusStatus.can_bus_flag.vehicle_acc_change = 0;
	}else
	{
		CANBusStatus.can_bus_flag.vehicle_stop = 0;
	}

}

void sensor_can_frame_gear_process(uCAN_MSG* can_msg)
{
    uint8_t gear =  can_msg->frame.data4;

    switch (gear)
    {
        case 0x60: /* Gear at P*/
        	CANBusStatus.can_bus_flag.vehicle_gear_p = 1;
        	CANBusStatus.can_bus_flag.vehicle_gear_d = 0;
        	break;
        case 0x65: /* EVENT_CAN_BUS_GEAR_D */
        	CANBusStatus.can_bus_flag.vehicle_gear_d = 1;
        	CANBusStatus.can_bus_flag.vehicle_gear_p = 0;
        	break;
        default:
        	{
				CANBusStatus.can_bus_flag.vehicle_gear_p = 0;
				CANBusStatus.can_bus_flag.vehicle_gear_d = 0;
        	}
        	break;
    }
}

void  sensor_can_frame_sealbelt_process(uCAN_MSG* can_msg)
{
    uint8_t status = can_msg->frame.data1;

    if (status == 0x04){
    	/* Seatbelt is ON */
    	CANBusStatus.can_bus_flag.vehicle_seatbelt = 1;
    }else
    {
    	/* Reset autohold state */
    	CANBusStatus.can_bus_flag.vehicle_seatbelt = 0;
    }
}

void  sensor_can_frame_acc_process(uCAN_MSG* can_msg)
{
    uint8_t status = can_msg->frame.data6;
    if (status == 0x00){
       /* Acc zero */
    	CANBusStatus.can_bus_flag.vehicle_acc_change = 0;
    }else
    {
    	/* Event:EVENT_CAN_BUS_ACC_CHANGE */
        /* Set un-breaking event */
    	CANBusStatus.can_bus_flag.vehicle_acc_change = 1;
    }
}
