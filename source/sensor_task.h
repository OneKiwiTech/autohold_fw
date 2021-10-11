#ifndef SENSOR_H_
#define SENSOR_H_

typedef enum {
    CAN_FRAME_SPEED    = 0x04F1,
    CAN_FRAME_GEAR     = 0x0367,
    CAN_FRAME_SEATBELT = 0x0541,
    CAN_FRAME_ACC      = 0x329
} CAN_Frame_Type_Enum;


typedef union {
	struct
	{
		uint8_t  vehicle_stop:1;
		uint8_t  vehicle_acc_change:1;
		uint8_t  vehicle_gear_p:1;
		uint8_t  vehicle_gear_d:1;
		uint8_t  vehicle_seatbelt:1;
		uint8_t  reserve:3;
	}can_bus_flag;
	uint8_t can_bus_status;
} CAN_Bus_Status_t;


void sensor_task_init();
bool sensor_is_seatbelt_active();
bool sensor_is_gear_p_active();
bool sensor_is_gear_d_active();
bool sensor_is_acc_change();
bool sensor_is_vehicle_stop();


void sensor_clear_gear_p_bit();

bool sensor_is_overcurrent();
void sensor_flush_current_check();

void sensor_start_current_sense();
void sensor_stop_current_sense();

#endif /* SENSOR_H_ */
