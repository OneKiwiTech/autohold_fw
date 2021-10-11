#ifndef _DRV_INPUT_H_
#define _DRV_INPUT_H_

#include "pin_mux.h"
#include "board.h"
#include "fsl_common.h"

#include <stdbool.h>

static inline bool drv_input_is_acc_active()
{
	return (GPIO_PinRead(BOARD_INITPINS_SW_ACC_GPIO, BOARD_INITPINS_SW_ACC_PORT, BOARD_INITPINS_SW_ACC_PIN) == 1);
}

/* Pedal  Signal */
static inline bool drv_input_is_pedal_break_active()
{
	return (GPIO_PinRead(BOARD_INITPINS_SW_PEDAL_BRAKE_GPIO, BOARD_INITPINS_SW_PEDAL_BRAKE_PORT, BOARD_INITPINS_SW_PEDAL_BRAKE_PIN) == 1);
}

/* User Switch Signal  */
static inline bool drv_input_is_break_active()
{
	return (GPIO_PinRead(BOARD_INITPINS_SW_BRAKE_GPIO, BOARD_INITPINS_SW_BRAKE_PORT, BOARD_INITPINS_SW_BRAKE_PIN) == 0);
}

static inline bool drv_input_is_debreak_active()
{
	return (GPIO_PinRead(BOARD_INITPINS_SW_DEBRAKE_GPIO, BOARD_INITPINS_SW_DEBRAKE_PORT, BOARD_INITPINS_SW_DEBRAKE_PIN) == 0);
}

static inline bool drv_input_is_autohold_valid()
{
	return (GPIO_PinRead(GPIO, BOARD_INITPINS_SW_AUTOHOLD_PORT, BOARD_INITPINS_SW_AUTOHOLD_PIN) == 0);
}

bool drv_input_is_autohold_active();
void drv_input_init(void);

#endif /* _DRV_INPUT_H_ */
