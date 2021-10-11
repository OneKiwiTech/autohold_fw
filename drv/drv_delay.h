#ifndef __DRV_PWM_H
#define __DRV_PWM_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define delay_rtos_ms( ms )   do {  vTaskDelay( pdMS_TO_TICKS(ms) ); } while(0);

void delay_init(void);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_PWM_H */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
