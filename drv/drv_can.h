#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "drv_com.h"

#define CAN_DRIVER_TEST_LOOPBACK

typedef union {
    struct {
        uint8_t idType;
        uint32_t id;
        uint8_t dlc;
        uint8_t data0;
        uint8_t data1;
        uint8_t data2;
        uint8_t data3;
        uint8_t data4;
        uint8_t data5;
        uint8_t data6;
        uint8_t data7;
    } frame;
    uint8_t array[14];
} uCAN_MSG;

#define dSTANDARD_CAN_MSG_ID_2_0B 1
#define dEXTENDED_CAN_MSG_ID_2_0B 2

#ifdef __cplusplus
 extern "C" {
#endif


void can_sleep(void);
bool can_init(void);
uint8_t can_transmit(uCAN_MSG *tempCanMsg);
uint8_t can_receive(uCAN_MSG *tempCanMsg);
uint8_t can_messages_in_buffer(void);
uint8_t can_is_buss_off(void);
uint8_t can_is_rx_error_passive(void);
uint8_t can_is_tx_error_passive(void);

#ifdef CAN_DRIVER_TEST_LOOPBACK
void  can_loopback_test(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ___DRV_CAN_H */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
