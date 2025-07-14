//
// Created by 21481 on 2025/3/17.
//

#ifndef BUBING_RM2025_CAN_SENT_H
#define BUBING_RM2025_CAN_SENT_H

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"

void can_sent();

void can_cmd_all(int16_t chassis_id1 ,       int16_t chassis_id2 ,
                 int16_t chassis_id3 ,       int16_t chassis_id4 ,
                 int16_t yaw_id1 ,           int16_t pitch_id2 ,
                 int16_t friction_wheel_id1, int16_t friction_wheel_id2 ,
                 int16_t shoot_id3 );


#endif //BUBING_RM2025_CAN_SENT_H
