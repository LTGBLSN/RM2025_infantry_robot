//
// Created by 21481 on 2025/3/17.
//
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"

void can_sent()
{
    while (1)
    {

        if(rc_s0 == 2)
        {
            CAN2_cmd_gimbal(0, 0, 0, 0);
            CAN1_cmd_chassis(0, 0, 0, 0);
            CAN2_cmd_friction_wheels(0,0,0,0);
        }
        if(rc_s0 == 1)
        {
            CAN2_cmd_gimbal(0, 0, 0, 0);
            CAN1_cmd_chassis(0, 0, 0, 0);
            CAN2_cmd_friction_wheels(0,0,0,0);
        }
        else if(rc_s0 == 3)
        {
            CAN2_cmd_gimbal(0, 0, 500, 0);
            CAN2_cmd_friction_wheels(500,-500,0,0);
            CAN1_cmd_chassis(0, 0, 0, 0);
        }
        osDelay(1);
    }




}

