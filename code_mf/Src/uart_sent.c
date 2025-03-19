//
// Created by 21481 on 2025/3/16.
//

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "jy61p.h"

void uart_sent()
{
    while (1)
    {
        usart6_printf("%d,%d,%d,%d,%d \r\n",
                      ( YAW_6020_ID2_GIVEN_SPEED - motor_can1_data[4].speed_rpm) ,
                      YAW_6020_ID2_GIVEN_SPEED ,
                      motor_can1_data[4].speed_rpm ,
                      yaw_6020_state ,
                      motor_can1_data[4].temperate);

//        usart6_printf("%d,%d \r\n",
//                      motor_can2_data[5].speed_rpm,
//                      rc_receive_time);

        osDelay(10);




    }

}


