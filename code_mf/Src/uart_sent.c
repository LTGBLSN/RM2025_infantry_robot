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

void uart_sent()
{
    while (1)
    {

        usart6_printf("%d,%d,%d,%d,%d,%d,%d \r\n",
                      motor_can1_data[0].speed_rpm,
                      motor_can1_data[1].speed_rpm,
                      motor_can1_data[2].speed_rpm,
                      motor_can1_data[3].speed_rpm,
                      motor_can1_data[4].speed_rpm,
                      motor_can1_data[5].speed_rpm,
                      motor_can1_data[6].speed_rpm);

        osDelay(10);




    }

}


