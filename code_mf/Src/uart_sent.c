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
#include "chassis_motor_control.h"

void uart_sent()
{
    while (1)
    {
        usart6_printf("%f,%d \r\n",
                      SHOOT_2006_ID3_GIVEN_SPEED,motor_can2_data[2].speed_rpm);



        osDelay(10);




    }

}


