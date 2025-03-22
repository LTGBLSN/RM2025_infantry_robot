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
        usart6_printf("%6.2f,%6.2f,%d \r\n",

                      YAW_IMU_SPEED,
                      YAW_6020_ID1_GIVEN_SPEED,
                      motor_can1_data[4].ecd);



        osDelay(10);




    }

}


