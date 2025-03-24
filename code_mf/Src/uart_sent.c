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
        usart6_printf("%d,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f \r\n",

                      chassis_power_state,
                      robot_max_power,
                      CHASSIS_3508_ALL_COMPUTE_SPEED,
                      beyond_power,
                      chassis_vround,
                      send_out_all_speed);



        osDelay(10);




    }

}


