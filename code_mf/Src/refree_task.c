//
// Created by 21481 on 2025/3/24.
//
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
#include "jy61p.h"
#include "pid.h"
#include "chassis_motor_control.h"
#include "gimbal_motor_control.h"
#include "shoot_control.h"
#include <math.h>
#include "refree_task.h"

void refree_task()
{
    while (1)
    {
        max_power_compute();//更新最新的最大功率



        osDelay(1);
    }
}



void max_power_compute()
{
    if(robot_level == 1)
    {
        robot_max_power = 60 ;

    }
    else if(robot_level == 2)
    {
        robot_max_power = 65 ;
    }
    else if(robot_level == 3)
    {
        robot_max_power = 70 ;
    }
    else if(robot_level == 4)
    {
        robot_max_power = 75 ;
    }
    else if(robot_level == 5)
    {
        robot_max_power = 80 ;
    }
    else if(robot_level == 6)
    {
        robot_max_power = 85 ;
    }
    else if(robot_level == 7)
    {
        robot_max_power = 90 ;
    }
    else if(robot_level == 8)
    {
        robot_max_power = 95 ;
    }
    else if(robot_level == 9)
    {
        robot_max_power = 100 ;
    }
    else if(robot_level == 10)
    {
        robot_max_power = 100 ;
    }
}

