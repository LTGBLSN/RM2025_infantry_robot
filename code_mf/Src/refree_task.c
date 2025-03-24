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




        if( (HAL_GetTick() - robot_level_time ) > KEY_CHECK_MIN_TIME )
        {
            if(key_B == 1)
            {
                robot_level_time = HAL_GetTick() ;
                if( robot_level < 10 )
                {
                    robot_level++ ;
                }

            }
            if(key_V == 1)
            {
                robot_level_time = HAL_GetTick() ;

                if( 1 < robot_level )
                {
                    robot_level-- ;
                }

            }
        }


        max_power_compute();//更新最新的最大功率



        osDelay(1);
    }
}



void max_power_compute()
{
    if(robot_level == 1)
    {
        robot_max_power = LEVEL_1_MAX_POWER ;

    }
    else if(robot_level == 2)
    {
        robot_max_power = LEVEL_2_MAX_POWER ;
    }
    else if(robot_level == 3)
    {
        robot_max_power = LEVEL_3_MAX_POWER ;
    }
    else if(robot_level == 4)
    {
        robot_max_power = LEVEL_4_MAX_POWER ;
    }
    else if(robot_level == 5)
    {
        robot_max_power = LEVEL_5_MAX_POWER ;
    }
    else if(robot_level == 6)
    {
        robot_max_power = LEVEL_6_MAX_POWER ;
    }
    else if(robot_level == 7)
    {
        robot_max_power = LEVEL_7_MAX_POWER ;
    }
    else if(robot_level == 8)
    {
        robot_max_power = LEVEL_8_MAX_POWER ;
    }
    else if(robot_level == 9)
    {
        robot_max_power = LEVEL_9_MAX_POWER ;
    }
    else if(robot_level == 10)
    {
        robot_max_power = LEVEL_10_MAX_POWER ;
    }
}

