//
// Created by 21481 on 2025/3/26.
//
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

void servo_task()
{
    while (1)
    {

        if( (HAL_GetTick() - servo_time) > KEY_CHECK_MIN_TIME )
        {
            if(key_E == 1)
            {
                servo_time = HAL_GetTick() ;
                servo_state++ ;
            }
        }

        if( (HAL_GetTick() - servo_rc_time) > RC_CH4_CHECK_MIN_TIME )
        {
            if( rc_ch4 > 300 )
            {
                servo_rc_time = HAL_GetTick() ;
                servo_state++ ;
            }
        }

        if(rc_s1 == 1)
        {
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500);
                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500);
                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);
                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 500);
        }
        else
        {
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1945);
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1945);
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1945);
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1945);
                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1945);
                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1945);
                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1945);
        }

//        if((servo_state % 2) == 1 )//°´¼üÅÐ¶Ï
//        {
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500);
//                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500);
//                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);
//                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 500);
//        }
//        else
//        {
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1945);
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1945);
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1945);
//                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1945);
//                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1945);
//                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1945);
//                    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1945);
//        }

        osDelay(1);
    }
}

