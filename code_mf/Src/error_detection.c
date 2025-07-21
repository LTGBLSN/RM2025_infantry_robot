//
// Created by 21481 on 2025/3/18.
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
#include "error_detection.h"


void error_detection()
{
    while (1)
    {

        rc_connection_status();//遥控器离线判断
        yaw_6020_status();//yaw电机过温判断
        pitch_6020_status();//pitch电机过温判断




        osDelay(1);
    }


}


void rc_connection_status()
{
    if(HAL_GetTick() - rc_receive_time > RC_NO_DATA_TIMEOUT)
    {
        rc_receive_state = RC_OFFLINE ;//遥控器离线
    }
    else
    {
        rc_receive_state = RC_ONLINE ;//遥控器在线
    }
}




void yaw_6020_status()
{
    if(GM6020_TEMP_MAX > motor_can1_data[4].temperate)
    {
        yaw_6020_state = GM6020_SAFE ;//安全
    }
    else
    {
        yaw_6020_state = GM6020_DIE ;//过温
    }

}

void pitch_6020_status()
{
    if(GM6020_TEMP_MAX > motor_can2_data[5].temperate)//观测值需要改
    {
        pitch_6020_state = GM6020_SAFE ;//安全
    }
    else
    {
        pitch_6020_state = GM6020_DIE ;//过温
    }

}


