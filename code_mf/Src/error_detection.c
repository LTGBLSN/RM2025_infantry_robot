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

        rc_connection_status();//“£øÿ∆˜¿Îœﬂ≈–∂œ
        imu_connection_status();//imu¿Îœﬂ≈–∂œ




        osDelay(1);
    }


}


void rc_connection_status()
{
    if(HAL_GetTick() - rc_receive_time > RC_NO_DATA_TIMEOUT)
    {
        rc_receive_state = RC_OFFLINE ;//“£øÿ∆˜¿Îœﬂ
    }
    else
    {
        rc_receive_state = RC_ONLINE ;//“£øÿ∆˜‘⁄œﬂ
    }
}

void imu_connection_status()
{
    if(HAL_GetTick() - imu_receive_time > IMU_NO_DATA_TIMEOUT)
    {
        imu_receive_state = IMU_OFFLINE ;//IMU¿Îœﬂ
    }
    else
    {
        imu_receive_state = IMU_ONLINE ;//IMU‘⁄œﬂ
    }
}


