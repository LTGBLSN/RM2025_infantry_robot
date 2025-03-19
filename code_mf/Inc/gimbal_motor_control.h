//
// Created by 21481 on 2025/3/19.
//

#ifndef BUBING_RM2025_GIMBAL_MOTOR_CONTROL_H
#define BUBING_RM2025_GIMBAL_MOTOR_CONTROL_H



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
#include <math.h>


#define YAW_6020_ID2_SPEED_PID_KP   300.0f
#define YAW_6020_ID2_SPEED_PID_KI   0.0f
#define YAW_6020_ID2_SPEED_PID_KD   0.0f
#define YAW_6020_ID2_SPEED_PID_OUT_MAX   16000.0f
#define YAW_6020_ID2_SPEED_PID_KI_MAX   0.0f

extern pid_type_def yaw_6020_ID2_speed_pid;


#define GIMBAL_PID_COMPUTE_FREQUENCY 1000  // Hz


void motor_yaw_pid_compute();

void yaw_speed_pid_init();
int16_t yaw_speed_pid_loop(int16_t YAW_6020_ID2_speed_set_loop);



#endif //BUBING_RM2025_GIMBAL_MOTOR_CONTROL_H
