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


#define YAW_6020_ID2_SPEED_PID_KP        200.0f
#define YAW_6020_ID2_SPEED_PID_KI        2.0f
#define YAW_6020_ID2_SPEED_PID_KD        0.0f
#define YAW_6020_ID2_SPEED_PID_OUT_MAX   30000.0f
#define YAW_6020_ID2_SPEED_PID_KI_MAX    25000.0f

#define PITCH_6020_ID2_SPEED_PID_KP        550.0f
#define PITCH_6020_ID2_SPEED_PID_KI        2.0f
#define PITCH_6020_ID2_SPEED_PID_KD        0.0f
#define PITCH_6020_ID2_SPEED_PID_OUT_MAX   30000.0f
#define PITCH_6020_ID2_SPEED_PID_KI_MAX    2000.0f

#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KP        45.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KI        0.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KD        0.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_OUT_MAX   16384.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KI_MAX    10000.0f

#define GIMBAL_PID_COMPUTE_FREQUENCY 1000  // Hz


extern pid_type_def yaw_6020_ID1_speed_pid;
extern pid_type_def pitch_6020_ID2_speed_pid;
extern pid_type_def friction_wheel_3510_ID1_speed_pid;





void motor_gimbal_pid_compute();


void yaw_speed_pid_init(void);
int16_t yaw_speed_pid_loop(int16_t YAW_6020_ID1_speed_set_loop);

void pitch_speed_pid_init(void);
int16_t pitch_speed_pid_loop(int16_t PITCH_6020_ID2_speed_set_loop);



void friction_wheel_3510_id1_speed_pid_init(void);
int16_t friction_wheel_3510_id1_speed_pid_loop(int16_t friction_wheel_3510_id1_speed_set_loop);



#endif //BUBING_RM2025_GIMBAL_MOTOR_CONTROL_H
