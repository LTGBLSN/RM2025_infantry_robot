//
// Created by 21481 on 2025/3/19.
//

#ifndef BUBING_RM2025_CHASSIS_MOTOR_CONTROL_H
#define BUBING_RM2025_CHASSIS_MOTOR_CONTROL_H

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



#define CHASSIS_3508_SPEED_PID_OUT_MAX   16384.0f   //max+_16384,for all chassis 3508 motors form id1 to id4
#define CHASSIS_3508_SPEED_PID_KI_MAX   1000.0f


#define CHASSIS_3508_ID1_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID1_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID1_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID2_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID2_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID2_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID3_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID3_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID3_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID4_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID4_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID4_SPEED_PID_KD   0.0f


#define CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KP        2.0f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KI        0.0f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KD        0.0f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_OUT_MAX   3000.0f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KI_MAX    0.0f

#define YAW_MID_ECD 4760

#define CHASSIS_PID_COMPUTE_FREQUENCY 1000  // Hz

#define KEY_CHECK_MIN_TIME 300
#define RC_CH4_CHECK_MIN_TIME 600

#define KEY_MOVE_VX_SPEED 1000
#define KEY_MOVE_VY_SPEED 1000

#define TOP_GIVE_MIN_SPEED 4000

#define MIN_CHASSIS_VROUND_SPEED 1000


extern pid_type_def chassis_3508_ID1_speed_pid;
extern pid_type_def chassis_3508_ID2_speed_pid;
extern pid_type_def chassis_3508_ID3_speed_pid;
extern pid_type_def chassis_3508_ID4_speed_pid;

extern pid_type_def chassis_follow_gimbal_pid;


void rc_to_gimbal_speed_compute();

void gimbal_to_chassis_speed_compute();

void yaw_ecd_angle_to_radian();//±àÂëÆ÷»»Ëã»¡¶È²î

void chassis_settlement();

void motor_chassis_pid_compute();

void chassis_3508_id1_speed_pid_init(void);
int16_t chassis_3508_id1_speed_pid_loop(int16_t chassis_3508_ID1_speed_set_loop);
void chassis_3508_id2_speed_pid_init(void);
int16_t chassis_3508_id2_speed_pid_loop(int16_t chassis_3508_ID2_speed_set_loop);
void chassis_3508_id3_speed_pid_init(void);
int16_t chassis_3508_id3_speed_pid_loop(int16_t chassis_3508_ID3_speed_set_loop);
void chassis_3508_id4_speed_pid_init(void);
int16_t chassis_3508_id4_speed_pid_loop(int16_t chassis_3508_ID4_speed_set_loop);

void chassis_follow_gimbal_angle_pid_init(void);
float chassis_follow_gimbal_pid_loop(float PITCH_6020_ID2_angle_set_loop);



#endif //BUBING_RM2025_CHASSIS_MOTOR_CONTROL_H
