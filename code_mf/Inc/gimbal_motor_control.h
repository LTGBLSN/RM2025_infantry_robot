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


#define YAW_6020_ID2_ANGLE_PID_KP        0.2f
#define YAW_6020_ID2_ANGLE_PID_KI        0.0f
#define YAW_6020_ID2_ANGLE_PID_KD        0.0f
#define YAW_6020_ID2_ANGLE_PID_OUT_MAX   10.0f
#define YAW_6020_ID2_ANGLE_PID_KI_MAX    0.0f

#define YAW_6020_ID2_SPEED_PID_KP        10000.0f
#define YAW_6020_ID2_SPEED_PID_KI        20.0f
#define YAW_6020_ID2_SPEED_PID_KD        0.0f
#define YAW_6020_ID2_SPEED_PID_OUT_MAX   30000.0f
#define YAW_6020_ID2_SPEED_PID_KI_MAX    10000.0f






#define PITCH_6020_ID2_SPEED_PID_KP        10000.0f
#define PITCH_6020_ID2_SPEED_PID_KI        20.0f
#define PITCH_6020_ID2_SPEED_PID_KD        0.0f
#define PITCH_6020_ID2_SPEED_PID_OUT_MAX   30000.0f
#define PITCH_6020_ID2_SPEED_PID_KI_MAX    15000.0f

#define PITCH_6020_ID2_ANGLE_PID_KP        0.4f
#define PITCH_6020_ID2_ANGLE_PID_KI        0.0f
#define PITCH_6020_ID2_ANGLE_PID_KD        0.0f
#define PITCH_6020_ID2_ANGLE_PID_OUT_MAX   5.0f
#define PITCH_6020_ID2_ANGLE_PID_KI_MAX    0.0f





#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KP        5.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KI        0.1f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KD        0.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_OUT_MAX   16000.0f
#define FRICTION_WHEEL_3510_ID1_SPEED_PID_KI_MAX    5000.0f

#define FRICTION_WHEEL_3510_ID2_SPEED_PID_KP        5.0f
#define FRICTION_WHEEL_3510_ID2_SPEED_PID_KI        0.1f
#define FRICTION_WHEEL_3510_ID2_SPEED_PID_KD        0.0f
#define FRICTION_WHEEL_3510_ID2_SPEED_PID_OUT_MAX   16000.0f
#define FRICTION_WHEEL_3510_ID2_SPEED_PID_KI_MAX    5000.0f


#define FRICTION_WHEEL_SHOOT_SPEED 6500

#define GIMBAL_PID_COMPUTE_FREQUENCY 1000  // Hz

#define PITCH_OFF_FRICTION_STOP_SPEED_COMPENSATE 0.0
#define PITCH_ON_FRICTION_STOP_SPEED_COMPENSATE  0.0

#define PITCH_RC_IN_KP (-0.0005f)
#define YAW_RC_IN_KP (-0.0008f)

#define MOUSE_VX_SPEED_SCALING_FACTOR 2.0f
#define MOUSE_VY_SPEED_SCALING_FACTOR 0.1f



extern pid_type_def yaw_6020_ID1_speed_pid;


extern pid_type_def pitch_6020_ID2_speed_pid;
extern pid_type_def pitch_6020_ID2_angle_pid;




extern pid_type_def friction_wheel_3510_ID1_speed_pid;
extern pid_type_def friction_wheel_3510_ID2_speed_pid;



extern pid_type_def shoot_2006_ID3_speed_pid;



void yaw_imu_getAbscissa();//YAW里程计
void motor_gimbal_angle_compute();//目标赋值

void motor_gimbal_pid_compute();//pid计算

void friction_wheel_speed_control();//目标赋值
void friction_wheel_pid_control();//pid计算





//void pitch_motor_mean_speed_compute();//权重速度滤波 弃用，有一定滞后性



void yaw_speed_pid_init(void);
float yaw_speed_pid_loop(float YAW_6020_ID1_speed_set_loop);
void yaw_angle_pid_init(void);
float yaw_angle_pid_loop(float YAW_6020_ID1_angle_set_loop);

void pitch_speed_from_bmi88_pid_init(void);
float pitch_speed_from_bmi088_pid_loop(float PITCH_6020_ID2_speed_set_loop);
void pitch_angle_pid_init(void);
float pitch_angle_from_bmi088_pid_loop(float PITCH_6020_ID2_angle_set_loop);//暂不用，pitch速控



void friction_wheel_3510_id1_speed_pid_init(void);
int16_t friction_wheel_3510_id1_speed_pid_loop(int16_t friction_wheel_3510_id1_speed_set_loop);

void friction_wheel_3510_id2_speed_pid_init(void);
int16_t friction_wheel_3510_id2_speed_pid_loop(int16_t friction_wheel_3510_id2_speed_set_loop);




#endif //BUBING_RM2025_GIMBAL_MOTOR_CONTROL_H
