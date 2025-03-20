//
// Created by 21481 on 2025/3/19.
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
#include <math.h>

pid_type_def yaw_6020_ID1_speed_pid;
pid_type_def pitch_6020_ID2_speed_pid;
pid_type_def friction_wheel_3510_ID1_speed_pid;


void gimbal_motor_control()
{
    while (1)
    {


        motor_gimbal_pid_compute();




        osDelay((1/GIMBAL_PID_COMPUTE_FREQUENCY)*1000);//控制频率

#if GIMBAL_PID_COMPUTE_FREQUENCY == 0
        osDelay(1);

#endif


    }
}


void motor_gimbal_pid_compute()
{
    YAW_6020_ID1_GIVEN_SPEED = 0 ;
    PITCH_6020_ID2_GIVEN_SPEED = 0 ;

    FRICTION_WHEEL_3510_ID1_GIVEN_SPEED = 1000 ;


    //yaw
    YAW_6020_ID1_GIVEN_CURRENT = yaw_speed_pid_loop(YAW_6020_ID1_GIVEN_SPEED);//速度环


    //pitch
    PITCH_6020_ID2_GIVEN_CURRENT = pitch_speed_pid_loop(PITCH_6020_ID2_GIVEN_SPEED); //速度环



    //friction_wheel
    FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT = friction_wheel_3510_id1_speed_pid_loop(FRICTION_WHEEL_3510_ID1_GIVEN_SPEED);//速度环
}





void yaw_speed_pid_init(void)
{
    static fp32 yaw_6020_id1_speed_kpkikd[3] = {YAW_6020_ID2_SPEED_PID_KP, YAW_6020_ID2_SPEED_PID_KI, YAW_6020_ID2_SPEED_PID_KD};
    PID_init(&yaw_6020_ID1_speed_pid, PID_POSITION, yaw_6020_id1_speed_kpkikd, YAW_6020_ID2_SPEED_PID_OUT_MAX, YAW_6020_ID2_SPEED_PID_KI_MAX);

}

int16_t yaw_speed_pid_loop(int16_t YAW_6020_ID1_speed_set_loop)
{
    PID_calc(&yaw_6020_ID1_speed_pid, motor_can1_data[4].speed_rpm , YAW_6020_ID1_speed_set_loop);
    int16_t yaw_6020_ID1_given_current_loop = (int16_t)(yaw_6020_ID1_speed_pid.out);

    return yaw_6020_ID1_given_current_loop ;

}





void pitch_speed_pid_init(void)
{
    static fp32 pitch_6020_id2_speed_kpkikd[3] = {PITCH_6020_ID2_SPEED_PID_KP,PITCH_6020_ID2_SPEED_PID_KI,PITCH_6020_ID2_SPEED_PID_KD};
    PID_init(&pitch_6020_ID2_speed_pid,PID_POSITION,pitch_6020_id2_speed_kpkikd,PITCH_6020_ID2_SPEED_PID_OUT_MAX,PITCH_6020_ID2_SPEED_PID_KI_MAX);

}

int16_t pitch_speed_pid_loop(int16_t PITCH_6020_ID2_speed_set_loop)
{
    PID_calc(&pitch_6020_ID2_speed_pid, motor_can2_data[5].speed_rpm , PITCH_6020_ID2_speed_set_loop);
    int16_t pitch_6020_ID2_given_current_loop = (int16_t)(pitch_6020_ID2_speed_pid.out);

    return pitch_6020_ID2_given_current_loop ;

}

//friction wheel
void friction_wheel_3510_id1_speed_pid_init(void)
{
    static fp32 friction_wheel_3510_id1_speed_kpkikd[3] = {FRICTION_WHEEL_3510_ID1_SPEED_PID_KP,FRICTION_WHEEL_3510_ID1_SPEED_PID_KI,FRICTION_WHEEL_3510_ID1_SPEED_PID_KD};
    PID_init(&friction_wheel_3510_ID1_speed_pid,PID_POSITION,friction_wheel_3510_id1_speed_kpkikd,FRICTION_WHEEL_3510_ID1_SPEED_PID_OUT_MAX,FRICTION_WHEEL_3510_ID1_SPEED_PID_KI_MAX);

}

int16_t friction_wheel_3510_id1_speed_pid_loop(int16_t friction_wheel_3510_id1_speed_set_loop)
{
    PID_calc(&friction_wheel_3510_ID1_speed_pid, motor_can2_data[0].speed_rpm , friction_wheel_3510_id1_speed_set_loop);
    int16_t friction_wheel_3510_id1_given_current_loop = (int16_t)(friction_wheel_3510_ID1_speed_pid.out);

    return friction_wheel_3510_id1_given_current_loop ;

}

