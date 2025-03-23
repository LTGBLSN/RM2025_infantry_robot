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
#include <math.h>

pid_type_def chassis_3508_ID1_speed_pid;
pid_type_def chassis_3508_ID2_speed_pid;
pid_type_def chassis_3508_ID3_speed_pid;
pid_type_def chassis_3508_ID4_speed_pid;



void chassis_motor_control()
{
    while (1)
    {
        rc_to_gimbal_speed_compute();//遥控器转换为云台速度
        yaw_ecd_angle_to_radian();
        gimbal_to_chassis_speed_compute();//底盘速度解算
        chassis_settlement();//轮系速度解算（暂时无云台）
        motor_chassis_pid_compute();//pid速度

        osDelay((1/CHASSIS_PID_COMPUTE_FREQUENCY)*1000);




#if CHASSIS_PID_COMPUTE_FREQUENCY == 0
        osDelay(1);

#endif

    }
}

void rc_to_gimbal_speed_compute()
{
    if( key_W == 1 & key_S == 1 )
    {
        gimbal_vy = 0 ;
    }
    else if( key_W == 1 & key_S == 0 )
    {
        gimbal_vy = KEY_MOVE_VY_SPEED ;
    }
    else if( key_W == 0 & key_S == 1 )
    {
        gimbal_vy = -KEY_MOVE_VY_SPEED ;
    }
    else
    {
        gimbal_vy = (float)(2 * rc_ch1 ) ;
    }


    if( key_A == 1 & key_D == 1 )
    {
        gimbal_vx = 0 ;
    }
    else if( key_A == 1 & key_D == 0 )
    {
        gimbal_vx = -KEY_MOVE_VX_SPEED ;
    }
    else if( key_A == 0 & key_D == 1 )
    {
        gimbal_vx = KEY_MOVE_VX_SPEED ;
    }
    else
    {
        gimbal_vx = (float)(2 * rc_ch0 ) ;
    }


    if( (HAL_GetTick() - gyro_time) > KEY_CHECK_MIN_TIME )
    {
        if(key_Q == 1)
        {
            gyro_time = HAL_GetTick() ;
            gyro_state++ ;
        }
    }
    //vround_compute


    if (rc_s0 == 1)
    {
        chassis_vround = 3000 ;
    } else
    {
        if((gyro_state % 2) == 1 )
        {
            chassis_vround = 3000 ;
        }
        else
        {
            chassis_vround = 0 ;
        }
    }




}



void gimbal_to_chassis_speed_compute()
{
    chassis_vx = gimbal_vx * (float)cos((double)yaw_radian_difference) - gimbal_vy * (float)sin((double)yaw_radian_difference);
    chassis_vy = gimbal_vx * (float)sin((double)yaw_radian_difference) + gimbal_vy * (float)cos((double)yaw_radian_difference);



}

void yaw_ecd_angle_to_radian()
{

    yaw_angle_difference = (float)(motor_can1_data[4].ecd - YAW_MID_ECD) / (float)(8192 / 360.0f) ;
    yaw_radian_difference = (float)yaw_angle_difference * (float)(M_PI / 180);
}


void chassis_settlement()
{
    CHASSIS_3508_ID1_GIVEN_SPEED = (int16_t)(-chassis_vy + chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID2_GIVEN_SPEED = (int16_t)( chassis_vy +  chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID3_GIVEN_SPEED = (int16_t)( chassis_vy -  chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID4_GIVEN_SPEED = (int16_t)(-chassis_vy - chassis_vx + chassis_vround) ;


//    CHASSIS_3508_ID1_GIVEN_SPEED = 0 ;
//    CHASSIS_3508_ID2_GIVEN_SPEED = 0 ;
//    CHASSIS_3508_ID3_GIVEN_SPEED = 0 ;
//    CHASSIS_3508_ID4_GIVEN_SPEED = 0 ;




}


//loop
void motor_chassis_pid_compute()
{
    CHASSIS_3508_ID1_GIVEN_CURRENT = chassis_3508_id1_speed_pid_loop(CHASSIS_3508_ID1_GIVEN_SPEED);
    CHASSIS_3508_ID2_GIVEN_CURRENT = chassis_3508_id2_speed_pid_loop(CHASSIS_3508_ID2_GIVEN_SPEED);
    CHASSIS_3508_ID3_GIVEN_CURRENT = chassis_3508_id3_speed_pid_loop(CHASSIS_3508_ID3_GIVEN_SPEED);
    CHASSIS_3508_ID4_GIVEN_CURRENT = chassis_3508_id4_speed_pid_loop(CHASSIS_3508_ID4_GIVEN_SPEED);

}



//pid control

//1号电机
void chassis_3508_id1_speed_pid_init(void)
{
    static fp32 chassis_3508_id1_speed_kpkikd[3] = {CHASSIS_3508_ID1_SPEED_PID_KP,CHASSIS_3508_ID1_SPEED_PID_KI,CHASSIS_3508_ID1_SPEED_PID_KD};
    PID_init(&chassis_3508_ID1_speed_pid,PID_POSITION,chassis_3508_id1_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id1_speed_pid_loop(int16_t chassis_3508_ID1_speed_set_loop)
{
    PID_calc(&chassis_3508_ID1_speed_pid, motor_can1_data[0].speed_rpm, chassis_3508_ID1_speed_set_loop);
    int16_t chassis_3508_ID1_given_current_loop = (int16_t)(chassis_3508_ID1_speed_pid.out);

    return chassis_3508_ID1_given_current_loop ;

}



//2号电机
void chassis_3508_id2_speed_pid_init(void)
{
    static fp32 chassis_3508_id2_speed_kpkikd[3] = {CHASSIS_3508_ID2_SPEED_PID_KP,CHASSIS_3508_ID2_SPEED_PID_KI,CHASSIS_3508_ID2_SPEED_PID_KD};
    PID_init(&chassis_3508_ID2_speed_pid,PID_POSITION,chassis_3508_id2_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id2_speed_pid_loop(int16_t chassis_3508_ID2_speed_set_loop)
{
    PID_calc(&chassis_3508_ID2_speed_pid, motor_can1_data[1].speed_rpm, chassis_3508_ID2_speed_set_loop);
    int16_t chassis_3508_ID2_given_current_loop = (int16_t)(chassis_3508_ID2_speed_pid.out);

    return chassis_3508_ID2_given_current_loop ;

}



//3号电机
void chassis_3508_id3_speed_pid_init(void)
{
    static fp32 chassis_3508_id3_speed_kpkikd[3] = {CHASSIS_3508_ID3_SPEED_PID_KP,CHASSIS_3508_ID3_SPEED_PID_KI,CHASSIS_3508_ID3_SPEED_PID_KD};
    PID_init(&chassis_3508_ID3_speed_pid,PID_POSITION,chassis_3508_id3_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id3_speed_pid_loop(int16_t chassis_3508_ID3_speed_set_loop)
{
    PID_calc(&chassis_3508_ID3_speed_pid, motor_can1_data[2].speed_rpm , chassis_3508_ID3_speed_set_loop);
    int16_t chassis_3508_ID3_given_current_loop = (int16_t)(chassis_3508_ID3_speed_pid.out);

    return chassis_3508_ID3_given_current_loop ;

}



//4号电机
void chassis_3508_id4_speed_pid_init(void)
{
    static fp32 chassis_3508_id4_speed_kpkikd[3] = {CHASSIS_3508_ID4_SPEED_PID_KP,CHASSIS_3508_ID4_SPEED_PID_KI,CHASSIS_3508_ID4_SPEED_PID_KD};
    PID_init(&chassis_3508_ID4_speed_pid,PID_POSITION,chassis_3508_id4_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id4_speed_pid_loop(int16_t chassis_3508_ID4_speed_set_loop)
{
    PID_calc(&chassis_3508_ID4_speed_pid, motor_can1_data[3].speed_rpm , chassis_3508_ID4_speed_set_loop);
    int16_t chassis_3508_ID4_given_current_loop = (int16_t)(chassis_3508_ID4_speed_pid.out);

    return chassis_3508_ID4_given_current_loop ;

}
