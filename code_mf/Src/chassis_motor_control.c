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

pid_type_def chassis_follow_gimbal_pid;



void chassis_motor_control()
{
    while (1)
    {
//        CHASSIS_FOLLOW_GIMBAL_GIVEN_SPEED = chassis_follow_gimbal_pid_loop(YAW_MID_ECD);//底盘跟随
        rc_to_gimbal_speed_compute();//遥控器转换为云台速度
        yaw_ecd_angle_to_radian();
        gimbal_to_chassis_speed_compute();//底盘速度解算
        chassis_settlement();//轮系速度解算
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
        gimbal_vy = 2*(float)(2 * rc_ch1 ) ;
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
        gimbal_vx = 2*(float)(2 * rc_ch0 ) ;
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
        chassis_vround = -1000 ;
    } else if(rc_s0 == 3)
    {
        if((gyro_state % 2) == 1 )
        {
            chassis_vround = -1000 ;
        }
        else
        {

            chassis_vround = CHASSIS_FOLLOW_GIMBAL_GIVEN_SPEED ;
        }

    }




}

void yaw_ecd_angle_to_radian()
{

    yaw_angle_difference = (float)(motor_can1_data[4].ecd - YAW_MID_ECD) / (float)(8192 / 360.0f) ;
    yaw_radian_difference = (float)yaw_angle_difference * (float)(M_PI / 180);
}



void gimbal_to_chassis_speed_compute()
{
    chassis_vx = gimbal_vx * (float)cos((double)yaw_radian_difference) - gimbal_vy * (float)sin((double)yaw_radian_difference);
    chassis_vy = gimbal_vx * (float)sin((double)yaw_radian_difference) + gimbal_vy * (float)cos((double)yaw_radian_difference);



}




void chassis_settlement()
{


    //先进行普通目标速度计算
    CHASSIS_3508_ID1_COMPUTE_SPEED = (int16_t)(-chassis_vy + chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID2_COMPUTE_SPEED = (int16_t)( chassis_vy +  chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID3_COMPUTE_SPEED = (int16_t)( chassis_vy -  chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID4_COMPUTE_SPEED = (int16_t)(-chassis_vy - chassis_vx + chassis_vround) ;

    //计算总功率
    CHASSIS_3508_ALL_COMPUTE_SPEED =
            fabsf((float)CHASSIS_3508_ID1_COMPUTE_SPEED) +
            fabsf((float)CHASSIS_3508_ID2_COMPUTE_SPEED) +
            fabsf((float)CHASSIS_3508_ID3_COMPUTE_SPEED) +
            fabsf((float)CHASSIS_3508_ID4_COMPUTE_SPEED) ;

    beyond_power = ( CHASSIS_3508_ALL_COMPUTE_SPEED - (float)robot_max_power ) ;//计算超出总功率,大于0为超功率

    if( beyond_power > 0 )
    {
        chassis_power_state = 1 ;
        CHASSIS_3508_ID1_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID1_COMPUTE_SPEED - (0.25f * beyond_power)) ;//速度赋值
        CHASSIS_3508_ID2_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID2_COMPUTE_SPEED - (0.25f * beyond_power)) ;
        CHASSIS_3508_ID3_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID3_COMPUTE_SPEED - (0.25f * beyond_power)) ;
        CHASSIS_3508_ID4_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID4_COMPUTE_SPEED - (0.25f * beyond_power)) ;

    }
    else
    {
        chassis_power_state = 0 ;
        CHASSIS_3508_ID1_GIVEN_SPEED = CHASSIS_3508_ID1_COMPUTE_SPEED ;
        CHASSIS_3508_ID2_GIVEN_SPEED = CHASSIS_3508_ID2_COMPUTE_SPEED ;
        CHASSIS_3508_ID3_GIVEN_SPEED = CHASSIS_3508_ID3_COMPUTE_SPEED ;
        CHASSIS_3508_ID4_GIVEN_SPEED = CHASSIS_3508_ID4_COMPUTE_SPEED ;
    }


    send_out_all_speed =
            fabsf((float)CHASSIS_3508_ID1_GIVEN_SPEED) +
            fabsf((float)CHASSIS_3508_ID2_GIVEN_SPEED) +
            fabsf((float)CHASSIS_3508_ID3_GIVEN_SPEED) +
            fabsf((float)CHASSIS_3508_ID4_GIVEN_SPEED) ;


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


//CHASSIS_FOLLOW_GIMBAL_ANGLE_PID
void chassis_follow_gimbal_angle_pid_init(void)
{
    static fp32 chassis_follow_gimbal_angle_kpkikd[3] = {CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KP,CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KI,CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KD};
    PID_init(&chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_angle_kpkikd, CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_OUT_MAX, CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KI_MAX);

}

float chassis_follow_gimbal_pid_loop(float PITCH_6020_ID2_angle_set_loop)
{
    PID_calc(&chassis_follow_gimbal_pid, motor_can1_data[4].ecd , PITCH_6020_ID2_angle_set_loop);
    float chassis_follow_gimbal_angle_loop = (float)(chassis_follow_gimbal_pid.out);

    return chassis_follow_gimbal_angle_loop ;

}


