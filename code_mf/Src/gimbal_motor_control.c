//
// Created by 21481 on 2025/3/19.
//
#include "main.h"
#include "cmsis_os.h"

#include "CAN_receive.h"
#include "pid.h"
#include "gimbal_motor_control.h"
#include "shoot_control.h"

pid_type_def yaw_6020_ID1_speed_pid;
pid_type_def yaw_6020_ID1_angle_pid;

pid_type_def pitch_6020_ID2_speed_pid;
pid_type_def pitch_6020_ID2_angle_pid;


pid_type_def friction_wheel_3510_ID1_speed_pid;
pid_type_def friction_wheel_3510_ID2_speed_pid;



pid_type_def shoot_2006_ID3_speed_pid;



 void gimbal_motor_control()
{
    while (1)
    {


        friction_wheel_speed_control();//摩擦轮目标速度控制
        friction_wheel_pid_control();//摩擦轮pid控制


        yaw_imu_getAbscissa() ;//更新陀螺仪总角度

        motor_gimbal_angle_compute();//目标角度控制
        motor_gimbal_pid_compute();//云台pid控制


        shoot_pid_control();//拨弹盘pid控制






        osDelay((1/GIMBAL_PID_COMPUTE_FREQUENCY)*1000);//控制频率

#if GIMBAL_PID_COMPUTE_FREQUENCY == 0
        osDelay(1);

#endif


    }
}



void motor_gimbal_angle_compute()
{
     if(mouse_vy == 0 & mouse_vx == 0)
     {
         if((PITCH_RC_IN_KP * (float )rc_ch3) < 0 )
         {
             if(PITCH_6020_ID2_GIVEN_ANGLE < -23.0f )
             {
                 PITCH_6020_ID2_GIVEN_ANGLE = -23.0f ;
             }
             else
             {
                 PITCH_6020_ID2_GIVEN_ANGLE = PITCH_6020_ID2_GIVEN_ANGLE + PITCH_RC_IN_KP * (float )rc_ch3  ;
             }
         }
         else
         {
             if(PITCH_6020_ID2_GIVEN_ANGLE > 25.0f )
             {
                 PITCH_6020_ID2_GIVEN_ANGLE = 25.0f ;
             }
             else
             {
                 PITCH_6020_ID2_GIVEN_ANGLE = PITCH_6020_ID2_GIVEN_ANGLE + PITCH_RC_IN_KP * (float )rc_ch3  ;
             }
         }


         YAW_6020_ID1_GIVEN_ANGLE = YAW_6020_ID1_GIVEN_ANGLE + (YAW_RC_IN_KP * (float)rc_ch2) ;

     } else//键盘还没写
     {
//         PITCH_6020_ID2_GIVEN_SPEED = MOUSE_VY_SPEED_SCALING_FACTOR * (float)-mouse_vy - PITCH_OFF_FRICTION_STOP_SPEED_COMPENSATE;
//         YAW_6020_ID1_GIVEN_SPEED = MOUSE_VX_SPEED_SCALING_FACTOR * (float)-mouse_vx ;


     }

}


void yaw_imu_getAbscissa()
{
    if((YAW_IMU_LAST_ECD - yaw_angle_from_bmi088) > 180.0f)
    {

        YAW_IMU_LAPS++ ;

    }
    if((yaw_angle_from_bmi088 - YAW_IMU_LAST_ECD) > 180.0f)
    {

        YAW_IMU_LAPS-- ;

    }

    YAW_IMU_LAST_ECD = yaw_angle_from_bmi088 ;

    YAW_IMU_ABSCISSA = 360.0f * YAW_IMU_LAPS + yaw_angle_from_bmi088 ;


}


void motor_gimbal_pid_compute()
{
    YAW_6020_ID1_GIVEN_SPEED = yaw_angle_pid_loop(YAW_6020_ID1_GIVEN_ANGLE) ;
    YAW_6020_ID1_GIVEN_CURRENT = (int16_t)yaw_speed_pid_loop(YAW_6020_ID1_GIVEN_SPEED);//速度环


    PITCH_6020_ID2_GIVEN_SPEED = pitch_angle_from_bmi088_pid_loop(PITCH_6020_ID2_GIVEN_ANGLE);//角度环
    PITCH_6020_ID2_GIVEN_CURRENT = (int16_t) (- pitch_speed_from_bmi088_pid_loop(PITCH_6020_ID2_GIVEN_SPEED)); //速度环

}



void friction_wheel_speed_control()
{
    if( rc_s1 == 2 )
    {
        FRICTION_WHEEL_3510_ID1_GIVEN_SPEED = 0 ;
        FRICTION_WHEEL_3510_ID2_GIVEN_SPEED = 0 ;
    } else
    {
        FRICTION_WHEEL_3510_ID1_GIVEN_SPEED = FRICTION_WHEEL_SHOOT_SPEED ;
        FRICTION_WHEEL_3510_ID2_GIVEN_SPEED = -FRICTION_WHEEL_SHOOT_SPEED ;

    }

}
void friction_wheel_pid_control()
{
    FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT = friction_wheel_3510_id1_speed_pid_loop(FRICTION_WHEEL_3510_ID1_GIVEN_SPEED);
     FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT = friction_wheel_3510_id2_speed_pid_loop(FRICTION_WHEEL_3510_ID2_GIVEN_SPEED);
}










void pitch_motor_mean_speed_compute()//弃用，滞后性有点大
{
     pitch_motor_mean_speed =
             0.5f * (float)motor_can2_data[5].speed_rpm +
             0.3f * (float)pitch_motor_speed_last_data[0]+
             0.15f * (float)pitch_motor_speed_last_data[1]+
             0.05f * (float)pitch_motor_speed_last_data[2] ;

    pitch_motor_speed_last_data[2] = pitch_motor_speed_last_data[1] ;
    pitch_motor_speed_last_data[1] = pitch_motor_speed_last_data[0] ;
    pitch_motor_speed_last_data[0] = motor_can2_data[5].speed_rpm ;

}



void yaw_angle_pid_init(void)
{
    static fp32 yaw_6020_id1_angle_kpkikd[3] = {YAW_6020_ID2_ANGLE_PID_KP, YAW_6020_ID2_ANGLE_PID_KI, YAW_6020_ID2_ANGLE_PID_KD};
    PID_init(&yaw_6020_ID1_angle_pid, PID_POSITION, yaw_6020_id1_angle_kpkikd, YAW_6020_ID2_ANGLE_PID_OUT_MAX, YAW_6020_ID2_ANGLE_PID_KI_MAX);

}

float yaw_angle_pid_loop(float YAW_6020_ID1_angle_set_loop)
{
    PID_calc(&yaw_6020_ID1_angle_pid, YAW_IMU_ABSCISSA , YAW_6020_ID1_angle_set_loop);
    float yaw_6020_ID1_given_speed_loop = (float)(yaw_6020_ID1_angle_pid.out);

    return yaw_6020_ID1_given_speed_loop ;

}


void yaw_speed_pid_init(void)
{
    static fp32 yaw_6020_id1_speed_kpkikd[3] = {YAW_6020_ID2_SPEED_PID_KP, YAW_6020_ID2_SPEED_PID_KI, YAW_6020_ID2_SPEED_PID_KD};
    PID_init(&yaw_6020_ID1_speed_pid, PID_POSITION, yaw_6020_id1_speed_kpkikd, YAW_6020_ID2_SPEED_PID_OUT_MAX, YAW_6020_ID2_SPEED_PID_KI_MAX);

}

float yaw_speed_pid_loop(float YAW_6020_ID1_speed_set_loop)
{
    PID_calc(&yaw_6020_ID1_speed_pid, yaw_speed_from_bmi088 , YAW_6020_ID1_speed_set_loop);
    int16_t yaw_6020_ID1_given_current_loop = (int16_t)(yaw_6020_ID1_speed_pid.out);

    return yaw_6020_ID1_given_current_loop ;

}





void pitch_speed_from_bmi88_pid_init(void)
{
    static fp32 pitch_6020_id2_speed_kpkikd[3] = {PITCH_6020_ID2_SPEED_PID_KP,PITCH_6020_ID2_SPEED_PID_KI,PITCH_6020_ID2_SPEED_PID_KD};
    PID_init(&pitch_6020_ID2_speed_pid,PID_POSITION,pitch_6020_id2_speed_kpkikd,PITCH_6020_ID2_SPEED_PID_OUT_MAX,PITCH_6020_ID2_SPEED_PID_KI_MAX);

}

float pitch_speed_from_bmi088_pid_loop(float PITCH_6020_ID2_speed_set_loop)
{
    PID_calc(&pitch_6020_ID2_speed_pid, pitch_speed_from_bmi088 , PITCH_6020_ID2_speed_set_loop);
    int16_t pitch_6020_ID2_given_current_loop = (int16_t)(pitch_6020_ID2_speed_pid.out);

    return pitch_6020_ID2_given_current_loop ;

}


void pitch_angle_pid_init(void)
{
    static fp32 pitch_6020_id2_angle_kpkikd[3] = {PITCH_6020_ID2_ANGLE_PID_KP,PITCH_6020_ID2_ANGLE_PID_KI,PITCH_6020_ID2_ANGLE_PID_KD};
    PID_init(&pitch_6020_ID2_angle_pid,PID_POSITION,pitch_6020_id2_angle_kpkikd,PITCH_6020_ID2_ANGLE_PID_OUT_MAX,PITCH_6020_ID2_ANGLE_PID_KI_MAX);

}

float pitch_angle_from_bmi088_pid_loop(float PITCH_6020_ID2_angle_set_loop)
{
    PID_calc(&pitch_6020_ID2_angle_pid, pitch_angle_from_bmi088 , PITCH_6020_ID2_angle_set_loop);
    float pitch_6020_ID2_given_speed_loop = (float)(pitch_6020_ID2_angle_pid.out);

    return pitch_6020_ID2_given_speed_loop ;

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



void friction_wheel_3510_id2_speed_pid_init(void)
{
    static fp32 friction_wheel_3510_id2_speed_kpkikd[3] = {FRICTION_WHEEL_3510_ID2_SPEED_PID_KP,FRICTION_WHEEL_3510_ID2_SPEED_PID_KI,FRICTION_WHEEL_3510_ID2_SPEED_PID_KD};
    PID_init(&friction_wheel_3510_ID2_speed_pid,PID_POSITION,friction_wheel_3510_id2_speed_kpkikd,FRICTION_WHEEL_3510_ID2_SPEED_PID_OUT_MAX,FRICTION_WHEEL_3510_ID2_SPEED_PID_KI_MAX);

}

int16_t friction_wheel_3510_id2_speed_pid_loop(int16_t friction_wheel_3510_id2_speed_set_loop)
{
    PID_calc(&friction_wheel_3510_ID2_speed_pid, motor_can2_data[1].speed_rpm , friction_wheel_3510_id2_speed_set_loop);
    int16_t friction_wheel_3510_id2_given_current_loop = (int16_t)(friction_wheel_3510_ID2_speed_pid.out);

    return friction_wheel_3510_id2_given_current_loop ;

}










