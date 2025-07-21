//
// Created by 21481 on 2025/3/17.
//
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "error_detection.h"
#include "can_sent.h"

void can_sent()
{
    while (1)
    {
        if( rc_receive_state == RC_OFFLINE | yaw_6020_state == GM6020_DIE | pitch_6020_state == GM6020_DIE)//遥控器离线，全车断电
        {
            can_cmd_all(0,0,0,0,0,0,0,0,0);
        }
        else//遥控器在线，正常进行开关控制
        {
            if(rc_s0 == 2)
            {
                can_cmd_all(0,0,0,0,0,0,0,0,0);
            }
            else if(rc_s0 == 3 | rc_s0 == 1)//运动模式
            {
//                can_cmd_all(0,
//                            0,
//                            0,
//                            0,
//                            0,
//                            PITCH_6020_ID2_GIVEN_CURRENT,
//                            0,
//                            0,
//                            0);
                can_cmd_all(CHASSIS_3508_ID1_GIVEN_CURRENT,
                            CHASSIS_3508_ID2_GIVEN_CURRENT,
                            CHASSIS_3508_ID3_GIVEN_CURRENT,
                            CHASSIS_3508_ID4_GIVEN_CURRENT,
                            YAW_6020_ID1_GIVEN_CURRENT,
                            PITCH_6020_ID2_GIVEN_CURRENT,
                            FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT,
                            FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT,
                            SHOOT_2006_ID3_GIVEN_CURRENT);
            }

            else//遥控器数据初始化中或错误，全车断电
            {
                can_cmd_all(0,0,0,0,0,0,0,0,0);
            }

        }

        osDelay(1);
    }

}


void can_cmd_all(int16_t chassis_id1 ,       int16_t chassis_id2 ,
                 int16_t chassis_id3 ,       int16_t chassis_id4 ,
                 int16_t yaw_id1 ,           int16_t pitch_id2 ,
                 int16_t friction_wheel_id1, int16_t friction_wheel_id2 ,
                 int16_t shoot_id3 )
{
    CAN2_cmd_pitch(pitch_id2,0,0,0);
    CAN2_cmd_friction_wheels(friction_wheel_id1,friction_wheel_id2,shoot_id3,0);
    CAN1_cmd_chassis(chassis_id1, chassis_id2, chassis_id3, chassis_id4);
    CAN1_cmd_yaw(yaw_id1,0,0,0);

}



