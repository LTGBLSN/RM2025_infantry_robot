/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const RC_ctrl_t *local_rc_ctrl ;
extern int16_t rc_ch0 ;
extern int16_t rc_ch1 ;
extern int16_t rc_ch2 ;
extern int16_t rc_ch3 ;
extern int16_t rc_ch4 ;
extern int16_t rc_s0 ;
extern int16_t rc_s1 ;
extern int16_t key_W ;
extern int16_t key_S ;
extern int16_t key_A ;
extern int16_t key_D ;
extern int16_t key_SHIFT ;
extern int16_t key_CTRL ;
extern int16_t key_Q ;
extern int16_t key_E ;
extern int16_t key_R ;
extern int16_t key_F ;
extern int16_t key_G ;
extern int16_t key_Z ;
extern int16_t key_X ;
extern int16_t key_C ;
extern int16_t key_V ;
extern int16_t key_B ;


extern int16_t mouse_vx ;
extern int16_t mouse_vy ;
extern int16_t mouse_press_l ;
extern int16_t mouse_press_r ;



extern int16_t rc_receive_state ;//遥控器状态 0为离线，1为在线
extern uint32_t rc_receive_time ;//遥控器接收到数据的时间戳

extern uint8_t uart1_receive_data ;//串口当前接收字节

extern int16_t yaw_6020_state ;//6020状态 0为错误，1为正常
extern int16_t pitch_6020_state ;//6020电机状态 0为错误，1为正常

extern float gimbal_vx ;
extern float gimbal_vy ;

extern float chassis_vx ;
extern float chassis_vy ;
extern float chassis_vround ;


extern float yaw_angle_difference ;
extern float yaw_radian_difference ;

extern uint32_t gyro_time ;
extern int16_t gyro_state ;


//chassis
extern int16_t CHASSIS_3508_ID1_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID1_GIVEN_CURRENT;
extern int16_t CHASSIS_3508_ID1_VXY_COMPUTE_SPEED ;

extern int16_t CHASSIS_3508_ID2_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID2_GIVEN_CURRENT;
extern int16_t CHASSIS_3508_ID2_VXY_COMPUTE_SPEED ;

extern int16_t CHASSIS_3508_ID3_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID3_GIVEN_CURRENT;
extern int16_t CHASSIS_3508_ID3_VXY_COMPUTE_SPEED ;

extern int16_t CHASSIS_3508_ID4_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID4_GIVEN_CURRENT;
extern int16_t CHASSIS_3508_ID4_VXY_COMPUTE_SPEED ;

extern float CHASSIS_3508_ALL_COMPUTE_SPEED ;

extern float CHASSIS_FOLLOW_GIMBAL_GIVEN_SPEED ;

extern float beyond_power ;

extern int16_t chassis_power_state ;

//gimbal_vx
extern float YAW_6020_ID1_GIVEN_SPEED ;
extern int16_t YAW_6020_ID1_GIVEN_CURRENT ;
extern float YAW_6020_ID1_GIVEN_ANGLE ;

extern float PITCH_6020_ID2_GIVEN_ANGLE ;
extern float PITCH_6020_ID2_GIVEN_SPEED ;
extern int16_t PITCH_6020_ID2_GIVEN_CURRENT ;
extern float pitch_motor_mean_speed ;
extern int16_t pitch_motor_speed_last_data [3] ;

//friction wheel
extern int16_t FRICTION_WHEEL_3510_ID1_GIVEN_SPEED ;
extern int16_t FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT ;

extern int16_t FRICTION_WHEEL_3510_ID2_GIVEN_SPEED ;
extern int16_t FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT ;



//shoot
extern float SHOOT_2006_ID3_GIVEN_SPEED ;
extern int16_t SHOOT_2006_ID3_GIVEN_CURRENT ;

extern int16_t robot_level ;
extern uint32_t robot_level_time ;

extern float robot_max_power ;

extern float send_out_all_speed ;

extern uint32_t servo_time ;
extern uint32_t servo_rc_time ;
extern uint32_t servo_state ;


extern float gyro[3];
extern float acce[3];
extern float temp;
extern float INS_quat[4] ;
extern float INS_angle[3] ;
extern float INS_degree[3] ;

extern float pitch_speed_from_bmi088 ;
extern float yaw_speed_from_bmi088 ;
extern float roll_speed_from_bmi088 ;


extern float pitch_angle_from_bmi088 ;
extern float yaw_angle_from_bmi088 ;
extern float roll_angle_from_bmi088 ;

extern float YAW_IMU_LAST_ECD ;
extern float YAW_IMU_LAPS ;
extern float YAW_IMU_ABSCISSA ;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_GRYO_Pin GPIO_PIN_5
#define INT1_GRYO_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
