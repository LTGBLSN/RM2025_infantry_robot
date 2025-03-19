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



extern int16_t rc_receive_state ;//ң����״̬ 0Ϊ���ߣ�1Ϊ����

extern uint32_t rc_receive_time ;//ң�������յ����ݵ�ʱ���

extern uint16_t rx_data[55] ;//���ڽ��մ洢�ռ�

extern uint8_t g_usart2_receivedata ;//���ڵ�ǰ�����ֽ�



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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
