/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

const RC_ctrl_t *local_rc_ctrl ;//遥控器数据存储空间
int16_t rc_ch0 ;
int16_t rc_ch1 ;
int16_t rc_ch2 ;
int16_t rc_ch3 ;
int16_t rc_ch4 ;
int16_t rc_s0 ;
int16_t rc_s1 ;
int16_t key_W ;
int16_t key_S ;
int16_t key_A ;
int16_t key_D ;
int16_t key_SHIFT ;
int16_t key_CTRL ;
int16_t key_Q ;
int16_t key_E ;
int16_t key_R ;
int16_t key_F ;
int16_t key_G ;
int16_t key_Z ;
int16_t key_X ;
int16_t key_C ;
int16_t key_V ;
int16_t key_B ;



int16_t rc_receive_state ;//遥控器状态 0为离线，1为在线
uint32_t rc_receive_time ;//遥控器接收到数据的时间戳

int16_t imu_receive_state ;//IMU状态 0为离线，1为在线
uint32_t imu_receive_time ;//IMU接收到数据的时间戳

uint8_t uart1_receive_data ;//串口当前接收字节

int16_t yaw_6020_state ;//6020电机状态 0为错误，1为正常
int16_t pitch_6020_state ;//6020电机状态 0为错误，1为正常




//chassis
int16_t CHASSIS_3508_ID1_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID1_GIVEN_CURRENT ;

int16_t CHASSIS_3508_ID2_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID2_GIVEN_CURRENT ;

int16_t CHASSIS_3508_ID3_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID3_GIVEN_CURRENT ;

int16_t CHASSIS_3508_ID4_GIVEN_SPEED ;
int16_t CHASSIS_3508_ID4_GIVEN_CURRENT ;


//gimbal
float YAW_6020_ID1_GIVEN_SPEED ;
int16_t YAW_6020_ID1_GIVEN_CURRENT ;

float PITCH_6020_ID2_GIVEN_ANGLE ;
float PITCH_6020_ID2_GIVEN_SPEED ;
int16_t PITCH_6020_ID2_GIVEN_CURRENT ;
float pitch_motor_mean_speed ;
int16_t pitch_motor_speed_last_data [3] ;


//friction wheel
int16_t FRICTION_WHEEL_3510_ID1_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT ;

int16_t FRICTION_WHEEL_3510_ID2_GIVEN_SPEED ;
int16_t FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT ;











/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_UART_Receive_DMA(&huart1, &uart1_receive_data, 1);  //串口2接收数据中断

    remote_control_init();//遥控器初始化

    local_rc_ctrl = get_remote_control_point();//遥控器初始化

    can_filter_init();//can通讯初始化

    //底盘电机初始化
    chassis_3508_id1_speed_pid_init();
    chassis_3508_id2_speed_pid_init();
    chassis_3508_id3_speed_pid_init();
    chassis_3508_id4_speed_pid_init();

    //云台电机初始化
    yaw_speed_pid_init();//yaw速度环pid初始化
    pitch_speed_pid_init();//pitch速度环pid初始化
    pitch_angle_pid_init();//pitch角度环pid初始化

    //摩擦轮电机初始化
    friction_wheel_3510_id1_speed_pid_init();//摩擦轮id1速度环初始化
    friction_wheel_3510_id2_speed_pid_init();//摩擦轮id2速度环初始化


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
