/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId board_LED_taskHandle;
osThreadId uart_sent_testHandle;
osThreadId get_rc_taskHandle;
osThreadId can_sent_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void board_LED(void const * argument);
void uart_sent(void const * argument);
void get_rc(void const * argument);
void can_sent(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of board_LED_task */
  osThreadDef(board_LED_task, board_LED, osPriorityIdle, 0, 128);
  board_LED_taskHandle = osThreadCreate(osThread(board_LED_task), NULL);

  /* definition and creation of uart_sent_test */
  osThreadDef(uart_sent_test, uart_sent, osPriorityIdle, 0, 256);
  uart_sent_testHandle = osThreadCreate(osThread(uart_sent_test), NULL);

  /* definition and creation of get_rc_task */
  osThreadDef(get_rc_task, get_rc, osPriorityIdle, 0, 128);
  get_rc_taskHandle = osThreadCreate(osThread(get_rc_task), NULL);

  /* definition and creation of can_sent_task */
  osThreadDef(can_sent_task, can_sent, osPriorityIdle, 0, 128);
  can_sent_taskHandle = osThreadCreate(osThread(can_sent_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_board_LED */
/**
* @brief Function implementing the board_LED_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_board_LED */
__weak void board_LED(void const * argument)
{
  /* USER CODE BEGIN board_LED */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END board_LED */
}

/* USER CODE BEGIN Header_uart_sent */
/**
* @brief Function implementing the uart_sent_test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_sent */
__weak void uart_sent(void const * argument)
{
  /* USER CODE BEGIN uart_sent */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_sent */
}

/* USER CODE BEGIN Header_get_rc */
/**
* @brief Function implementing the get_rc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_get_rc */
__weak void get_rc(void const * argument)
{
  /* USER CODE BEGIN get_rc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END get_rc */
}

/* USER CODE BEGIN Header_can_sent */
/**
* @brief Function implementing the can_sent_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_sent */
__weak void can_sent(void const * argument)
{
  /* USER CODE BEGIN can_sent */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can_sent */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
