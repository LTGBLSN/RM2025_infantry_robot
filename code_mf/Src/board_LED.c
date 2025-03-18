//
// Created by 21481 on 2025/3/16.
//
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

void board_LED()
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
        osDelay(100);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
        osDelay(100);
        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
        osDelay(100);
        osDelay(1);
    }



}


