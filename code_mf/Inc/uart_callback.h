//
// Created by 21481 on 2025/3/18.
//

#ifndef BUBING_RM2025_UART_CALLBACK_H
#define BUBING_RM2025_UART_CALLBACK_H



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




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#endif //BUBING_RM2025_UART_CALLBACK_H
