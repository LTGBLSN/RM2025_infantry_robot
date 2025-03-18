#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
extern UART_HandleTypeDef huart6;
void usart6_printf(const char *fmt, ...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    HAL_UART_Transmit(&huart6,tx_buf, len,100);//在这修改发送的串口，包括上面的‘extern’后面的

}

