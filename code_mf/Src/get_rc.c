//
// Created by 21481 on 2025/3/16.
//
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"

void get_rc()
{
    while(1)
    {
        rc_ch0 = local_rc_ctrl->rc.ch[0];
        rc_ch1 = local_rc_ctrl->rc.ch[1];
        rc_ch2 = local_rc_ctrl->rc.ch[2];
        rc_ch3 = local_rc_ctrl->rc.ch[3];
        rc_ch4 = local_rc_ctrl->rc.ch[4];
        rc_s0  = local_rc_ctrl->rc.s[0];
        rc_s1  = local_rc_ctrl->rc.s[1];

        mouse_vx = local_rc_ctrl->mouse.x;
        mouse_vy = local_rc_ctrl->mouse.y;
        mouse_press_l = local_rc_ctrl->mouse.press_l ;
        mouse_press_r = local_rc_ctrl->mouse.press_r ;


        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W)
        {
            key_W = 1 ;
        } else
        {
            key_W = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S)
        {
            key_S = 1 ;
        } else
        {
            key_S = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A)
        {
            key_A = 1 ;
        } else
        {
            key_A = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)
        {
            key_D = 1 ;
        } else
        {
            key_D = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)
        {
            key_SHIFT = 1 ;
        } else
        {
            key_SHIFT = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)
        {
            key_CTRL = 1 ;
        } else
        {
            key_CTRL = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)
        {
            key_Q = 1 ;
        } else
        {
            key_Q = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)
        {
            key_E = 1 ;
        } else
        {
            key_E = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_R)
        {
            key_R = 1 ;
        } else
        {
            key_R = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)
        {
            key_F = 1 ;
        } else
        {
            key_F = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
        {
            key_G = 1 ;
        } else
        {
            key_G = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Z)
        {
            key_Z = 1 ;
        } else
        {
            key_Z = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X)
        {
            key_X = 1 ;
        } else
        {
            key_X = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)
        {
            key_C = 1 ;
        } else
        {
            key_C = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
        {
            key_V = 1 ;
        } else
        {
            key_V = 0 ;
        }
        if(local_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
        {
            key_B = 1 ;
        } else
        {
            key_B = 0 ;
        }
        osDelay(1);

    }
}


