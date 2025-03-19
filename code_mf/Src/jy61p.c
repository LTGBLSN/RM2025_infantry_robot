#include "jy61p.h"

static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t AngleRxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/


float PITCH_ANGLE;/*角度信息，如果只需要整数可以改为整数类型*/
float YAW_ANGLE;/*角度信息，如果只需要整数可以改为整数类型*/
float ROLL_ANGLE;/*角度信息，如果只需要整数可以改为整数类型*/



/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_Receive_Angle_Data(uint8_t RxData)
{
	uint8_t i,sum=0;
	
	if (AngleRxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer[RxIndex] = RxData;
            AngleRxState = 1;
			RxIndex = 1; //进入下一状态
		}
	}
	
	else if (AngleRxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
            AngleRxState = 2;
			RxIndex = 2; //进入下一状态
		}
	}
	
	else if (AngleRxState == 2)	//接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i]; //计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
//				/*计算数据，根据数据内容选择对应的计算公式360度*/
//				PITCH_ANGLE = ((uint16_t) ((uint16_t) RxBuffer[3] << 8 | (uint16_t) RxBuffer[2])) / 32768.0f * 180.0f;
//                ROLL_ANGLE = ((uint16_t) ((uint16_t) RxBuffer[5] << 8 | (uint16_t) RxBuffer[4])) / 32768.0f * 180.0f;
//                YAW_ANGLE = ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;
                /*计算数据，根据数据内容选择对应的计算公式*/
                PITCH_ANGLE = ((short)((short)RxBuffer[3] << 8 | RxBuffer[2])) / 32768.0f * 180.0f;
                ROLL_ANGLE = ((short)((short)RxBuffer[5] << 8 | RxBuffer[4])) / 32768.0f * 180.0f;
                YAW_ANGLE = ((short)((short)RxBuffer[7] << 8 | RxBuffer[6])) / 32768.0f * 180.0f;

            }
            AngleRxState = 0;
			RxIndex = 0; //读取完成，回到最初状态，等待包头
		}
	}
}

