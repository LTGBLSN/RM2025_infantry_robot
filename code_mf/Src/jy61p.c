#include "jy61p.h"

//angle
static uint8_t Rx_Angle_Buffer[11];/*接收数据数组*/
static volatile uint8_t AngleRxState = 0;/*接收状态标志位*/
static uint8_t RxAngleIndex = 0;/*接受数组索引*/

float PITCH_IMU_ANGLE;
float YAW_IMU_ANGLE;
float ROLL_IMU_ANGLE;


//speed
static uint8_t Rx_Speed_Buffer[11];/*接收数据数组*/
static volatile uint8_t SpeedRxState = 0;/*接收状态标志位*/
static uint8_t RxSpeedIndex = 0;/*接受数组索引*/

float PITCH_IMU_SPEED;
float YAW_IMU_SPEED;
float ROLL_IMU_SPEED;



/**
 * @brief       数据包处理函数，解角度
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
            Rx_Angle_Buffer[RxAngleIndex] = RxData;
            AngleRxState = 1;
            RxAngleIndex = 1; //进入下一状态
		}
	}

	else if (AngleRxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
            Rx_Angle_Buffer[RxAngleIndex] = RxData;
            AngleRxState = 2;
            RxAngleIndex = 2; //进入下一状态
		}
	}

	else if (AngleRxState == 2)	//接收数据
	{
        Rx_Angle_Buffer[RxAngleIndex++] = RxData;
		if(RxAngleIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + Rx_Angle_Buffer[i]; //计算校验和
			}
			if(sum == Rx_Angle_Buffer[10])		//校验成功
			{
//				/*计算数据，根据数据内容选择对应的计算公式360度*/
//				PITCH_IMU_ANGLE = ((uint16_t) ((uint16_t) Rx_Angle_Buffer[3] << 8 | (uint16_t) Rx_Angle_Buffer[2])) / 32768.0f * 180.0f;
//                ROLL_IMU_ANGLE = ((uint16_t) ((uint16_t) Rx_Angle_Buffer[5] << 8 | (uint16_t) Rx_Angle_Buffer[4])) / 32768.0f * 180.0f;
//                YAW_IMU_ANGLE = ((uint16_t) ((uint16_t) Rx_Angle_Buffer[7] << 8 | (uint16_t) Rx_Angle_Buffer[6])) / 32768.0f * 180.0f;
                /*计算数据，根据数据内容选择对应的计算公式180度*/
                PITCH_IMU_ANGLE = ((short)((short)Rx_Angle_Buffer[3] << 8 | Rx_Angle_Buffer[2])) / 32768.0f * 180.0f;
                ROLL_IMU_ANGLE = ((short)((short)Rx_Angle_Buffer[5] << 8 | Rx_Angle_Buffer[4])) / 32768.0f * 180.0f;
                YAW_IMU_ANGLE = ((short)((short)Rx_Angle_Buffer[7] << 8 | Rx_Angle_Buffer[6])) / 32768.0f * 180.0f;



            }
            AngleRxState = 0;
            RxAngleIndex = 0; //读取完成，回到最初状态，等待包头
		}
	}
}


/**
 * @brief       数据包处理函数,解速度
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_Receive_Speed_Data(uint8_t RxData)
{
    uint8_t i,sum=0;

    if (SpeedRxState == 0)	//等待包头
    {
        if (RxData == 0x55)	//收到包头
        {
            Rx_Speed_Buffer[RxSpeedIndex] = RxData;
            SpeedRxState = 1;
            RxSpeedIndex = 1; //进入下一状态
        }
    }

    else if (SpeedRxState == 1)
    {
        if (RxData == 0x52)	/*判断数据内容，修改这里可以改变要读的数据内容，0x52为角速度输出*/
        {
            Rx_Speed_Buffer[RxSpeedIndex] = RxData;
            SpeedRxState = 2;
            RxSpeedIndex = 2; //进入下一状态
        }
    }

    else if (SpeedRxState == 2)	//接收数据
    {
        Rx_Speed_Buffer[RxSpeedIndex++] = RxData;
        if(RxSpeedIndex == 11)	//接收完成
        {
            for(i=0;i<10;i++)
            {
                sum = sum + Rx_Speed_Buffer[i]; //计算校验和
            }
            if(sum == Rx_Speed_Buffer[10])		//校验成功
            {

                /*计算数据，根据数据内容选择对应的计算公式*/
                PITCH_IMU_SPEED = ((short)((short)Rx_Speed_Buffer[3] << 8 | Rx_Speed_Buffer[2])) / 32768.0f * 2000.0f;
                ROLL_IMU_SPEED = ((short)((short)Rx_Speed_Buffer[5] << 8 | Rx_Speed_Buffer[4])) / 32768.0f * 2000.0f;
                YAW_IMU_SPEED = ((short)((short)Rx_Speed_Buffer[7] << 8 | Rx_Speed_Buffer[6])) / 32768.0f * 2000.0f;




            }
            SpeedRxState = 0;
            RxSpeedIndex = 0; //读取完成，回到最初状态，等待包头
        }
    }
}

