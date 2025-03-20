#ifndef __JY61P_H
#define __JY61P_H

#include "main.h" 

void jy61p_Receive_Angle_Data(uint8_t RxData);

void jy61p_Receive_Speed_Data(uint8_t RxData);

extern float PITCH_IMU_ANGLE;
extern float YAW_IMU_ANGLE;
extern float ROLL_IMU_ANGLE;

extern float PITCH_IMU_SPEED;
extern float YAW_IMU_SPEED;
extern float ROLL_IMU_SPEED;


#endif
