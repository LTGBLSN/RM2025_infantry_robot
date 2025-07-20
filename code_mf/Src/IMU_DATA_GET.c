//
// Created by 21481 on 2025/7/20.
//
#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "IMU_DATA_GET.h"
#include "MahonyAHRS.h"
#include "math.h"

void IMU_DATA_GET()
{
    while (1)
    {
        BMI088_read(gyro,acce,&temp);
        roll_speed_from_bmi088 = gyro[0];
        pitch_speed_from_bmi088 = gyro[1];
        yaw_speed_from_bmi088 = gyro[2];


        MahonyAHRSupdate(INS_quat,gyro[0],gyro[1],gyro[2],acce[0],acce[1],acce[2],0.0f,0.0f,0.0f);
        get_angle(INS_quat,&INS_angle[0],&INS_angle[1],&INS_angle[2]);
        INS_degree[0] = INS_angle[0] * (180.0f / 3.14159265358979323846f);
        INS_degree[1] = INS_angle[1] * (180.0f / 3.14159265358979323846f);
        INS_degree[2] = INS_angle[2] * (180.0f / 3.14159265358979323846f);

        yaw_angle_from_bmi088 = INS_degree[0] ;
        pitch_angle_from_bmi088 = INS_degree[1] ;
        roll_angle_from_bmi088 = INS_degree[2] ;


        osDelay(1);
    }
}