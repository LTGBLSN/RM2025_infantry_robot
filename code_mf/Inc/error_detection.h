//
// Created by 21481 on 2025/3/18.
//

#ifndef BUBING_RM2025_ERROR_DETECTION_H
#define BUBING_RM2025_ERROR_DETECTION_H

//RC
#define RC_NO_DATA_TIMEOUT 300
#define RC_ONLINE 1
#define RC_OFFLINE 0


//IMU
#define GM6020_TEMP_MAX 50
#define GM6020_SAFE 1
#define GM6020_DIE 0


void error_detection();

void rc_connection_status();

void imu_connection_status();

void yaw_6020_status();
void pitch_6020_status();

#endif //BUBING_RM2025_ERROR_DETECTION_H
