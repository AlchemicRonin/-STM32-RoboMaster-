#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f4xx.h"


#define M_PI  (float)3.1415926535


void IMU_Configure(void);
void IMU_INT_Configure(void);
void IMU_EXIT_STOP(void);
void IMU_EXIT_START(void);
void IMU_control_decide(void);

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef struct packed_angle
{
	float last_yaw;
	float new_yaw;
	float diff_yaw;
	float yaw_counts;
	float yaw;
	float pitch;
	float roll;
	float gx;
	float gy;
	float gz;
	float temperature;
	float state;

}packed_angle;

typedef struct
{
	float default_temp;
	float all_temp;
	uint8_t cnt;
}Imu_Temp;
extern GyroCaliStruct_t GyroSavedCaliData; 
extern volatile packed_angle real_angle;
extern float Gyro_z_err;
extern float Gyro_x_err;
extern float Gyro_y_err;

#endif
