#ifndef __APP_H__
#define __APP_H__
#include <stm32f4xx.h>
#include "can1.h"

#define ESC_MAX 1500.0f

typedef struct PID_PARAMETER
{
	float Kp;
	float Ki;
	float Kd;
	float error_now;
	float error_last;
	float error_inter;
	float pid_out;
}PID;
extern PID Gimbal_Position_Pitch,Gimbal_Position_Yaw,Gimbal_Speed_Pitch,Gimbal_Speed_Yaw;

void DriveMotor(int16_t current_205,int16_t current_206);
void PID_Control( float current_position, float expected_position,PID* motor_type); 

#endif
