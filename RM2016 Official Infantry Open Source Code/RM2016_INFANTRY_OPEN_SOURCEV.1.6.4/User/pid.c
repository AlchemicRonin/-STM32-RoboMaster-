#include "main.h"
#define GAP 0.0

#define VELOCITY_MAX   2000

#define TORQUE_MAX     5000



#define Inter_Max   2000

PID Gimbal_Position_Pitch,Gimbal_Position_Yaw,Gimbal_Speed_Pitch,Gimbal_Speed_Yaw;

void PID_Control(float current_position, float expected_position,PID* motor_type)
{
//  float error_position;
	motor_type->error_last=motor_type->error_now;
	motor_type->error_now = expected_position - current_position;
	motor_type->error_inter += motor_type->error_now;
	// limit intergration of pid
	if(motor_type->error_inter>Inter_Max)
		  motor_type->error_inter = Inter_Max;
	if(motor_type->error_inter<-Inter_Max)
		  motor_type->error_inter = -Inter_Max;
 motor_type->pid_out = motor_type->Kp * motor_type->error_now + motor_type->Ki * motor_type->error_inter +	motor_type->Kd * (motor_type->error_now-motor_type->error_last);

}
 


