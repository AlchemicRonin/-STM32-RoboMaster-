#ifndef __CLOUDMOTOR_H___
#define __CLOUDMOTOR_H___

#include "delay.h"

#define PITCH_MECHANICAL_TOP 	-5.0f
#define PITCH_MECHANICAL_CENTRE 40.0f
#define PITCH_MECHANICAL_LOW	75.0f
#define YAW_MECHANICAL_CENTRE	-32.0f
typedef struct Angle
{
	float yaw_target;
	float pitch_target;							//target 为中间值
	float yaw_set;
	float pitch_set;								//两个轴的设定值
	float pitch_low;
	float pitch_top;
	float yaw_right;
	float yaw_left;									//上下左右极限值
//	float yaw_real;
//	float pitch_real;								//两个轴的真实值
//	float yaw_bias;
//	float pitch_bias;								//两个轴的基础角度
	int back_flag;									//回中标志位
	float pitch_remote_last;
	float pitch_remote_now;
	float pitch_differ;
	float pitch_set_last;
//	void (*f_angle_init)(struct Angle *angle);
//	void (*f_angle_update)(struct Angle *angle);
	float pitch_first;
	float yaw_first;
}Angle;

void angle_init(struct Angle *angle);
void angle_out_update(struct Angle *angle);
void angle_set_update(struct Angle *angle);
void Set_CloudMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);//云台电机电流发送函数//CAN1发送函数


extern int fly_flag;
extern Angle t_angle;
#endif





// low -180
//middle -150
//-100
