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
	float pitch_target;							//target Ϊ�м�ֵ
	float yaw_set;
	float pitch_set;								//��������趨ֵ
	float pitch_low;
	float pitch_top;
	float yaw_right;
	float yaw_left;									//�������Ҽ���ֵ
//	float yaw_real;
//	float pitch_real;								//���������ʵֵ
//	float yaw_bias;
//	float pitch_bias;								//������Ļ����Ƕ�
	int back_flag;									//���б�־λ
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
void Set_CloudMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);//��̨����������ͺ���//CAN1���ͺ���


extern int fly_flag;
extern Angle t_angle;
#endif





// low -180
//middle -150
//-100
