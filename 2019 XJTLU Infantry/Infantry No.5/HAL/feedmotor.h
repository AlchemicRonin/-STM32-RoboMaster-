#ifndef _FEEDMOTOR_H_
#define _FEEDMOTOR_H_
#include "stm32f4xx.h"

typedef struct Feed_Set
{
	float feed_speed_set;
	float feed_speed_real;
	int16_t feed_speed_max;
}Feed_set;
;

extern Feed_set feed_set;

void Shoot(void);
void fireMotorPidReal_update(void);
void fireMotorSingleLoopPid_out_update(void);
void Set_FeedMotor_Current(int16_t feed_motor_iq);//��������������ͺ���//CAN2���ͺ���

#endif































////////////////////////////////////////////////////////////////////////////////////////
//#ifndef _FEEDMOTOR_H_
//#define _FEEDMOTOR_H_
//#include "stm32f4xx.h"
//#define SingleBall 2160
//#define Feed_Speed_MAX	450
//#define Feed_Speed_Err  200
//#define Feed_Angle_Err  100
//#define Feed_All_Time	150		//��Ӧʱ��
//typedef struct Feed_Set
//{
//	//float feedSingle_set;
//	float feed_angle_set;
//	float feed_speed_set;
//	
//	//float feedSingle_real;
//	float feed_angle_real;
//	float feed_speed_real;
//}Feed_set;

//typedef struct Feed_Motor_State
//{	
//	int16_t if_dance;
//	int16_t dance_flag;				//Ť��flag 
//	int16_t last_dance_flag;		//Ť��flag 
//	int16_t dance_time;				//����150msŤ��
//	uint64_t presstime_1ms;			//���µĺ�����
//	int16_t press_flag;				//1	��ת	2	�ٶȻ�
//	int16_t now_mouse_l;			//
//	int64_t sum_angle;				//��ǰ�������Ƕ�
//	int64_t press_number;			//��־λ	
//	int16_t pressState;				//���µ�״̬   0 ��δ����    1 ��������    2 ������
//	int64_t jamTime_1ms;			//����ʱ�䣨ms��
//	int16_t now_Jam;				//�˿̿���״̬ 		  0 �� ����     1 �� ��
//	int16_t last_Jam;				//��һʱ�̿̿���״̬   0 �� ����     1 �� ��

//}Feed_Motor_State ;

//extern Feed_set feed_set;
//extern Feed_Motor_State feed_state;
//void shoot(int t,int speed);
//int feedBullet(int t,int speed);
//void stopFeedBullet(void);
//int checkStop(void);
//int returNormal(int t,int speed);
//float fabsValue(float a);
//void shoot2(void);
//void fireMotorPidReal_update(void);
//void fireMotorDoubleLoopPid_out_update(void);
//void fireMotorSingleLoopPid_out_update(void);
//void Shoot(void);
//int16_t IfJam(void);
//int16_t IfJam2(int16_t state);
//void Set_FeedMotor_Current(int16_t feed_motor_iq);//��������������ͺ���//CAN2���ͺ���
//int16_t Judge_Dance(void);
//#endif









////OLD
////#ifndef _FEEDMOTOR_H_
////#define _FEEDMOTOR_H_
////#include "stm32f4xx.h"

////typedef struct Feed_Set
////{
////	float feed_speed_set;
////	float feed_speed_real;
////	int16_t feed_speed_max;
////}Feed_set;
////;

////extern Feed_set feed_set;

////void Shoot(void);
////void fireMotorPidReal_update(void);
////void fireMotorSingleLoopPid_out_update(void);
////void Set_FeedMotor_Current(int16_t feed_motor_iq);//��������������ͺ���//CAN2���ͺ���

////#endif




