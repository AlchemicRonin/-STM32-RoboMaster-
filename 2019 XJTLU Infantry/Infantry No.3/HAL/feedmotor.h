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
void Set_FeedMotor_Current(int16_t feed_motor_iq);//拨弹电机电流发送函数//CAN2发送函数

#endif































////////////////////////////////////////////////////////////////////////////////////////
//#ifndef _FEEDMOTOR_H_
//#define _FEEDMOTOR_H_
//#include "stm32f4xx.h"
//#define SingleBall 2160
//#define Feed_Speed_MAX	450
//#define Feed_Speed_Err  200
//#define Feed_Angle_Err  100
//#define Feed_All_Time	150		//反应时间
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
//	int16_t dance_flag;				//扭腰flag 
//	int16_t last_dance_flag;		//扭腰flag 
//	int16_t dance_time;				//按下150ms扭腰
//	uint64_t presstime_1ms;			//按下的毫秒数
//	int16_t press_flag;				//1	反转	2	速度环
//	int16_t now_mouse_l;			//
//	int64_t sum_angle;				//当前编码器角度
//	int64_t press_number;			//标志位	
//	int16_t pressState;				//按下的状态   0 ：未按下    1 ：防卡弹    2 ：连按
//	int64_t jamTime_1ms;			//卡球时间（ms）
//	int16_t now_Jam;				//此刻卡球状态 		  0 ： 不卡     1 ： 卡
//	int16_t last_Jam;				//上一时刻刻卡球状态   0 ： 不卡     1 ： 卡

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
//void Set_FeedMotor_Current(int16_t feed_motor_iq);//拨弹电机电流发送函数//CAN2发送函数
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
////void Set_FeedMotor_Current(int16_t feed_motor_iq);//拨弹电机电流发送函数//CAN2发送函数

////#endif




