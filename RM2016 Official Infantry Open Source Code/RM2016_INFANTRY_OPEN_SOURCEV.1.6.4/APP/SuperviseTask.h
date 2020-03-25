#ifndef _DEBUG_H_
#define _DEBUG_H_
#include "stm32f4xx.h"

typedef enum ERROR_CODE
{
	NO_ERROR = 0,
	CALI_ERROR = 1,
	GYRO_MODULE_ERROR = 2,
	MOTOR_201_ERROR = 3,
	MOTOR_202_ERROR = 4,
	MOTOR_203_ERROR = 5,
	MOTOR_204_ERROR = 6,
	MOTOR_205_ERROR = 7,
	MOTOR_206_ERROR = 8,
	REMOTE_LOST_ERROR = 9,
	DEAD_LOCK_ERROR = 10,
	SHOOT_Encoder_ERROR = 11,
}ERROR_CODE;

typedef struct Can_Info
{
	uint32_t last_count;
	uint32_t this_count;
	uint32_t temp_count;
	uint32_t ms_count;
	uint8_t error_flag;
}Can_Info;       //

typedef struct Cali_Error
{
	uint8_t config_error;
	uint8_t error_gyro;
	uint8_t error_hmc5883;
	uint8_t error_mag;
	uint8_t error_encoder_205;
	uint8_t error_encoder_206;
}Cali_Error;

typedef struct Remote_Info
{
	uint32_t last_count;
	uint32_t this_count;
	uint32_t remote_count;
	uint32_t ms_count;
	uint8_t lost_flag;
}Remote_Info;


typedef struct Debug_Info
{
	ERROR_CODE error_code;
	Can_Info motor_201_info;      //1Hz
	Can_Info motor_202_info;      //2Hz
	Can_Info motor_203_info;      //3Hz
	Can_Info motor_204_info;      //4Hz
	Can_Info motor_205_info;      //3Hz
	Can_Info motor_206_info;      //4Hz	
	Can_Info can2_tx_info;				//can2 tx error
	Can_Info can1_info;				//	can1 error
	Cali_Error cali_info;
	Remote_Info remote_info;
	Remote_Info shoot_encoder_info;
}Debug_Info;



//LostCounterFeed

#define LOST_COUNTER_NUM                             11u

//IMU不正常
#define LOST_COUNTER_INDEX_RC                        0u   //green:green:green 1:1:1
#define LOST_COUNTER_INDEX_IMU                       1u    //red always on
#define LOST_COUNTER_INDEX_ZGYRO                     2u    //
#define LOST_COUNTER_INDEX_MOTOR1                    3u    //green:red:green 1 1 1 
#define LOST_COUNTER_INDEX_MOTOR2                    4u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR3                    5u    //green:red:green 1 3 1 
#define LOST_COUNTER_INDEX_MOTOR4                    6u    //green:red:green 1 4 1 
#define LOST_COUNTER_INDEX_MOTOR5                    7u    //green:red:green 1 5 1 
#define LOST_COUNTER_INDEX_MOTOR6                    8u    //green:red:green 1 6 1 
#define LOST_COUNTER_INDEX_DEADLOCK                  9u    //red:red:red 1:1:1
#define LOST_COUNTER_INDEX_NOCALI            		 10u    //red:red:red 1:1:1

#define LOST_ERROR_RC									(1<<0)		//rc lost 
#define LOST_ERROR_IMU									(1<<1)		//mpu6050 error
#define LOST_ERROR_ZGYRO								(1<<2)		//can1 zyro error
#define LOST_ERROR_MOTOR1								(1<<3)		//motor1 error
#define LOST_ERROR_MOTOR2								(1<<4)		//
#define LOST_ERROR_MOTOR3								(1<<5)		//
#define LOST_ERROR_MOTOR4								(1<<6)		//
#define LOST_ERROR_MOTOR5								(1<<7)		//
#define LOST_ERROR_MOTOR6								(1<<8)		//
#define LOST_ERROR_DEADLOCK								(1<<9)		//deadlock error
#define LOST_ERROR_NOCALI  						        (1<<10)		//nocali error

#define LOST_ERROR_ALL (LOST_ERROR_RC|LOST_ERROR_IMU|LOST_ERROR_ZGYRO|LOST_ERROR_MOTOR1|LOST_ERROR_MOTOR2|LOST_ERROR_MOTOR3|LOST_ERROR_MOTOR4|LOST_ERROR_MOTOR5|LOST_ERROR_MOTOR5|LOST_ERROR_DEADLOCK|LOST_ERROR_NOCALI)
#define RED_LED 0
#define GREEN_LED 1

extern Debug_Info debug_info;   //错误消息

void SuperviseTask(void);
void LED_TOGGLE(uint8_t led, uint16_t CYCLE_S, uint16_t TOGGLE_S, uint16_t shine_times,uint16_t TIME_SPAN);
uint32_t *GetLostCounter(uint8_t index);
uint32_t Get_Lost_Error(uint32_t err_code);
uint8_t Is_Lost_Error_Set(uint32_t err_code);
uint8_t Is_Serious_Error(void);
#endif
