#ifndef _CHASSISMOTOR_H_
#define _CHASSISMOTOR_H_

#include "delay.h"
#define keypad_K 2u
#define remote_K 1.6f
#define limit 1012u

#define  CHASSIS_VX_RC_SEN 1.666f			//x轴
#define CHASSIS_VY_RC_SEN 1.610f			//y轴

#define VX_MAX 1250.0f              //速度Max1250
#define VY_MAX 1200.0f              //速度max1200


#define CHASSIS_CONTROL_TIME_MS 1u

#ifndef PI
#define PI 3.14159265358979f
#endif

//底盘任务控制间隔 0.001s
#define CHASSIS_CONTROL_TIME 0.001f
#define CHASSIS_ACCEL_X_NUM 0.0133333333f
#define CHASSIS_ACCEL_Y_NUM 0.0139999999f
#define CHASSIS_WZ_SET_SCALE 0.0f
#define MOTOR_DISTANCE_TO_CENTER 0.35f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR


void chassis_set_update(void);
void chassis_out_update(void);
void mecanum_Resolving(float *a ,float *b,float *c,float *d,int z);    //麦伦结算
void Chassis_fllow(float *x, float *y);
void remote_fifo(void);

// 用于底盘一阶滤波启动
typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

typedef struct Chassis_set
{
	
	float cm1_set;
	float cm2_set;
	float cm3_set;
	float cm4_set;
	
	float cm1_real;
	float cm2_real;
	float cm3_real;
	float cm4_real;
	float follow_set;
	float follow_real;
	float YAW_CENTRE;
	float YAW_CENTRE_Init;
	float YAW_DIR;
	
	float RAD_RC_last;
	float RAD_RC_now;
	float RAD_RC_diff;
	first_order_filter_type_t chassis_cmd_slow_set_vx;
	first_order_filter_type_t chassis_cmd_slow_set_vy;
}Chassis_set;
extern Chassis_set chassis_set;
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void chassis_init(void);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);


//volatile Chassis_set chassis_set;
#endif




//#ifndef _CHASSISMOTOR_H_
//#define _CHASSISMOTOR_H_

//#include "delay.h"
//#define keypad_K 2
//#define remote_K 1.6f
//#define limit 1012

////#define X_SPEED_UP_FIRST 0
////#define Y_SPEED_UP_FIRST 1
////#define X_SPEED_UP_SECOND 2
////#define Y_SPEED_UP_SECOND 3
////#define X_SLOW_DOWN 4
////#define Y_SLOW_DOWN 5
////#define CHASSIS_FOLLOW_SLOW 6
//#define X_FILTER 0
//#define Y_FILTER 1
//#define FOLLOW_FILTER 2


//#define MAX_CURRENT 12000.0f

////#define  CHASSIS_VX_RC_SEN 1.1363636f			//x轴
////#define CHASSIS_VY_RC_SEN 1.1363636f			//y轴
//#define  CHASSIS_VX_RC_SEN 1.1363636f			//x轴
//#define CHASSIS_VY_RC_SEN 1.59595959f			//y轴
//#define VX_MAX 850.0f
//#define VY_MAX 850.0f

//#define VX_MAX_TWO_STAGE 850.0f
//#define VY_MAX_TWO_STAGE 850.0f 


//#define CHASSIS_CONTROL_TIME_MS 1u

//#ifndef PI
//#define PI 3.14159265358979f
//#endif


////底盘任务控制间隔 0.001s
//#define CHASSIS_CONTROL_TIME 0.001f
//#define CHASSIS_ACCEL_X_NUM 		 0.69999999f
//#define CHASSIS_ACCEL_Y_NUM 		 0.69999999f
//#define CHASSIS_ACCEL_FOLLOW 		 0.16666666f
//#define CHASSIS_ACCEL_FOLLOW_SECOND  0.1666666f
//#define CHASSIS_ACCEL_X_SECOND_NUM   3.33333333f
//#define CHASSIS_ACCEL_Y_SECOND_NUM	 3.39999999f
//#define CHASSIS_ACCEL_X_STOP_NUM 	 0.33333333f
//#define CHASSIS_ACCEL_Y_STOP_NUM	 0.39999999f

//#define CHASSIS_WZ_SET_SCALE 0.0f
//#define MOTOR_DISTANCE_TO_CENTER 0.35f

////m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

////底盘电机最大速度
////#define MAX_WHEEL_SPEED 755.0f
////#define MAX_WHEEL_SPEED_TWO_STAGE 1050.0f;



//// 用于底盘一阶滤波启动
//typedef __packed struct
//{
//    float input;        //输入数据
//    float out;          //滤波输出的数据
//    float num[1];       //滤波参数
//    float frame_period; //滤波的时间间隔 单位 s
//} first_order_filter_type_t;

//typedef struct Chassis_set
//{
//	
//	float cm_set[4];
//	
//	float cm1_real;
//	float cm2_real;
//	float cm3_real;
//	float cm4_real;
//	float follow_set;
//	float follow_real;
//	float YAW_CENTRE;
//	float YAW_CENTRE_Init;
//	float YAW_DIR;
//	float total_current_real;
//	float total_current_set;
//	float total_current;
//	float RAD_RC_last;
//	float RAD_RC_now;
//	float RAD_RC_diff;
//	float current_ratio;
//	float chassis_set_total;
//	first_order_filter_type_t chassis_cmd_slow_set_vx;
//	first_order_filter_type_t chassis_cmd_slow_set_vy;
//	first_order_filter_type_t chassis_cmd_slow_set_follow;
//	first_order_filter_type_t chassis_cmd_slow_set_follow_second;
//	first_order_filter_type_t chassis_cmd_slow_set_second_vx;
//	first_order_filter_type_t chassis_cmd_slow_set_second_vy;
//	first_order_filter_type_t chassis_cmd_stop_set_vx;
//	first_order_filter_type_t chassis_cmd_stop_set_vy;
//	
//	float chassis_max_current;
//	float x_filted;
//	float y_filted;
//	float follow_filted;
//}Chassis_set;


//extern Chassis_set chassis_set;
////void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
//void chassis_init(void);
//void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input, u8 filter);
//void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
//void chassis_set_update(void);
//void chassis_out_update(void);
//void mecanum_Resolving(Chassis_set *set, int z);    //麦伦结算
//void Chassis_fllow(float *x, float *y);
//void remote_fifo(void);

////volatile Chassis_set chassis_set;
//#endif
