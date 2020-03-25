#ifndef		_RM_PID_H_
#define		_RM_PID_H_

#include "delay.h"

#define FR 0
#define FL 1
#define BL 2
#define BR 3
#define PITCH_ANGLE 4
#define PITCH_SPEED 5
#define YAW_ANGLE  6
#define YAW_SPEED 7
#define CHASSIS_FOLLOW 8


#define FEED_MOTOR_SINGLE 9
#define FEED_MOTOR_DOUBLE_ANGLE 10
#define FEED_MOTOR_DOUBLE_SPEED 11

#define VISUAL_YAW_OFFSET 12
#define VISUAL_PITCH_OFFSET 13

#define FIRE_MOTOR_SPEED_L 14
#define FIRE_MOTOR_SPEED_R 15
#define CHASSIS_CURRENT_MAX 16

#define ALL_PID 17



#define TEMP 5
#define ALL_PID_INCR 6

typedef struct PID
{
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	
	float poutmax;
	float ioutmax;
	float doutmax;
	float outmax;
	
	float set;
	float real;
	float out;
	
	float err;							//定义偏差值
	float err_last;					//上一次偏差值
	float err_llast;				//上上次偏差值
	float integral;					//累计偏差值
	void(*f_pid_init)(struct PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);			//用来初始化pid
	void(*f_pid_reset)(struct PID *pid);
}PID;

typedef struct
{
	 //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
	
    float set;
    float fdb;	

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次	
	
}PID_INCR;


void pid_incr_init(PID_INCR *pid, const float PID[3], float max_out, float max_iout);
float pid_incr_calc(PID_INCR *pid, float ref, float set);
void ALL_Pid_Incr_Configuration(void);
extern PID_INCR pid_incr[ALL_PID_INCR];
extern float out_incr[ALL_PID_INCR];


extern PID pid[ALL_PID];
extern float out[ALL_PID];
static void pid_init(PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);
static void pid_reset(PID *pid);

void All_Pid_Configuration(PID pid[]);
float Calculate_Current_Value(PID *pid, float set, float real);
float Calculate_Current_Value_For_Err(PID *pid, float err);
void Look(void);


#endif
