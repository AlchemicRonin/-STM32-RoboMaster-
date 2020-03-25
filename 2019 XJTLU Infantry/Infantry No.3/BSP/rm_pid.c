#include "main.h"

PID pid[ALL_PID];		//定义所有PID结构体数组
PID_INCR pid_incr[ALL_PID_INCR];
float out[ALL_PID] = {0};//PID输出值数组
float out_incr[ALL_PID_INCR] = {0};



//int64_t aabbccdd = 0;
//float sset1 , rreal1 , oout1 ;
//void Look(void)
//{
////	sset = pid[PITCH_ANGLE].set;
////	rreal = pid[PITCH_ANGLE].real;
////	oout = pid[PITCH_ANGLE].out;
//	
//	sset1 = pid_incr[TEMP].set;
//	rreal1 = pid_incr[TEMP].fdb;
//	oout1 = pid_incr[TEMP].out;
//	aabbccdd ++;
//}

void pid_incr_init(PID_INCR *pid, const float PID[3], float max_out, float max_iout)
{
	pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float pid_incr_calc(PID_INCR *pid, float ref, float set)
{
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;

	pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
	pid->Iout = pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	pid->out += pid->Pout + pid->Iout + pid->Dout;
	pid->out = pid->out > pid->max_out ? pid->max_out : pid->out;
	pid->out = pid->out < -pid->max_out ? -pid->max_out : pid->out;
	return pid->out;
}

void ALL_Pid_Incr_Configuration()
{
	float PID_INCR_CHASSIS[3] = {10.0f, 1.0f, 0.0f};
	
	pid_incr_init(&pid_incr[FR], PID_INCR_CHASSIS, 5000.0f, 50.0f);
	pid_incr_init(&pid_incr[FL], PID_INCR_CHASSIS, 5000.0f, 50.0f);
	pid_incr_init(&pid_incr[BL], PID_INCR_CHASSIS, 5000.0f, 50.0f);
	pid_incr_init(&pid_incr[BR], PID_INCR_CHASSIS, 5000.0f, 50.0f);
	
//	float PID_INCR_CHASSIS[3] = {60.0f, 1.0f, 0.0f};
//	
//	pid_incr_init(&pid_incr[FR], PID_INCR_CHASSIS, 16000.0f, 2000.0f);
//	pid_incr_init(&pid_incr[FL], PID_INCR_CHASSIS, 16000.0f, 2000.0f);
//	pid_incr_init(&pid_incr[BL], PID_INCR_CHASSIS, 16000.0f, 2000.0f);
//	pid_incr_init(&pid_incr[BR], PID_INCR_CHASSIS, 16000.0f, 2000.0f);
	
	
}


//pid值初始化
static void pid_init(PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax)
{	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->poutmax = poutmax;
	pid->ioutmax = ioutmax;
	pid->doutmax = doutmax;
	pid->outmax = outmax;
	
	pid->f_pid_reset = pid_reset;
	pid->f_pid_reset(pid);
	
	pid->err = 0;
	pid->err_last = 0;
	pid->err_llast = 0;
	pid->integral = 0;
	
}
//pid输出值重置
static void pid_reset(PID *pid)
{
	
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out  = 0;
	
}



//PID初始化
//所有电机pid值初始化（形参需要后续更改，第一次测试形参全为0）
//(&pid[i], kp, ki, kd, poutmax, ioutmax, doutmax, outmax)
void All_Pid_Configuration(PID pid[])
{	
	pid[PITCH_ANGLE].f_pid_init = pid_init;
	pid[PITCH_ANGLE].f_pid_init(&pid[PITCH_ANGLE], 25.0f, 0.0f, 0.0f, 250.0f, 50.0f, 100.0f, 250.0f);
	
	pid[PITCH_SPEED].f_pid_init = pid_init;
	pid[PITCH_SPEED].f_pid_init(&pid[PITCH_SPEED], 290.0f, 1.0f, 80.0f, 30000.0f, 1000.0f, 10000.0f, 30000.0f);

	pid[YAW_ANGLE].f_pid_init = pid_init;
	pid[YAW_ANGLE].f_pid_init(&pid[YAW_ANGLE], 20.0f, 0.0f, 0.0f, 1000.0f, 50.0f, 500.0f, 1000.0f);

	pid[YAW_SPEED].f_pid_init = pid_init;
//	pid[YAW_SPEED].f_pid_init(&pid[YAW_SPEED], 180.0f, 4.0f, 80.0f, 30000.0f, 1000.0f, 10000.0f, 30000.0f);
	pid[YAW_SPEED].f_pid_init(&pid[YAW_SPEED], 180.0f, 0.0f, 0.0f, 30000.0f, 1000.0f, 10000.0f, 30000.0f);
	
	pid[CHASSIS_FOLLOW].f_pid_init = pid_init;
	pid[CHASSIS_FOLLOW].f_pid_init(&pid[CHASSIS_FOLLOW], 6.0f, 2.5f, 300.0f, 500.0f, 0.0f, 500.0f, 500.0f);
	
	pid[FEED_MOTOR_SINGLE].f_pid_init = pid_init;
	pid[FEED_MOTOR_SINGLE].f_pid_init(&pid[FEED_MOTOR_SINGLE], 50.0f, 0.0f, 0.0f, 5000.0f, 0.0f, 0.0f, 5000.0f);
	
	pid[FEED_MOTOR_DOUBLE_ANGLE].f_pid_init = pid_init;
	pid[FEED_MOTOR_DOUBLE_ANGLE].f_pid_init(&pid[FEED_MOTOR_DOUBLE_ANGLE], 12.5f, 0.0f, 100.0f, 5000.0f, 0.0f, 5000.0f, 5000.0f);
	
	pid[VISUAL_YAW_OFFSET].f_pid_init = pid_init;
	pid[VISUAL_YAW_OFFSET].f_pid_init(&pid[VISUAL_YAW_OFFSET], 0.005f, 0.0f, 0.0001f, 1.5f, 0.0f, 0.2f, 1.7f);
	
	pid[VISUAL_PITCH_OFFSET].f_pid_init = pid_init;
	pid[VISUAL_PITCH_OFFSET].f_pid_init(&pid[VISUAL_PITCH_OFFSET], 0.005f, 0.0f, 0.0001f, 1.5f, 0.0f, 0.2f, 1.7f);
	
	pid[FIRE_MOTOR_SPEED_L].f_pid_init = pid_init;
	pid[FIRE_MOTOR_SPEED_L].f_pid_init(&pid[FIRE_MOTOR_SPEED_L], 5.0f, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f, 50.0f);
	 
	pid[FIRE_MOTOR_SPEED_R].f_pid_init = pid_init;
	pid[FIRE_MOTOR_SPEED_R].f_pid_init(&pid[FIRE_MOTOR_SPEED_R], 5.0f, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f, 50.0f);
	
	pid[CHASSIS_CURRENT_MAX].f_pid_init = pid_init;
	pid[CHASSIS_CURRENT_MAX].f_pid_init(&pid[CHASSIS_CURRENT_MAX], 200.0f, 0.0f, 0.0f, 9000.0f, 0.0f, 0.0f, 9000.0f);

}

//pid计算出输出值 
float Calculate_Current_Value(PID *pid, float set, float real)
{
	//首先置零上一次的输出值
	pid->f_pid_reset = pid_reset;
	pid->f_pid_reset(pid);
	
	pid->set = set ;
	pid->real = real;
	
	pid->err_last = pid->err;
	pid->err = pid->set - pid->real;
	pid->integral += pid->err;
	
	pid->pout = pid->kp * pid->err;
	pid->pout = pid->pout < pid->poutmax ? pid->pout : pid->poutmax;
	pid->pout = pid->pout > -pid->poutmax ? pid->pout : -pid->poutmax;
	
	pid->iout = pid->ki * pid->integral;
	pid->iout = pid->iout < pid->ioutmax  ? pid->iout : pid->ioutmax;
	pid->iout = pid->iout > -pid->ioutmax ? pid->iout : -pid->ioutmax;
	
	pid->dout = pid->kd * (pid->err - pid->err_last);
	pid->dout = pid->dout < pid->doutmax ? pid->dout : pid->doutmax;
	pid->dout = pid->dout > -pid->doutmax ? pid->dout : -pid->doutmax;
	
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = pid->out < pid->outmax ? pid->out : pid->outmax;
	pid->out = pid->out > -pid->outmax ? pid->out : -pid->outmax;
	
	return pid->out;
}
float Calculate_Current_Value_For_Err(PID *pid, float err)
{
	pid->f_pid_reset = pid_reset;
	pid->f_pid_reset(pid);
	
	
	pid->err_last = pid->err;
	pid->err = err;
	pid->integral += pid->err;
	
	pid->pout = pid->kp * pid->err;
	pid->pout = pid->pout < pid->poutmax ? pid->pout : pid->poutmax;
	pid->pout = pid->pout > -pid->poutmax ? pid->pout : -pid->poutmax;
	
	pid->iout = pid->ki * pid->integral;
	pid->iout = pid->iout < pid->ioutmax  ? pid->iout : pid->ioutmax;
	pid->iout = pid->iout > -pid->ioutmax ? pid->iout : -pid->ioutmax;
	
	pid->dout = pid->kd * (pid->err - pid->err_last);
	pid->dout = pid->dout < pid->doutmax ? pid->dout : pid->doutmax;
	pid->dout = pid->dout > -pid->doutmax ? pid->dout : -pid->doutmax;
	
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = pid->out < pid->outmax ? pid->out : pid->outmax;
	pid->out = pid->out > -pid->outmax ? pid->out : -pid->outmax;
	
	return pid->out;
}

