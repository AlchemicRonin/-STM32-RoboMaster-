#include "main.h"

extern u8 offset_OK_flag;
extern u8 offset_OK_flag1;
extern u8 run_flag;
u8 time_1ms = 0;
double power_all;
void control_task() 
{
	power_all += (double)(judge_rece_mesg.power_heat_data.chassis_power) * 0.001;
//	wait_to_charge();
	remote_off_line();
//	power_info_tx();
	if(run_flag == 1)//启动
	{

		if(time_1ms <= 3)//待数据稳定后开始执行
		{
			angle_init(&t_angle);//位置设定值初始化
			time_1ms ++;
		}
		else
		{
			//控制算法
			angle_out_update(&t_angle);//云台电机控制	
			if(t_angle.back_flag == 1)
			{
				chassis_out_update();//底盘电机控制
				fireMotor();//摩擦轮控制
				ramp_calc();//斜坡启动	
//				if(ramp_start.back_flag == 1)//摩擦轮打开
//				{
					Shoot();//拨弹电机控制
//				}
//				else
//				{
//					feed_set.feed_speed_set = 0;
//					Set_FeedMotor_Current((int16_t)0);
//				}
			}	
		}
		head_control();
	}
	else if(run_flag == 2)//校准
	{
		if(offset_OK_flag == 2)
		{
			cali_gyro_hook();
		}
		else if(offset_OK_flag == 1)
		{
			run_flag = 1;
		}
	}
}
