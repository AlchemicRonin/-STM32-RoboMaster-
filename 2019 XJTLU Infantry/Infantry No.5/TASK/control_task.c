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
	if(run_flag == 1)//����
	{

		if(time_1ms <= 3)//�������ȶ���ʼִ��
		{
			angle_init(&t_angle);//λ���趨ֵ��ʼ��
			time_1ms ++;
		}
		else
		{
			//�����㷨
			angle_out_update(&t_angle);//��̨�������	
			if(t_angle.back_flag == 1)
			{
				chassis_out_update();//���̵������
				fireMotor();//Ħ���ֿ���
				ramp_calc();//б������	
//				if(ramp_start.back_flag == 1)//Ħ���ִ�
//				{
					Shoot();//�����������
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
	else if(run_flag == 2)//У׼
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
