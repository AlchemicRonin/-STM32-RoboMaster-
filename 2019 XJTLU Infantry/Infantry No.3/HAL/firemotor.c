#include "main.h"

uint64_t fireFlag = 0;
int16_t last_shift = 0;
int16_t now_shift = 0;
Ramp_Start ramp_start;

void ramp_init(void)
{
	ramp_start.frame_period = 0.001;
	ramp_start.add_each_second = 100;
	ramp_start.add_real = ramp_start.frame_period * ramp_start.add_each_second;
	ramp_start.out = 1450;
}

int state_t = 0;
void ramp_calc(void)
{
	if((int16_t)ramp_start.out != (int16_t)ramp_start.input)
	{
		if(ramp_start.out < ramp_start.input)
			ramp_start.out += ramp_start.add_real;
		else if(ramp_start.out > ramp_start.input)
			ramp_start.out -= ramp_start.add_real;
		state_t = 1;
	}
	else 
	{
		state_t = 0;
		ramp_start.out = ramp_start.input;
	}
}

int64_t cnt = 0;
void fireMotor_stop()
{
	ramp_start.input = 1450;
	TIM1->CCR1 = ramp_start.out;
	TIM1->CCR4 = ramp_start.out;
	cnt = 0;
}



void fireMotor_fire()
{
//	TIM4->CCR1 = leftFireMotor;
//	TIM4->CCR2 = rightFireMotor;
//	cnt++;
//	if(cnt <= 1000000)
//	{
//		TIM1->CCR1 = 1300;
//	}
//	
//	else if(cnt >=2000000 && cnt<= 4000000)
//	{
//		TIM1->CCR4 = 1300;
//	}
//	else if(cnt  > 4000000)
//	{
//		TIM1->CCR1 = 1300;
//		TIM1->CCR4 = 1300;
//		cnt = 4000002;
//	}
	ramp_start.input = 1300;				// 摩擦轮速度范围 1000（最快） - 1450(停) ~~~~1350 21m/s 
	TIM1->CCR1 = ramp_start.out;
	TIM1->CCR4 = ramp_start.out;
	
}

void fireMotor()
{
	now_shift = rc.Shift ;
	if(now_shift == 1 && last_shift == 0)
		fireFlag++;
	if((fireFlag % 2 == 1) || (rc.sl == 3) || (rc.sl == 2))
		fireMotor_fire();
	else 
		fireMotor_stop();
	last_shift = now_shift;
//	TIM4->CCR1 = fireFlag;
//	TIM4->CCR2 = fireFlag;
}
