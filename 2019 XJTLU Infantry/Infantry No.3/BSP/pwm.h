#ifndef __PWM_H__
#define __PWM_H__

#include "main.h"


void TIM1_FireMotor_Configure(void);
void TIM3_Init(uint16_t arr, uint16_t psc);
void heat_output(uint16_t pwm);
void TIM5_Pwm_Configure(void);
void head_close(void);
void head_open(void);
void head_control(void);
typedef struct 
{
	u8 last_w;
	u8 now_w;
	u8 last_s;
	u8 now_s;
//	u8 head_cnts;
}HEAD;
#endif

