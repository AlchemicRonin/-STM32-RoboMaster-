#ifndef __TIMER_H__
#define __TIMER_H__

#include "main.h"

void TIM6_Configure(void);
void TIM6_Start(void);
void TIM6_Stop(void);
void TIM4_FireMotor_Configure(void);
void TIM3_Int_Init(void);
//extern int16_t firemotor_flag;
#endif
