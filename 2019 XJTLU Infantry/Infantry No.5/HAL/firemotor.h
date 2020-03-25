#ifndef _FIREMOTOR_H_
#define _FIREMOTOR_H_
#include "stm32f4xx.h"
void fireMotor_stop(void);
void fireMotor_fire(void);
void fireMotor(void);
void ramp_calc(void);
void ramp_init(void);

typedef struct 
{
	float input;
	float out;
	float target;
	float min;
	float max;
	float frame_period;
	uint8_t add_each_second;
	float add_real;
}Ramp_Start;

#endif






