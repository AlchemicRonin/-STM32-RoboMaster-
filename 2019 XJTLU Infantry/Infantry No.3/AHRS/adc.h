#ifndef ADC_H
#define ADC_H
#include "stm32f4xx.h"
#define TEMP_MAX_SET 40.0f
typedef float fp32;

typedef struct 
{
	fp32 temp_set;
	fp32 temp_real;
	fp32 temp_max_set;
}Temperature;

#define cali_get_mcu_temperature() get_temprate() //获取stm32上的温度 作为IMU校准的环境温度

void temperature_ADC_init(void);
void temperature_ADC_Reset(void);
fp32 get_temprate(void);
uint16_t get_ADC(uint8_t ch);
u8 get_set_temp(void);
extern Temperature temperature;



#endif
