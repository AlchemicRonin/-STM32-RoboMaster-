#ifndef DELAY_H
#define DELAY_H
#include "main.h"
#include "stm32f4xx.h"

extern void delay_init(uint32_t TICK_RATE_HZ);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);
#endif

