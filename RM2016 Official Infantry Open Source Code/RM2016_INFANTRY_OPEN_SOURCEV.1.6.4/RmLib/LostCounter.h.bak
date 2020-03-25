#ifndef __LOST_COUNTER_H__
#define __LOST_COUNTER_H__


#include "stm32f4xx.h"


typedef uint32_t LostCounter_t;

void LostCounterFeed(LostCounter_t *lc);
void LostCounterCount(LostCounter_t *lc, uint32_t os_ticks);
uint8_t LostCounterOverflowCheck(LostCounter_t lc, uint32_t threshold);

#endif 
