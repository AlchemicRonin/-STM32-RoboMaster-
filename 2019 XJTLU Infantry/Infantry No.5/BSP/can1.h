#ifndef __CAN1_H_
#define __CAN1_H_
#include "delay.h"

uint8_t CAN1_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
void CAN1_STOP(void);
void CNA1_START(void);

#endif
