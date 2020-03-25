#ifndef __CAN2_H_
#define __CAN2_H_
#include "delay.h"


uint8_t CAN2_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
void CAN2_STOP(void);
void CAN2_START(void);
#endif
