#ifndef SYS_H
#define SYS_H
#include "main.h"

void WFI_SET(void);
void INTX_DISABLE(void);
void INTX_ENABLE(void);
void MSR_MSP(uint32_t addr);

#endif
