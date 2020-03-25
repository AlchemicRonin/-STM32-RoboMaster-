#include "sys.h"

__asm void WFI_SET(void)
{
    WFI;
}

__asm void INTX_DISABLE(void)
{
    CPSID I
        BX LR
}

__asm void INTX_ENABLE(void)
{
    CPSIE I
        BX LR
}

__asm void MSR_MSP(uint32_t addr)
{
    MSR MSP, r0 //set Main Stack value
                 BX r14
}
