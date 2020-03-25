#include "rng.h"
#include "stm32f4xx.h"
#include "delay.h"

uint8_t RNG_init(void)
{
    uint16_t retry = 0;

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

    RCC_AHB2PeriphResetCmd(RCC_AHB2Periph_RNG, ENABLE);
    RCC_AHB2PeriphResetCmd(RCC_AHB2Periph_RNG, DISABLE);

    RNG_Cmd(ENABLE);

    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET && retry < 10000)
    {
        retry++;
        delay_us(100);
    }
    if (retry >= 10000)
        return 1;
    return 0;
}

unsigned int RNG_get_random_num(void)
{
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET)
        ;
    return RNG_GetRandomNumber();
}

int RNG_get_random_range(int min, int max)
{
    return RNG_get_random_num() % (max - min + 1) + min;
}
