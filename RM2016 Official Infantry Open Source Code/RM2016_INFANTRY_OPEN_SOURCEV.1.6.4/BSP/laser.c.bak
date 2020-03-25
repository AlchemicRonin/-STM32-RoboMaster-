#include "main.h"

void Laser_Configuration()
{
	GPIO_InitTypeDef gpioInitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_8;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

