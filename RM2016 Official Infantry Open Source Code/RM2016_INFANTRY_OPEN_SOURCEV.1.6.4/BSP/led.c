#include "main.h"

/*----GREEN LED----PA6-----'0' is on,'1' is off */
/*----RED LED----PA7-----'0' is on,'1' is off */

void Led_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 ;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
    
    GREEN_LED_OFF();
    RED_LED_OFF();
}
