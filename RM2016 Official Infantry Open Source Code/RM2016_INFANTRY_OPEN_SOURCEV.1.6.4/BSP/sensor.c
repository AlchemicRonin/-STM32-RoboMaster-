#include "main.h"

/*Sensor1---PE1----*/
/*Sensor2---PE2---*/
/*Sensor3---PE3----*/
/*Sensor4---PE4----*/

/*Sensor5---PE5---*/
/*Sensor6---PE6----*/
/*Sensor7---PC0----*/
/*Sensor8---PC1----*/

/*Sensor9---PC2----*/
/*Sensor10--PC3---*/
/*Sensor11--PA2---*/
/*Sensor12--PA3---*/

/*Sensor13--PA4---*/
/*Sensor14--PA5---*/
/*Sensor15--PC4----*/
/*Sensor16--PC5----*/

void Sensor_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOC |
						   RCC_AHB1Periph_GPIOE ,ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;

	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOA,&gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC,&gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE,&gpio);
}
