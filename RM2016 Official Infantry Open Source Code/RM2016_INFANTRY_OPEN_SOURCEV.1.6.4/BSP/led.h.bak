#ifndef __LED_H__
#define __LED_H__

void Led_Configuration(void);

#define GREEN_LED_ON()      GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_OFF()     GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define RED_LED_ON()            GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define RED_LED_OFF()           GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOC, GPIO_Pin_2)


//#define GREEN_LED_ON()          ((void)0)
//#define GREEN_LED_OFF()         ((void)0)
//#define GREEN_LED_TOGGLE()      ((void)0)

//#define RED_LED_ON()            ((void)0)
//#define RED_LED_OFF()           ((void)0)
//#define RED_LED_TOGGLE()        ((void)0)
	
#define BOTH_LED_TOGGLE()\
GPIO_ToggleBits(GPIOC, GPIO_Pin_1);\
GPIO_ToggleBits(GPIOC, GPIO_Pin_2)

#endif
