#ifndef __BUZZER_H__
#define __BUZZER_H__

void Buzzer_Configuration(void);

#define BUZZER_ON()      GPIO_SetBits(GPIOA,GPIO_Pin_1)
#define BUZZER_OFF()     GPIO_ResetBits(GPIOA,GPIO_Pin_1)
#define BUZZER_TOGGLE()  GPIO_ToggleBits(GPIOA,GPIO_Pin_1)

#endif
