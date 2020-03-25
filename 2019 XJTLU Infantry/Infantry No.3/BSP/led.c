
#include "main.h"

void led_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    led_green_off();
    led_red_off();

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
		
    flow_led_off(GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8);
		delay_ms(1000);
		flow_led_off(0);
		flow_led_off(1);
		flow_led_off(2);
		flow_led_off(3);
		flow_led_off(4);
		flow_led_off(5);
		flow_led_off(6);
		flow_led_off(7);
		
}

void led_green_off(void)
{
    GPIO_SetBits(GPIOF, GPIO_Pin_14);
}
void led_green_on(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_14);
}
void led_green_toggle(void)
{
    GPIO_ToggleBits(GPIOF, GPIO_Pin_14);
}

void led_red_off(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_11);
}
void led_red_on(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}
extern void led_red_toggle(void)
{
    GPIO_ToggleBits(GPIOE, GPIO_Pin_11);
}

void flow_led_on(uint16_t num)
{
    GPIO_ResetBits(GPIOG, GPIO_Pin_8 >> num);
}
void flow_led_off(uint16_t num)
{
    GPIO_SetBits(GPIOG, GPIO_Pin_8 >> num);
}
void flow_led_toggle(uint16_t num)
{
    GPIO_ToggleBits(GPIOG, GPIO_Pin_8 >> num);
}

void led_io_cinfiguration()
{
	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_2);
	GPIO_ResetBits(GPIOE, GPIO_Pin_6);
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	
}

