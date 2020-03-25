#include "laser.h"
#include "stm32f4xx.h"

void laser_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOG, &GPIO_InitStructure);             //��ʼ��

    laser_on();
}
void laser_on(void)
{
    GPIO_SetBits(GPIOG, GPIO_Pin_13);
}
void laser_off(void)
{
    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}
