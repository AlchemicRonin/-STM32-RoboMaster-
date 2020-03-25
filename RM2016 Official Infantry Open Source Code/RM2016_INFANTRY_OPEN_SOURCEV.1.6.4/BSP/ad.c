#include "main.h"

//24V电源检测
//连接在PA2引脚上
//18K-2K电阻分压，电压范围如果不在22---26.5，就蜂鸣器报警。

//ADC123_IN2(PA2)

void Power_Detection_Configuration(void)
{
    ADC_InitTypeDef adc;
    GPIO_InitTypeDef gpio;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&gpio);
    
    adc.ADC_Resolution = ADC_Resolution_10b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1,&adc);
    
    ADC_Cmd(ADC1,ENABLE);
    
    ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_56Cycles);
}


//电源电压检测
void Power_Detect(void)
{
    unsigned int i;
    unsigned int AD_Value;
    double Voltage_Value;

    ADC_SoftwareStartConv(ADC1);

    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==RESET);
    AD_Value = ADC_GetConversionValue(ADC1);
    ADC_Cmd(ADC1,DISABLE);  //停止AD采样

    Voltage_Value = (AD_Value/1024.0)*33.04;
    //printf("Power voltage value = %2.1f\r\n",Voltage_Value);

    //if(Voltage_Value > 26.5 || Voltage_Value < 22)
    if(0)
    {
        for(i=0;i<7;i++)
        {
            BUZZER_ON();
            delay_ms(200);
            BUZZER_OFF();
            delay_ms(250);
        }
        BUZZER_ON();
        delay_ms(300);
        BUZZER_OFF();
        while(1);
    }
    else
    {
        BUZZER_ON();
        delay_ms(70);
        BUZZER_OFF();
        delay_ms(70);
        BUZZER_ON();
        delay_ms(70);
        BUZZER_OFF();
        delay_ms(70);        
    }
}
