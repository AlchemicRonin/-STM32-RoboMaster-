/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       freeRTOS_middle.c/h
  * @brief      freeRTOS���м�㣬���δ��ʱ�жϺ�ͳ��������ʱ�Ľӿں����ŵ�����.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "FreeRTOS_Middleware.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//���õ����ж϶�ʱ������
void vPortSetupTimerInterrupt(void)
{
}

extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

//����tim3��Ϊ����ͳ����ʱ����������Ŀǰ����cpu������
volatile uint64_t FreeRTOSRunTimeTicks = 0;

void ConfigureTimeForRunTimeStats(void)
{
    FreeRTOSRunTimeTicks = 0;
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearFlag(TIM3, TIM_IT_Update);
        FreeRTOSRunTimeTicks++;
    }
}
 
