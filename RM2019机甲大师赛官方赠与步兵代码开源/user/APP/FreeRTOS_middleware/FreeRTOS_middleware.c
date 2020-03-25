/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       freeRTOS_middle.c/h
  * @brief      freeRTOS的中间层，将滴答计时中断和统计任务用时的接口函数放到这里.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//设置调度中断定时器配置
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

//利用tim3作为任务统计用时，所有任务目前测试cpu利用率
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
 
