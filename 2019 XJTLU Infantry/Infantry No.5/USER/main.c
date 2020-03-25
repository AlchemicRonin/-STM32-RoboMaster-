#include "main.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(180);
	Remote_uart1_init();//Ò£¿Ø³õÊ¼»¯
	IMU_control_decide();
	BSP_Init();
	

	while(1)
	{
//		if(rc.sl == 3)
//			TIM5->CCR4 = 2000;
//		else if(rc.sl == 2)
//			TIM5->CCR4 = 2400;
//		else if(rc.sl == 1)
//			TIM5->CCR4 = 1000;
	}
}
