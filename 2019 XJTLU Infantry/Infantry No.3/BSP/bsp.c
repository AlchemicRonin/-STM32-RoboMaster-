#include "main.h"

uint8_t mpu_temp=2;//MPU初始化标志位

void BSP_Init(void)						//底层初始化
{
	
	//初始化
	temperature_ADC_init();
	while(!get_set_temp());
//	while (ist8310_init() != IST8310_NO_ERROR);
	Read_Offset();
	while(mpu_temp) mpu_temp = mpu6500_init();//MPU初始化	
	buzzer_init(30000, 90);
	led_configuration();
//	while (ist8310_init() != IST8310_NO_ERROR);
	All_Pid_Configuration(pid);//pid参数初始化
	ALL_Pid_Incr_Configuration();
	miniPC_Init();//miniPC数据初始化
	laser_configuration(); //激光
	SHOOT_PLAN_Init();	//预置位
    power_ctrl_configuration();//24输出控制口 初始化
	ramp_init();//摩擦轮数据初始化
	chassis_init();	//底盘数据初始化
	TIM6_Configure();	
	TIM1_FireMotor_Configure();
	TIM5_Pwm_Configure();
	//CAN初始化
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	 //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
	delay_ms(1000);//等CAN发过来第一次数据
	
	//开中断	
	
	miniPC_uart6_init();//miniPC通信初始化
	JudgeSystem_uart7_init();//裁判系统
	IMU_Configure();//MPU中断初始化
	TIM6_Start();
}
