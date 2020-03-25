#include "main.h"

uint8_t mpu_temp=2;//MPU��ʼ����־λ

void BSP_Init(void)						//�ײ��ʼ��
{
	
	//��ʼ��
	temperature_ADC_init();
	while(!get_set_temp());
//	while (ist8310_init() != IST8310_NO_ERROR);
	Read_Offset();
	while(mpu_temp) mpu_temp = mpu6500_init();//MPU��ʼ��	
	buzzer_init(30000, 90);
	led_configuration();
//	while (ist8310_init() != IST8310_NO_ERROR);
	All_Pid_Configuration(pid);//pid������ʼ��
	ALL_Pid_Incr_Configuration();
	miniPC_Init();//miniPC���ݳ�ʼ��
	laser_configuration(); //����
	SHOOT_PLAN_Init();	//Ԥ��λ
    power_ctrl_configuration();//24������ƿ� ��ʼ��
	ramp_init();//Ħ�������ݳ�ʼ��
	chassis_init();	//�������ݳ�ʼ��
	TIM6_Configure();	
	TIM1_FireMotor_Configure();
	TIM5_Pwm_Configure();
	//CAN��ʼ��
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	 //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
	delay_ms(1000);//��CAN��������һ������
	
	//���ж�	
	
	miniPC_uart6_init();//miniPCͨ�ų�ʼ��
	JudgeSystem_uart7_init();//����ϵͳ
	IMU_Configure();//MPU�жϳ�ʼ��
	TIM6_Start();
}
