#include "main.h"
			
//得到的数据
volatile packed_angle real_angle = {0};

#define MPU6500_DATA_READY_EXIT_INIT() IMU_INT_Configure() //初始化mpu6500的 外部中断 使用PB8 外部中断线 8
#define MPU6500_DATA_READY_EXIT_IRQHandler EXTI9_5_IRQHandler //宏定义外部中断函数，使用了line8外部中断
#define MPU6500_DATA_READY_EXIT_Line EXTI_Line8 //宏定义外部中断线
//宏定义初始化SPI的DMA，同时设置SPI为8位，4分频
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                                 \
    {                                                                      \
        SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM);       \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Rx, ENABLE);                   \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Tx, ENABLE);                   \
        SPI5SetSpeedAndDataSize(SPI_BaudRatePrescaler_8, SPI_DataSize_8b); \
    }

#define MPU6500_SPI_DMA_Enable() SPI5_DMA_Enable(DMA_RX_NUM) // 开始一次SPI的DMA传输
//宏定义SPI的DMA传输中断函数以及传输中断标志位
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5
#define MPU6500_DMA_FLAG DMA_FLAG_TCIF5
//DMA的SPI 发送的buf，以INT_STATUS开始连续读取 DMA_RX_NUM大小地址的值
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] =
    {
        MPU_INT_STATUS | MPU_SPI_READ_MSB};
	

		

void IMU_Configure()
{
	MPU6500_DATA_READY_EXIT_INIT();
	MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);
}


//中断线配置
void IMU_INT_Configure()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void IMU_EXIT_STOP()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void IMU_EXIT_START()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void MPU6500_DATA_READY_EXIT_IRQHandler(void)
{
    if (EXTI_GetITStatus(MPU6500_DATA_READY_EXIT_Line) != RESET)
    {

        EXTI_ClearITPendingBit(MPU6500_DATA_READY_EXIT_Line);

//如果开启DMA传输 唤醒任务由DMA中断完成
        mpu6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();

    }
}

void MPU6500_DMA_IRQHandler(void)
{
    if (DMA_GetFlagStatus(MPU6500_DMA_Stream, MPU6500_DMA_FLAG))
    {
        DMA_ClearFlag(MPU6500_DMA_Stream, MPU6500_DMA_FLAG);
        mpu6500_SPI_NS_H();
		
        //数据处理	
		real_angle.last_yaw = real_angle.new_yaw;
		real_angle.new_yaw   = INS_Angle[0] * 57.3f;
		real_angle.diff_yaw = real_angle.new_yaw - real_angle.last_yaw;
		if(real_angle.diff_yaw > 180.0f)
				real_angle.yaw_counts--;
		else if(real_angle.diff_yaw < -180.0f)
			real_angle.yaw_counts++;
		real_angle.yaw = real_angle.new_yaw + real_angle.yaw_counts * 360.0f;
		real_angle.pitch = -INS_Angle[2] * 57.3f;
		real_angle.roll  = INS_Angle[1] * 57.3f;
		real_angle.gx = mpu6500_real_data.gyro[1] * 57.3f;
		real_angle.gy = mpu6500_real_data.gyro[0] * 57.3f;
		real_angle.gz = -mpu6500_real_data.gyro[2] * 57.3f;
		
    }
}
u8 run_flag = 0;//1 启动	//2 校准
u8 offset_OK_flag = 0;//1 校准完成	//2 未完成
u8 offset_OK_flag1 = 0;//1 校准完成	//2 未完成

void IMU_control_decide()
{
	//上电之后直接启动 或者 IMU校准
	while(1)//等待遥控器选择
	{
		if(rc.sr == 1 )//IMU校准
		{
			run_flag = 2;
			offset_OK_flag = 2;
			offset_OK_flag1 = 2;
			break;
		}    
		else if(rc.sr == 2 )//启动
		{
			run_flag = 1;
			offset_OK_flag = 2;
			offset_OK_flag1 = 2;
			break;
		}
		else
		{
			run_flag = 0;
			offset_OK_flag = 0;
			offset_OK_flag1 = 0;
		}
	}
}
