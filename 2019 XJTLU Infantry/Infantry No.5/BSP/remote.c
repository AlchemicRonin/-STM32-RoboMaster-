/******************************************************************************
 * @file     remote.c
 * @brief    RM遥控器接收端程序
 * @author   Liyi
 * @version  V1.2
 * @date     2018.3.19
 * @note
 * @par			 STM32F407ZGT6  使用DMA，使用串口2  PD6引脚
 *
 * @version  V1.1
 * @date     2018.3.10
 * @note
 * @par			 STM32F407ZGT6  加入使用DMA，使用串口1   PA10引脚
 *			
 * @version  V1.0
 * @date     2017.6.12
 * @par			串口使用波特率100000	没有奇偶校验	停止位1
 *					串口接受到的数据 u8 RXBuff[18];	一个数据包18个字节
 *	
 * Copyright (C) 2017 SLDX Limited. All rights reserved.
 ******************************************************************************/

#include "main.h"


#define RX_BUF_SIZE 18		//DMA接收缓冲区大小
u8 RXBuff[RX_BUF_SIZE];		//DMA接收缓冲区

/**
  * @brief  遥控器串口DMA初始化
  * @param  None
  * @retval None
  * @note   RXBuff 串口数据接受缓存
  */
void Remote_uart1_init(void)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma用到中断
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);  //DMA2时钟使能 ---********************
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); //GPIOB7复用为USART2
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化PB7

  //USART1 初始化设置
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_Init(USART1, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART1, ENABLE);  //使能串口2 	
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  //使能串口2的DMA接收     
	
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;//DMA1 数据流5 中断通道*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	
  /* 配置 DMA Stream */
	DMA_DeInit(DMA2_Stream2); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART1->DR);//DMA外设地址****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RXBuff;//DMA 存储器0地址 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式*******************************
  DMA_InitStructure.DMA_BufferSize = RX_BUF_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用普通模式 ********************************
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级**********
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;//存储器突发单次传输*****
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);//初始化DMA1 Stream5	
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream2,ENABLE);
	
}



/**
  * @brief  DMA中断处理遥控器数据
  * @param  None
  * @retval None
  * @note   RXBuff 串口数据接受缓存
	*					rm 遥控器数据结构体变量，详见remote.h
  */
RC rc;
void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
	{
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
		
		rc.cnts++;
		
		rc.R_x = 1024 - ((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff); //!< Channel 0
		rc.R_y = 1024 - (((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff); //!< Channel 1
		rc.L_x = 1024 - (((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff);//!< Channel 2
		rc.L_y = 1024 - (((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff); //!< Channel 3
		rc.sl = ((RXBuff[5] >> 4)& 0x000C) >> 2; //!< Switch left
		rc.sr = ((RXBuff[5] >> 4)& 0x0003); //!< Switch left
		
		rc.mouse_x = RXBuff[6] | (RXBuff[7] << 8); //!< Mouse X axis
		rc.mouse_y = RXBuff[8] | (RXBuff[9] << 8); //!< Mouse Y axis
		rc.mouse_z = RXBuff[10] | (RXBuff[11] << 8); //!< Mouse Z axis
		rc.mouse_l = RXBuff[12]; //!< Mouse Left Is Press ?
		rc.mouse_r = RXBuff[13]; //!< Mouse Right Is Press ?
		rc.key = RXBuff[14] | (RXBuff[15] << 8); //!< KeyBoard value
		rc.W = (rc.key&0x01)>>0;
		rc.S = (rc.key&0x02)>>1;
		rc.A = (rc.key&0x04)>>2;
		rc.D = (rc.key&0x08)>>3;
		rc.Shift = (rc.key&0x10)>>4;
		rc.Ctrl = (rc.key&0x20)>>5;
		rc.Q = (rc.key&0x40)>>6;
		rc.E  = (rc.key&0x80)>>7;
		
		
		
		
		//遥控器值抛出异常
		rc.R_x = rc.R_x > 660 ? 0 : rc.R_x;
		rc.R_x = rc.R_x < -660 ? 0 : rc.R_x;
		
		rc.R_y = rc.R_y > 660 ? 0 : rc.R_y;
		rc.R_y = rc.R_y < -660 ? 0 : rc.R_y;
		
		rc.L_x = rc.L_x > 660 ? 0 : rc.L_x;
		rc.L_x = rc.L_x < -660 ? 0 : rc.L_x;
		
		rc.L_y = rc.L_y > 660 ? 0 : rc.L_y;
		rc.L_y = rc.L_y < -660 ? 0 : rc.L_y;
		
		rc.mouse_x = rc.mouse_x > 1024 ? 0 : rc.mouse_x;
		rc.mouse_x = rc.mouse_x < -1024 ? 0 : rc.mouse_x;
		rc.mouse_y = rc.mouse_y > 1024 ? 0 : rc.mouse_y;
		rc.mouse_y = rc.mouse_y < -1024 ? 0 : rc.mouse_y;
		
	}
}

void remote_off_line(void)
{
	rc.last_cnts = rc.now_cnts;
	rc.now_cnts = rc.cnts;
	
	if(rc.last_cnts == rc.now_cnts)
	{
		rc.off_line_times++;
		if(rc.off_line_times >= 1000)
			rc.off_line_flag = 1;
		else 
			rc.off_line_flag = 0;
		if(rc.off_line_times >= 5000)
		{
				__set_FAULTMASK(1);	
				NVIC_SystemReset();
		}
	}
	else 
	{
		rc.off_line_times = 0;
		rc.off_line_flag = 0;
	}
}
///**
//  * @brief  遥控器数据处理
//  * @param  None
//  * @retval rc RC结构体类型变量，详见头文件
//  * @note   RXBuff 串口数据接受缓存
//  */
//extern unsigned char RXBuff[];		//串口接收缓存
//RC GetRemoteData()
//{	
//	return rc;
//}
