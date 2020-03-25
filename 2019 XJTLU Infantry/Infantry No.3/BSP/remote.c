/******************************************************************************
 * @file     remote.c
 * @brief    RMң�������ն˳���
 * @author   Liyi
 * @version  V1.2
 * @date     2018.3.19
 * @note
 * @par			 STM32F407ZGT6  ʹ��DMA��ʹ�ô���2  PD6����
 *
 * @version  V1.1
 * @date     2018.3.10
 * @note
 * @par			 STM32F407ZGT6  ����ʹ��DMA��ʹ�ô���1   PA10����
 *			
 * @version  V1.0
 * @date     2017.6.12
 * @par			����ʹ�ò�����100000	û����żУ��	ֹͣλ1
 *					���ڽ��ܵ������� u8 RXBuff[18];	һ�����ݰ�18���ֽ�
 *	
 * Copyright (C) 2017 SLDX Limited. All rights reserved.
 ******************************************************************************/

#include "main.h"


#define RX_BUF_SIZE 18		//DMA���ջ�������С
u8 RXBuff[RX_BUF_SIZE];		//DMA���ջ�����

/**
  * @brief  ң��������DMA��ʼ��
  * @param  None
  * @retval None
  * @note   RXBuff �������ݽ��ܻ���
  */
void Remote_uart1_init(void)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma�õ��ж�
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);  //DMA2ʱ��ʹ�� ---********************
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); //GPIOB7����ΪUSART2
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��PB7

  //USART1 ��ʼ������
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_Init(USART1, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���2 	
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  //ʹ�ܴ���2��DMA����     
	
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;//DMA1 ������5 �ж�ͨ��*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	
  /* ���� DMA Stream */
	DMA_DeInit(DMA2_Stream2); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART1->DR);//DMA�����ַ****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RXBuff;//DMA �洢��0��ַ �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ*******************************
  DMA_InitStructure.DMA_BufferSize = RX_BUF_SIZE;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// ʹ����ͨģʽ ********************************
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�е����ȼ�**********
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;//�洢��ͻ�����δ���*****
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);//��ʼ��DMA1 Stream5	
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream2,ENABLE);
	
}



/**
  * @brief  DMA�жϴ���ң��������
  * @param  None
  * @retval None
  * @note   RXBuff �������ݽ��ܻ���
	*					rm ң�������ݽṹ����������remote.h
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
		
		
		
		
		//ң����ֵ�׳��쳣
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
//  * @brief  ң�������ݴ���
//  * @param  None
//  * @retval rc RC�ṹ�����ͱ��������ͷ�ļ�
//  * @note   RXBuff �������ݽ��ܻ���
//  */
//extern unsigned char RXBuff[];		//���ڽ��ջ���
//RC GetRemoteData()
//{	
//	return rc;
//}
