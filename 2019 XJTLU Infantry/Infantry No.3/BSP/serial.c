#include "main.h"
u8 SEE_RXBuff[SEE_RX_BUF_SIZE];		//DMA���ջ�����
u8 SEE_rx_data[18];
VisionData enemy_position;
Final_Offset_Angle FOA = {0, 0};
/**
  * @brief  miniPC����DMA��ʼ��
  * @param  None
  * @retval None
  * @note   SEE_RXBuff �������ݽ��ܻ���
  */
	
//float b[3] = { 0.0155, 0.0155, 0};
//float a[3] = {1.0000, -0.9691, 0};

//float usart_xBuf[3] = {0, 0, 0};
//float usart_yBuf[3] = {0, 0, 0};
double b[4] = { 0.00000376, 0.00001127, 0.00001127, 0.00000376};
double a[4] = { 1.0000, -2.9372, 2.8763, -0.9391} ;

double usart_xBuf[4] = {0, 0, 0, 0};
double usart_yBuf[4] = {0, 0, 0, 0};

	unsigned char  mode;
	int16_t SEE_yaw_angle;
	int16_t SEE_pitch_angle;
	float z;
	

void miniPC_uart6_init(void)
{
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma�õ��ж�
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);;//ʹ��USART1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //DMA2ʱ��ʹ�� ---********************
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;         
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOG, &GPIO_InitStructure);  /* TXIO */  

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;                
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOG, &GPIO_InitStructure);  /* RXIO */

  //USART1 ��ʼ������
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART6, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���1
	
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);  //ʹ�ܴ���1��DMA����     
	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;//DMA1 ������5 �ж�ͨ��*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	
  /* ���� DMA Stream */
	DMA_DeInit(DMA2_Stream1); 	
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);//�ȴ�DMA������
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART6->DR);//DMA�����ַ****************************
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SEE_RXBuff;//DMA �洢��0��ַ �ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ*******************************
	DMA_InitStructure.DMA_BufferSize = SEE_RX_BUF_SIZE;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�е����ȼ�**********
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���*****
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���

	DMA_Init(DMA2_Stream1, &DMA_InitStructure);//��ʼ��DMA2 Stream5	
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream1,ENABLE);
	
	
	//�Ӿ���Ϣ��ʼ����ֹ������δ�������
	enemy_position.mode = 0;
	enemy_position.SEE_pitch_angle.d = 0;
	enemy_position.SEE_yaw_angle.d = 0;
	enemy_position.SEE_shoot_speed.f = 0;

	
	
}





void DMA2_Stream1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1)!=RESET)
	{
		DMA_Cmd(DMA2_Stream1, DISABLE); //�ر�DMA,��ֹ�������������		
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//���DMA2_Steam7������ɱ�־
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);		
		memcpy(SEE_rx_data,SEE_RXBuff,18);
		getVisionData(SEE_rx_data,&enemy_position);
		DMA_Cmd(DMA2_Stream1, ENABLE); 
	}
}



void miniPC_uart6_tx(u8 *USART_RX_BUF ,int len)
{
			int t;
			for(t=0;t<len;t++)
			{
				USART_SendData(USART6, USART_RX_BUF[t]);         //�򴮿�1��������
				while(USART_GetFlagStatus(USART6,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
}
int64_t usart_see = 0;
int16_t yaw_yaw = 0;
int16_t pitch_pitch = 0;

u8 vision_flag = 0;
void getVisionData(u8* data,VisionData* enemy_position)
{
//	usart_see++;
	if(data[0] == 0xA5 && Verify_CRC16_Check_Sum(data,18)) 
	{
		enemy_position->mode = data[1];
		
		enemy_position->SEE_pitch_angle.c[0] = data[2];
		enemy_position->SEE_pitch_angle.c[1] = data[3];
		
		enemy_position->SEE_yaw_angle.c[0] = data[4];
		enemy_position->SEE_yaw_angle.c[1] = data[5];
		
		enemy_position->SEE_yaw_speed.c[0] = data[6];
		enemy_position->SEE_yaw_speed.c[1] = data[7];
		
		enemy_position->SEE_shoot_speed.c[0] = data[8];
		enemy_position->SEE_shoot_speed.c[1] = data[9];
		enemy_position->SEE_shoot_speed.c[2] = data[10];
		enemy_position->SEE_shoot_speed.c[3] = data[11];
		
		enemy_position->z.c[0] = data[12];
		enemy_position->z.c[1] = data[13];
		enemy_position->z.c[2] = data[14];
		enemy_position->z.c[3] = data[15];
		

		FOA.yaw_offset   = (float)enemy_position->SEE_yaw_angle.d / 10.0f;
		FOA.pitch_offset = (float)enemy_position->SEE_pitch_angle.d / 10.0f;
		
		
		out[VISUAL_YAW_OFFSET] = Calculate_Current_Value_For_Err(&pid[VISUAL_YAW_OFFSET], FOA.yaw_offset);
		out[VISUAL_PITCH_OFFSET] = Calculate_Current_Value_For_Err(&pid[VISUAL_PITCH_OFFSET], FOA.pitch_offset);

	}	
	
}

float usart_IIRLowPass(float x)
{
	int i;
	//����֮ǰBuf��ǰ�ƶ�һ��λ�ã��Ա���֮ǰBuf�����ݣ�
	for(i = 3; i > 0; i--)
	{
		usart_yBuf[i] = usart_yBuf[i-1];
		usart_xBuf[i] = usart_xBuf[i-1];
	}
	usart_xBuf[0] = x;
	usart_yBuf[0] = 0;
	for(i = 1;i < 4;i++)
	{
		usart_yBuf[0] = usart_yBuf[0] + b[i]*usart_xBuf[i];
		usart_yBuf[0] = usart_yBuf[0] - a[i]*usart_yBuf[i];
	}
	usart_yBuf[0] = usart_yBuf[0] + b[0]*usart_xBuf[0];
	return usart_yBuf[0];
}
