#include "main.h"

fifo_s_t* JudgeSystemUart3_RXFIFO;		//����ϵͳ���ݽ��ն���
u8 RX3Buff[RX3_BUF_SIZE];		//DMA���ջ�����
/**
  * @brief  ����3��ʼ��
  * @param  None
  * @retval None
  * @note   ʹ��DMA���գ��������ڿ����ж�stream 3 _CH5 PE7 PE8
  */
int JudgeSystem_init = 0;  //��ɾ�����ڲ鿴���д���ϵͳ�Ƿ��ʼ��

	
void JudgeSystem_uart7_init(void)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma�õ��ж�
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//ʹ��USART3ʱ��
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);  //DMA1ʱ��ʹ�� ---********************
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7); //GPIOB11����ΪUSART3
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; 
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOE, &GPIO_InitStructure); //��ʼ��

  //USART3 ��ʼ������
	USART_DeInit(UART7);
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//У��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//  USART_InitStructure.USART_Mode = USART_Mode_Rx ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_Init(UART7, &USART_InitStructure); //��ʼ������3
	
  USART_Cmd(UART7, ENABLE);  //ʹ�ܴ���3	
	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);//ʹ�ܽ����ж�
	USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);	//ʹ�ܴ���3����DMA
	
	
	
	
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//����3�ж�ͨ��*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1 ;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
  /* ���� DMA Stream */
	DMA_DeInit(DMA1_Stream3); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(UART7->DR);//DMA�����ַ****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RX3Buff;//DMA �洢��0��ַ �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ*******************************
  DMA_InitStructure.DMA_BufferSize = RX3_BUF_SIZE;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ ********************************
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�е����ȼ�**********
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Circular;//�洢��ͻ�����δ���*****
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);//��ʼ��DMA1 Stream5	
//	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);	//DMA���ж�
//DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);	//DMA���ж�
	DMA_Cmd(DMA1_Stream3,ENABLE);
	
	JudgeSystemUart3_RXFIFO = fifo_s_create(140);		//����ϵͳ���ݽ���Э��֡����
	
	/*********************************/
	JudgeSystem_init = 1 ;
}


/**
  * @brief  ����3�жϷ�����
  * @param  
  * @retval 
  * @note   None
  */
int USART_INT = 0 ;//��ɾ�������鿴�Ƿ�����ж�
u8 look_usart_length =0;

int64_t uart7_cnts = 0;
void UART7_IRQHandler(void)
{
	
	u8 num;			//���ڿ����ж�ʱ�����ݳ���
	
unpack_data_t p_obj;	//Э��֡�������ݽ����ṹ��	
	
	if(USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)  //�����ж�
	{
		uart7_cnts++;
		num = UART7->SR;
		num = UART7->DR;	//���жϱ�־λ
				
		/*********************************/	
		USART_INT =1;		//�жϽ����־λ
		
		DMA_Cmd(DMA1_Stream3,DISABLE);	//��DMA
	  num = RX3_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream3);	//��ȡ���ݳ���	
		
    look_usart_length=num;
		
		for(int i = 0; i < num; i++)	//�����������
		
    //if(!fifo_is_full(JudgeSystemUart3_RXFIFO))	//�ж϶Ӳ���
		
		fifo_s_put(JudgeSystemUart3_RXFIFO, RX3Buff[i]);		//���
			
		DMA1_Stream3->NDTR = RX3_BUF_SIZE;//�������ý������ݸ���
		
		DMA_Cmd(DMA1_Stream3,ENABLE);		//��DMA�����¿�ʼ��������
		
		p_obj.data_fifo = JudgeSystemUart3_RXFIFO;	//׼������Э��֡
		
		unpack_fifo_data(&p_obj, DN_REG_ID);	//����Э��֡���� �ж��������ͣ������룩
		

  } 
} 
int64_t last_7_cnts = 0;
int64_t now_7_cnts = 0;
int64_t _7_times = 0;
u8 _7_flag = 0;
int8_t judge_t(void)
{
	last_7_cnts = now_7_cnts;
	now_7_cnts = uart7_cnts;
	
	if(last_7_cnts == now_7_cnts)
	{
		
		_7_times ++;
//		if(_7_times >= 50)
//		{
//			judge_rece_mesg.power_heat_data.chassis_power_buffer = 0.0f;
//		}
		if(_7_times >= 1000)
			_7_flag = 1;
		else 
			_7_flag = 0;
	}
	else 
	{
		_7_flag = 0;
		_7_times = 0;
	}
	return _7_flag;
}
/*********************DEBUG******************************************************************************/
//unpack_data_t p_obj;	//Э��֡�������ݽ����ṹ��

//void GETTTTT()	//Э��֡���ݽ���
//{
//	p_obj.data_fifo = JudgeSystemUart3_RXFIFO;
//	
//	unpack_fifo_data(&p_obj, 0xA5);	//����Э��֡���� �ж��������ͣ������룩
//	
//}






