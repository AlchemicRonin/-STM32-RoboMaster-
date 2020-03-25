#include "main.h"

fifo_s_t* JudgeSystemUart3_RXFIFO;		//裁判系统数据接收队列
u8 RX3Buff[RX3_BUF_SIZE];		//DMA接收缓冲区
/**
  * @brief  串口3初始化
  * @param  None
  * @retval None
  * @note   使用DMA接收，开启串口空闲中断stream 3 _CH5 PE7 PE8
  */
int JudgeSystem_init = 0;  //可删，用于查看裁判串口系统是否初始化

	
void JudgeSystem_uart7_init(void)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma用到中断
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//使能USART3时钟
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);  //DMA1时钟使能 ---********************
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7); //GPIOB11复用为USART3
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; 
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure); //初始化

  //USART3 初始化设置
	USART_DeInit(UART7);
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//  USART_InitStructure.USART_Mode = USART_Mode_Rx ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_Init(UART7, &USART_InitStructure); //初始化串口3
	
  USART_Cmd(UART7, ENABLE);  //使能串口3	
	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);//使能接受中断
	USART_DMACmd(UART7,USART_DMAReq_Rx,ENABLE);	//使能串口3接收DMA
	
	
	
	
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//串口3中断通道*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1 ;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
  /* 配置 DMA Stream */
	DMA_DeInit(DMA1_Stream3); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(UART7->DR);//DMA外设地址****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RX3Buff;//DMA 存储器0地址 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式*******************************
  DMA_InitStructure.DMA_BufferSize = RX3_BUF_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 ********************************
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级**********
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Circular;//存储器突发单次传输*****
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);//初始化DMA1 Stream5	
//	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);	//DMA满中断
//DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);	//DMA满中断
	DMA_Cmd(DMA1_Stream3,ENABLE);
	
	JudgeSystemUart3_RXFIFO = fifo_s_create(140);		//裁判系统数据接收协议帧队列
	
	/*********************************/
	JudgeSystem_init = 1 ;
}


/**
  * @brief  串口3中断服务函数
  * @param  
  * @retval 
  * @note   None
  */
int USART_INT = 0 ;//可删，用来查看是否进入中断
u8 look_usart_length =0;

int64_t uart7_cnts = 0;
void UART7_IRQHandler(void)
{
	
	u8 num;			//串口空闲中断时的数据长度
	
unpack_data_t p_obj;	//协议帧队列数据解析结构体	
	
	if(USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)  //接收中断
	{
		uart7_cnts++;
		num = UART7->SR;
		num = UART7->DR;	//清中断标志位
				
		/*********************************/	
		USART_INT =1;		//中断进入标志位
		
		DMA_Cmd(DMA1_Stream3,DISABLE);	//关DMA
	  num = RX3_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream3);	//获取数据长度	
		
    look_usart_length=num;
		
		for(int i = 0; i < num; i++)	//所有数据入队
		
    //if(!fifo_is_full(JudgeSystemUart3_RXFIFO))	//判断队不满
		
		fifo_s_put(JudgeSystemUart3_RXFIFO, RX3Buff[i]);		//入队
			
		DMA1_Stream3->NDTR = RX3_BUF_SIZE;//重新设置接收数据个数
		
		DMA_Cmd(DMA1_Stream3,ENABLE);		//开DMA，重新开始接收数据
		
		p_obj.data_fifo = JudgeSystemUart3_RXFIFO;	//准备解析协议帧
		
		unpack_fifo_data(&p_obj, DN_REG_ID);	//解析协议帧队列 判断数据类型（命令码）
		

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
//unpack_data_t p_obj;	//协议帧队列数据解析结构体

//void GETTTTT()	//协议帧数据解析
//{
//	p_obj.data_fifo = JudgeSystemUart3_RXFIFO;
//	
//	unpack_fifo_data(&p_obj, 0xA5);	//解析协议帧队列 判断数据类型（命令码）
//	
//}






