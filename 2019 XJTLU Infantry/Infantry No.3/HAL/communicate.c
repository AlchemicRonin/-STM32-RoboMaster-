#include "main.h"
uint8_t look_usat_byte ;

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)		//解析协议帧队列数据包
{
  uint8_t byte = 0;
  
  while ( fifo_used_count(p_obj->data_fifo) )		//协议帧fifo元素数目
  {
   byte = fifo_s_get(p_obj->data_fifo);	//出队
		look_usat_byte= byte;
    switch(p_obj->unpack_step)			//unpack 步骤
    {
      case STEP_HEADER_SOF:		//SOP域  帧头
					 {
						if(byte == sof)
						{
							 p_obj->unpack_step = STEP_LENGTH_LOW;			//下一步
								p_obj->protocol_packet[p_obj->index++] = byte;		//协议帧队列数据存入数组
						 }
							else
							{
							 p_obj->index = 0;
						 }
					 }break;
      
      case STEP_LENGTH_LOW:		//协议帧数据域长度低8位
						{
							p_obj->data_len = byte;
							p_obj->protocol_packet[p_obj->index++] = byte;
							p_obj->unpack_step = STEP_LENGTH_HIGH;			//下一步
					 }break;

      case STEP_LENGTH_HIGH:	//协议帧数据域长度高8位
						{
							p_obj->data_len |= (byte << 8);
							p_obj->protocol_packet[p_obj->index++] = byte;

							if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))		//如果FramerHeader帧头中的数据长度小于实际接收的协议帧的数据域的字节数
							{
							 p_obj->unpack_step = STEP_FRAME_SEQ;	//返回第一步 帧头SOP域  
						 }
							else
							{
								p_obj->unpack_step = STEP_HEADER_SOF;				//下一步
								p_obj->index = 0;
						 }
					 }break;
    
      case STEP_FRAME_SEQ:	//包序号
						{
						 p_obj->protocol_packet[p_obj->index++] = byte;		//协议帧队列数据存入数组
						 p_obj->unpack_step = STEP_HEADER_CRC8;
						}break;

      case STEP_HEADER_CRC8:	//FramerHeader  CRC8校验
						{
							p_obj->protocol_packet[p_obj->index++] = byte;		//协议帧队列数据存入数组

							if (p_obj->index == HEADER_LEN)
							{
								if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, HEADER_LEN) )	//FramerHeader crc8校验通过
								{
								 p_obj->unpack_step = STEP_DATA_CRC16;					//下一步
								}
								else
								{
								 p_obj->unpack_step = STEP_HEADER_SOF;		//返回第一步 帧头SOP域  
								 p_obj->index = 0;
							 }
							}
					 }break;  

      case STEP_DATA_CRC16:		//协议帧的数据域的CRC16校验
						{
						 if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))		//判断协议帧队列数据是否全部存入数组
							{
								p_obj->protocol_packet[p_obj->index++] = byte;  		//协议帧队列数据存入数组
						 }
							if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))	//协议帧的数据域解析完（全部协议帧数据存入数组）
							{
							 p_obj->unpack_step = STEP_HEADER_SOF;
							 p_obj->index = 0;

							 if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )	//通过crc16校验
								{
//            if (sof == UP_REG_ID)	//再次判断是上传的自定义数据
//            {
//              pc_data_handler(p_obj->protocol_packet);
//            }
//            else  //DN_REG_ID	//确认是裁判系统发出的数据
//            {
										judgement_data_handler(p_obj->protocol_packet);		//对裁判系统的协议帧 拆包判断 //根据不同命令码进行相关详细信息解析
//            }
							 }
						 }
						}break;

      default:
							{
								p_obj->unpack_step = STEP_HEADER_SOF;
							 p_obj->index = 0;
							}break;
}
  }
}


/**
  * @brief  协议帧打包
  * @param  uint16_t 	cmd_id 	 0x0100 自定义数据
  * @param  uint8_t 	*p_data  数据帧的数据部分
  * @param  uint16_t 	len			 数据帧的数据部分的长度
  * @param  uint8_t 	sof			 FRAMEHEADER帧的SOF域 
  * @param  uint8_t 	*tx_buf	 frame_header_t结构体指针
  * @retval txbuf
  * @note   None
  */
static uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)	//
{
  memset(tx_buf, 0, tx_buf_length);
  static uint8_t seq;
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;	  //FRAMEHEADER帧长度
  frame_header_t *p_header = (frame_header_t*)tx_buf;							//数据转化，也是赋值
  
  p_header->sof          = sof;
  p_header->data_length  = len;
  p_header->seq          = 0;
  p_header->seq          = seq++;		//发送数据序列号
  
  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);		//赋值命令码  命令码数据结构类型不同，强制转化
  Append_CRC8_Check_Sum(tx_buf, HEADER_LEN);		//FRAMEHEADER帧数据CRC8校验
	//
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);			//赋值数据帧的数据部分
  Append_CRC16_Check_Sum(tx_buf, frame_length);		//数据帧的数据部分CRC16校验
  
  return tx_buf;
}

  //uint16_t data_length = p_header->data_length;		//p_header是上面定义的局部变量
 // uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN); //指针操作 一次性读取两个字节的命令码数据
 // uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;	//数据部分的指针
/**
  * @brief  客户端显示数据发送（上传数据到裁判系统）
  * @param  4个自定义类型数据
  * @retval None
  * @note   None
  */
void client_show_data_send(float data1, float data2, float data3, uint8_t mask)
{
	
	client_custom_data_t 	updata ;
	uint8_t 						tx_buf[tx_buf_length] = {0};//tx_buf_length
	uint16_t 						i 										= 0;
	
	updata.data_cmd_id = 0xD180;
	updata.send_ID = 0x01;
	updata.receiver_ID = 0x0101;
	updata.data1 = data1;
	updata.data2 = data2;
	updata.data3 = data3;
	updata.masks = mask;
	
	protocol_packet_pack(ROBO_CONNECTION_DATA, (uint8_t *)&updata, sizeof(client_custom_data_t), DN_REG_ID, tx_buf);	//数据打包
//   memcpy();

	for(i = 0; i < tx_buf_length; i++)
	{
		//sendata
		USART_SendData(USART3, tx_buf[i]);
		while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	}
}
