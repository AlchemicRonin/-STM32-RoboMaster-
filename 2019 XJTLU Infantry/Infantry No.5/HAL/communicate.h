
#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__

#include "stm32f4xx.h"
#include "data_fifo.h"
#include "crc.h"


#define tx_buf_length 	(HEADER_LEN + CMD_LEN + sizeof(client_custom_data_t) + CRC_LEN)		//串口发送缓冲
	
typedef enum				//数据解析时的步骤
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;		

typedef enum
{
  UART_IDLE_IT     = 0,
  UART_DMA_HALF_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef struct
{
  USART_InitTypeDef *huart;	//*******************************
  fifo_s_t           *data_fifo;
  uint16_t           buff_size;
  uint8_t            *buff[2];
  uint16_t           read_index;
  uint16_t           write_index;
} uart_dma_rxdata_t;

typedef struct			//数据解析时用到的结构体
{
  fifo_s_t       *data_fifo;		//队列
  frame_header_t *p_header;			//帧头
  uint16_t       data_len;			//数据包的数据部分长度
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];		//把队列数据存入数组
  unpack_step_e  unpack_step;		//数据解析步骤
  uint16_t       index;					//队列数据存入数组之后的下标号
} unpack_data_t;

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);		//协议帧打包

/* dma double_buffer data puts to unpack_buffer */
//void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type);

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof);		//协议帧队列解析

//void data_upload_handler(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);

//uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof);

void client_show_data_send(float data1, float data2, float data3, uint8_t mask);		//数据上传到裁判系统客户端

#endif
