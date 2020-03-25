#ifndef _CRC_CHECK_H_
#define _CRC_CHECK_H_

#include "serial.h"

#define UP_REG_ID    0xA0  //up layer regional id		//�ϴ�����sof???��TMҲ��֪���Ǹ�ɶ�õ�
#define DN_REG_ID    0xA5  //down layer regional id //����ϵͳ����sof���������ϵͳ���ͽ��ն�������
#define HEADER_LEN   sizeof(frame_header_t)					//֡ͷ����
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#define PROTOCAL_FRAME_MAX_SIZE  200

typedef __packed struct
{
  uint8_t  sof;//����֡��ʼ�ֽڣ��̶�ֵΪ0xA5
  uint16_t data_length; //����֡��data�ĳ���
  uint8_t  seq;  //�����
  uint8_t  crc8;  //֡ͷCRC8У��	
	
} frame_header_t;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif

