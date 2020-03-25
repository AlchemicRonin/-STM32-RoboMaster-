#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "main.h"

#define RATE_BUF_SIZE  6

//���̵�� can1 id
#define CAN_ID_CM1		0x201
#define CAN_ID_CM2		0x202
#define CAN_ID_CM3		0x203
#define CAN_ID_CM4		0x204
//��̨��� can1 id
#define CAN_ID_YAW		0x205
#define CAN_ID_PITCH	0x206
//������� can2 id
#define CAN_ID_CM_FEED  0x207
#define POWER      0x110
typedef struct
{
	uint8_t can_tx;
	uint8_t can_rx;
	uint8_t can_rx1;
}Can_Data;
extern Can_Data can_data;
typedef struct{
	int32_t raw_value;   					 //���������������ԭʼֵ
	int32_t last_raw_value;					 //��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;							 //���α�����֮��Ĳ�ֵ
	int32_t temp_count;                      //������
	uint8_t buf_count;						 //�˲�����buf��
	int32_t ecd_bias;						 //��ʼ������ֵ	
	int32_t ecd_raw_rate;					 //ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	     //buf��for filter
	int32_t round_cnt;						 //Ȧ��
	int32_t filter_rate;					 //�ٶ�
	int32_t filter_rate_max;				 //����ٶ�
	float ecd_angle;						 //�Ƕ�
	float init_angle; 						 //��ʼ��һ������
	int16_t current;
	
}Encoder;

void CanReceiveMsgProcess(CanRxMsg *message);
void getEncoderData(volatile Encoder *v, CanRxMsg * msg);
void get_Data(Can_Data *data, CanRxMsg * msg);
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder CMFeedEncoder;
#endif

