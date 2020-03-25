#include "main.h"

Can_Data can_data = {0};
volatile Encoder CM1Encoder      =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder      =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder      =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder      =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder    =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder  = 		{0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile Encoder CMFeedEncoder   =  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0};

union   
{  
     char pos_byte[8];
            
     struct
	 {
		 float x;
		 float y;
	 }POWER_T;		 
}byte2float;

int rx_cnt = 0;
int rx_id = 0;
void CanReceiveMsgProcess(CanRxMsg *message)
{
	rx_id = message->StdId;
	switch(message->StdId)
	{
		case CAN_ID_CM1:    	getEncoderData(&CM1Encoder, message);   	break;
            
		case CAN_ID_CM2:    	getEncoderData(&CM2Encoder, message); 	  break;

		case CAN_ID_CM3:  	  getEncoderData(&CM3Encoder, message);   	break;

		case CAN_ID_CM4:	    getEncoderData(&CM4Encoder, message); 	  break;

		case CAN_ID_YAW:	    getEncoderData(&GMYawEncoder, message);	  break;

		case CAN_ID_PITCH:	  getEncoderData(&GMPitchEncoder, message); break;
		
		case CAN_ID_CM_FEED:  getEncoderData(&CMFeedEncoder, message);  break;
		
		case POWER :					get_Data(&can_data, message);   					break;
	}
}

//int a11=7000;
void getEncoderData(volatile Encoder *v, CanRxMsg * msg)
{
//	a11+=20;
//	if(a11>7100){
//		a11=7000;
//	}
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
	
	if(fabs((float)v->filter_rate_max) < fabs((float)v->filter_rate))
	{
		v->filter_rate_max = v->filter_rate;
	}
	v->current = (uint16_t)((msg)->Data[4] << 8 | (msg)->Data[5]);
	
}

void get_Data(Can_Data *data, CanRxMsg * msg)
{
		if(msg->StdId == 0x110){
		for(int i = 0; i < 8; i++)
		byte2float.pos_byte[i] = msg->Data[i];
		data->can_rx = 	(int16_t)byte2float.POWER_T.x;
			data->can_rx1 = (int16_t)byte2float.POWER_T.y;
		}
}

