#ifndef _SERIAL
#define _SERIAL
#include "sys.h"




#define SEE_RX_BUF_SIZE 18		//DMA接收缓冲区大小



//装甲信息结构体定义
typedef union {
	int16_t d;
	unsigned char c[2];
}int16uchar;


typedef union {
	float f;
	unsigned char c[4];
}float2uchar;


typedef struct {
	unsigned char mode; // 1 auto 2 sudoku
	int16uchar SEE_yaw_angle;
	int16uchar SEE_pitch_angle;
	int16uchar SEE_yaw_speed;
	float2uchar SEE_shoot_speed;
	uint8_t    which;
	uint8_t    pos;

	float2uchar x;
	float2uchar y;
	float2uchar z;
}VisionData;

typedef struct{
	float yaw_offset;
	float pitch_offset;

}Final_Offset_Angle;
extern VisionData enemy_position;//结构体中的内容即可获取云台矫正角度

extern u8 SEE_RXBuff[SEE_RX_BUF_SIZE];		//DMA接收缓冲区
extern u8 SEE_rx_data[18];


/////////////////////////////////////////////////////
//***测试使用***
//jscope不能看结构体中的共用体的变量，声明全局变量作为中间变量以供查询
extern unsigned char mode;
extern int16_t SEE_yaw_angle;
extern int16_t SEE_pitch_angle;
extern float z;
/////////////////////////////////////////////////////

void getVisionData(u8* data,VisionData* enemy_position);
void miniPC_uart6_init(void);
void miniPC_uart6_tx(u8 *USART_RX_BUF ,int len);
//extern Final_Offset_Angle FOA;
float usart_IIRLowPass(float x);
#endif



