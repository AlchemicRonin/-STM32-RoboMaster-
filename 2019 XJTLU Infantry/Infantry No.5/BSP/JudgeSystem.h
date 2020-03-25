
#ifndef _JudgeSystem_H_
#define _JudgeSystem_H_	 

#include "main.h"

#define RX3_BUF_SIZE 160		//DMA接收缓冲区大小

void JudgeSystem_uart7_init(void);
int8_t judge_t(void);
extern receive_judge_t judge_rece_mesg;
//extern u8 RX3Buff[RX3_BUF_SIZE];		//DMA接收缓冲区

#endif

