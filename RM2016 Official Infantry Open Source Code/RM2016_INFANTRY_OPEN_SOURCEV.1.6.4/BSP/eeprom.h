#ifndef __EEPROM_H
#define __EEPROM_H

#include "main.h"
/*
使用 STM32F 内部Flash做EEPROM
引出的API 子程序
*/


void Write_config(void);  //写入配置
void LoadCaliInfo();	  //读取配置

#endif /* __EEPROM_H */

//------------------End of File----------------------------
