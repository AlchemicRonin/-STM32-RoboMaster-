#ifndef __EEPROM_H
#define __EEPROM_H

#include "main.h"
/*
ʹ�� STM32F �ڲ�Flash��EEPROM
������API �ӳ���
*/


void Write_config(void);  //д������
void LoadCaliInfo();	  //��ȡ����

#endif /* __EEPROM_H */

//------------------End of File----------------------------
