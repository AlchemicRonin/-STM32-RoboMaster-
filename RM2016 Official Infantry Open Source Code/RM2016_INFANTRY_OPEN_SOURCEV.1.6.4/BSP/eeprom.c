/* eeprom.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.net
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-05-05
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
���ܣ�
��Flash����EEPROM ���ڱ���ƫ�úͱ궨����
------------------------------------
 */			  

#include "eeprom.h"
#include "main.h"
#include "stm32f4xx.h"



////����ǰ����д��flash
//void Write_config(void){
//	int16_t i;
//	__packed int16_t *ptr = &gAppParamStruct.is_good;   //why should add the packed
//	uint32_t ptemp_addr = PAGE_Config;
//	FLASH_Unlock();
// 	FLASH_EraseSector(FLASH_Sector_11,VoltageRange_3);   //�������һ��sector,��configд���������
//	for(i=0;i<sizeof(Config)/sizeof(int16_t);i++){
//	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
//	 	ptemp_addr+=2;
//	}
//	FLASH_Lock();
//}

//------------------End of File----------------------------
