/* eeprom.c file
编写者：lisn3188
网址：www.chiplab7.net
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-05-05
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
将Flash用作EEPROM 用于保存偏置和标定数据
------------------------------------
 */			  

#include "eeprom.h"
#include "main.h"
#include "stm32f4xx.h"



////将当前配置写入flash
//void Write_config(void){
//	int16_t i;
//	__packed int16_t *ptr = &gAppParamStruct.is_good;   //why should add the packed
//	uint32_t ptemp_addr = PAGE_Config;
//	FLASH_Unlock();
// 	FLASH_EraseSector(FLASH_Sector_11,VoltageRange_3);   //擦除最后一个sector,讲config写入这个区中
//	for(i=0;i<sizeof(Config)/sizeof(int16_t);i++){
//	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
//	 	ptemp_addr+=2;
//	}
//	FLASH_Lock();
//}

//------------------End of File----------------------------
