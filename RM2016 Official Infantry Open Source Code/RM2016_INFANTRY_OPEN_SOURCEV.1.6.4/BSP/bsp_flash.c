/*
*********************************************************************************************************
*                                     DJI BOARD SUPPORT PACKAGE
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           INFANTRY_CONTROL_BOARD_V2
*
* Filename      : bsp_flash.c
* Version       : V0.10
* Programmer(s) : SC.H
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "bsp_flash.h"

/*
*********************************************************************************************************
*                                               STMFLASH_ReadByte
*
* ���� : ��flash�ж�ȡһ���ֽ�
*
* ���� : faddr��flash��ַ
*
* ��� : ��ȡ�����ֽ�
*
* ���� : �ڲ�����
*
* ˵�� : ��
*********************************************************************************************************
*/
static u8 BSP_FLASH_ReadByte(u32 faddr)
{
	return *(vu8*)faddr; 
} 


/*
*********************************************************************************************************
*                                               STMFLASH_GetFlashSector
*
* ���� : ��ȡָ����ַ����flash�Ŀ��׵�ַ
*
* ���� : addr��flash��ַ
*
* ��� : ����ҳ�׵�ַ
*
* ���� : �ⲿ����
*
* ˵�� : ��߿�֧��1Mflash��STM32оƬ
*********************************************************************************************************
*/
uint16_t BSP_FLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}


/*
*********************************************************************************************************
*                                               BSP_FLASH_Write
*
* ���� : ��ָ����ַ��ʼд��ָ�����ȵ�����
*
* ���� : WriteAddr����ʼ��ַ
*        pBuffer��Ҫд�����ݴ洢���׵�ַ
*        ByteToWrite��Ҫд����ֽ�����ע�����ֽ��������д���֣���Ҫ����������4
*
* ��� : 0:д��ʧ��     1��д��ɹ�
*
* ���� : �ⲿ����
*
* ˵�� : ��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
*        д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
*        д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
*        û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
*
*        ��������OTP����Ҳ��Ч!��������дOTP��!
*        OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
*********************************************************************************************************
*/
u8 BSP_FLASH_Write(u32 WriteAddr, u8 *pBuffer, u32 ByteToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
    u8 res=1;
    u32 addrx=0;
    u32 endaddr=0;	
		int i = 0;
    
    u32 start_sector = 0;
    u32 end_sector = 0;
    
    if(WriteAddr<STM32_FLASH_BASE)return 0;	//�Ƿ���ַ
		FLASH_Unlock();									//���� 
    FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
		addrx=WriteAddr;				//д�����ʼ��ַ
		endaddr=WriteAddr+ByteToWrite;	//д��Ľ�����ַ
    
    start_sector = BSP_FLASH_GetFlashSector(addrx);
    end_sector = BSP_FLASH_GetFlashSector(endaddr);
    
		if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
		{             
				 for(i = start_sector; i <= end_sector; i += 8)
				 {
						 status = FLASH_EraseSector(i, VoltageRange_3);
						 if(status!=FLASH_COMPLETE)
						 {
								 res = 0;	//����������
								 break;
						 }           
				 }           
		}
			
		if(status == FLASH_COMPLETE)
		{
			while(WriteAddr < endaddr)//д����
			{
				if(FLASH_ProgramByte(WriteAddr,*pBuffer) != FLASH_COMPLETE)//д������
				{ 
					res = 0;	//д���쳣
									break;
				}
				WriteAddr+=1;
				pBuffer = (u8*)pBuffer+1;
			} 
		}
   
    FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
    FLASH_Lock();//����
    return res;
} 


/*
*********************************************************************************************************
*                                               BSP_FLASH_Write
*
* ���� : ��ָ����ַ��ʼ����ָ�����ȵ�����
*
* ���� : ReadAddr����ʼ��ַ
*        pBuffer��Ҫд�����ݴ洢���׵�ַ
*        ByteToWrite��Ҫ�������ֽ�����ע�����ֽ��������д���֣���Ҫ����������4
*
* ��� : ��
*
* ���� : �ⲿ����
*
* ˵�� : ��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
*        д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
*        д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
*        û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
*
*        ��������OTP����Ҳ��Ч!��������дOTP��!
*        OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
*********************************************************************************************************
*/
void BSP_FLASH_Read(u32 ReadAddr, u8 *pBuffer, u32 ByteToRead)   	
{
	u32 i;
//    u32 NumToRead = ((ByteToRead+3u)&(~3u))/4u;
	for(i=0;i<ByteToRead;i++)
	{
		pBuffer[i]=BSP_FLASH_ReadByte(ReadAddr);//��ȡ1���ֽ�.
		ReadAddr+=1;//ƫ��1���ֽ�.	
	}
}




















