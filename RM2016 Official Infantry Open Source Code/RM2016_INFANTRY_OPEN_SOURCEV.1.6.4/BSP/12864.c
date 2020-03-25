#include "main.h"

#define LCD12864_CLK_H   GPIO_SetBits(GPIOC,GPIO_Pin_10)
#define LCD12864_CLK_L   GPIO_ResetBits(GPIOC,GPIO_Pin_10)

#define LCD12864_SID_H   GPIO_SetBits(GPIOC,GPIO_Pin_11)
#define LCD12864_SID_L   GPIO_ResetBits(GPIOC,GPIO_Pin_11)


#define Command_ClearScreen           0x01   //LCD����
#define Command_CursorReturn          0x02   //LCD����λ
#define Command_EntryMode_ACdowm      0x04   //DDRAM ��ַ��������AC���� 1,���治��λ
#define Command_EntryMode_ACup 	      0x06   //DDRAM ��ַ��������AC���� 1,���治��λ
#define Command_EntryMode_FrameRight  0x05   //������������λ
#define Command_EntryMode_FrameLeft   0x07   //������������λ
#define Command_DispControl_DispOff   0x08   //��ʾ��,�α��,�α�λ�÷��׹�
#define Command_DispControl_DispOn    0x0c   //��ʾ��,�α��,�α�λ�÷��׹�
#define Command_DispControl_ReverseOn 0x0d   //��ʾ��,�α��,�α�λ�÷��׿�
#define Command_DispControl_CursorOn  0x0e   //��ʾ��,�α꿪,�α�λ�÷��׹�
#define Command_DispControl_AllOn     0x0f   //��ʾ��,�α꿪,�α�λ�÷��׿�
#define Command_Function_4bits_basic  0x20   //4λ���ݣ�����ָ�����
#define Command_Function_4bits_exten  0x24   //4λ���ݣ�����ָ�����
#define Command_Function_8bits_basic  0x30   //8λ���ݣ�����ָ�����				
#define Command_Function_8bits_exten  0x34   //8λ���ݣ�����ָ�����

//LCD�α����ʾ��λ����
#define Command_CursorShiftLeft  0x10   //�������
#define Command_CursorShiftRight 0x14	//�������
#define Command_AllshiftLeft	 0x18	//��ʾ��������
#define Command_AllshiftRight	 0x1c	//��ʾ��������


/**
  * @brief  LCD12864_Delay    ������ʱ
  * @param  void
  * @retval void
  */
void LCD12864_Delay(unsigned int delay)                 
{
     unsigned int i;
     for(;delay>0;delay--)
        for(i=0;i<10;i++);
}

/**
  * @brief  LCD12864_WriteByte    ��12864дһ�ֽ�
  * @param  void
  * @retval void
  */
void LCD12864_WriteByte(unsigned char Byte)
{
     unsigned char i;
	 LCD12864_CLK_L;  
     for(i=0;i<8;i++)
     {
          if(Byte&0x80)  LCD12864_SID_H;
          else           LCD12864_SID_L; 
		  LCD12864_CLK_H;
		  LCD12864_Delay(40);
		  LCD12864_CLK_L;
          LCD12864_Delay(20);  
          Byte<<=1;      //����
     }  
}

/**
  * @brief  LCD12864_WriteCmd    ��LCDд��ָ��
  * @param  void
  * @retval void
  */
void LCD12864_WriteCmd(unsigned char Command)  
{
      unsigned char Hdata,Ldata;
      Hdata=Command&0xf0;		   //ȡ����λ
      Ldata=(Command<<4)&0xf0;     //ȡ����λ
      LCD12864_WriteByte(0xf8);    //дָ���ʼ���� 1111 1000
      LCD12864_Delay(50);          //��ʱ�Ǳ����
      LCD12864_WriteByte(Hdata);   //���͸���λ
      LCD12864_Delay(50);          //��ʱ�Ǳ����
      LCD12864_WriteByte(Ldata);   //���͵���λ
      LCD12864_Delay(50);          //��ʱ�Ǳ����
}

/**
  * @brief  LCD12864_WriteData    ��LCDд����Ҫ��ʾ������
  * @param  void
  * @retval void
  */
void LCD12864_WriteData(unsigned char Data)
{
      unsigned char Hdata,Ldata;
      Hdata=Data&0xf0;		       //ȡ����λ
      Ldata=(Data<<4)&0xf0;        //ȡ����λ
      LCD12864_WriteByte(0xfa);   //д���ݣ���ʼ���� 1111 1010
      LCD12864_Delay(40);         //��ʱ�Ǳ����
      LCD12864_WriteByte(Hdata);  //���͸���λ
      LCD12864_Delay(40);         //��ʱ�Ǳ����
      LCD12864_WriteByte(Ldata);  //���͵���λ
      LCD12864_Delay(40);         //��ʱ�Ǳ����
}

/**
  * @brief  LCD12864_SetPosition    ָ��LCD����ʾλ�� x:0~3 y:0~7
  * @param  void
  * @retval void
  */
void LCD12864_SetPosition(unsigned char x,unsigned char y)  
{
    unsigned char address=0;
      if(x==0x00)  address = 0x80 + (y&0x07);
      if(x==0x01)  address = 0x90 + (y&0x07);	  
      if(x==0x02)  address = 0x88 + (y&0x07);
      if(x==0x03)  address = 0x98 + (y&0x07);  
      LCD12864_WriteCmd(0x80|address);
}

/**
  * @brief  LCD12864_DispChar    ��LCD��ָ��λ����ʾһ���ַ�
  * @param  void
  * @retval void
  */
void LCD12864_DispChar(unsigned char x,unsigned char y,unsigned char Char)
{
      LCD12864_SetPosition(x,y);
      LCD12864_WriteData(Char);
}

/**
  * @brief  LCD12864_DispString    ��LCD��ָ��λ����ʾһ���ַ���
  * @param  void
  * @retval void
  */
void LCD12864_DispString(unsigned char x,unsigned char y,unsigned char *str)
{
      LCD12864_SetPosition(x,y);
      while(*str!='\0')
	  LCD12864_WriteData(*str++);
}

/**
  * @brief  LCD12864_Clear    12864Һ��������
  * @param  void
  * @retval void
  */
void LCD12864_Clear(void)
{
	LCD12864_WriteCmd(Command_ClearScreen);   // LCD����
	LCD12864_WriteCmd(Command_CursorReturn);  // LCD����λ	
}

/**
  * @brief  LCD12864_Init    ��ʼ��12864Һ����
  * @param  void
  * @retval void
  */
void LCD12864_Init(void)
{
	unsigned long x=1800000;
    GPIO_InitTypeDef gpio;
   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);   
    
	while(x--);
	LCD12864_WriteCmd(Command_ClearScreen);         // LCD����
	LCD12864_WriteCmd(Command_CursorReturn);        // LCD����λ
	LCD12864_WriteCmd(Command_EntryMode_ACup);      // ���������,AC���ӣ����治��λ
	LCD12864_WriteCmd(Command_DispControl_DispOn);  // ��ʾ�����α�أ��α�λ�÷��׹�
	LCD12864_WriteCmd(Command_Function_8bits_basic);// 8λ���ݣ�����ָ�����
}

/**
  * @brief  LCD12864_Printf    12864��ʽ�����
  * @param  void
  * @retval void
  */
void LCD12864_Printf(unsigned char x,unsigned char y,const char *fmt,...)
{
	static unsigned char last_len[4]={0,0,0,0};	
	static unsigned char LCD_BUF[128]={0}; 
	unsigned char len;
	unsigned char i;
	__va_list ap;
	
	va_start(ap,fmt);
	vsprintf((char *)LCD_BUF,fmt,ap);
	va_end(ap);
	
	len=strlen((char *)LCD_BUF)+2*y;
	for(i=len;i<last_len[x];i++)
	{
		LCD_BUF[i-2*y]=' ';
	}
	LCD_BUF[i-2*y]=0;
	LCD12864_DispString(x,y,LCD_BUF);
	last_len[x]=len;
}
