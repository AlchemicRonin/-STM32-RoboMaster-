#include "main.h"

#define LCD12864_CLK_H   GPIO_SetBits(GPIOC,GPIO_Pin_10)
#define LCD12864_CLK_L   GPIO_ResetBits(GPIOC,GPIO_Pin_10)

#define LCD12864_SID_H   GPIO_SetBits(GPIOC,GPIO_Pin_11)
#define LCD12864_SID_L   GPIO_ResetBits(GPIOC,GPIO_Pin_11)


#define Command_ClearScreen           0x01   //LCD清屏
#define Command_CursorReturn          0x02   //LCD光标归位
#define Command_EntryMode_ACdowm      0x04   //DDRAM 地址计数器（AC）减 1,画面不移位
#define Command_EntryMode_ACup 	      0x06   //DDRAM 地址计数器（AC）加 1,画面不移位
#define Command_EntryMode_FrameRight  0x05   //画面整体右移位
#define Command_EntryMode_FrameLeft   0x07   //画面整体左移位
#define Command_DispControl_DispOff   0x08   //显示关,游标关,游标位置反白关
#define Command_DispControl_DispOn    0x0c   //显示开,游标关,游标位置反白关
#define Command_DispControl_ReverseOn 0x0d   //显示开,游标关,游标位置反白开
#define Command_DispControl_CursorOn  0x0e   //显示开,游标开,游标位置反白关
#define Command_DispControl_AllOn     0x0f   //显示开,游标开,游标位置反白开
#define Command_Function_4bits_basic  0x20   //4位数据，基本指令操作
#define Command_Function_4bits_exten  0x24   //4位数据，扩充指令操作
#define Command_Function_8bits_basic  0x30   //8位数据，基本指令操作				
#define Command_Function_8bits_exten  0x34   //8位数据，扩充指令操作

//LCD游标或显示移位控制
#define Command_CursorShiftLeft  0x10   //光标左移
#define Command_CursorShiftRight 0x14	//光标右移
#define Command_AllshiftLeft	 0x18	//显示整体左移
#define Command_AllshiftRight	 0x1c	//显示整体右移


/**
  * @brief  LCD12864_Delay    粗略延时
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
  * @brief  LCD12864_WriteByte    向12864写一字节
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
          Byte<<=1;      //左移
     }  
}

/**
  * @brief  LCD12864_WriteCmd    向LCD写入指令
  * @param  void
  * @retval void
  */
void LCD12864_WriteCmd(unsigned char Command)  
{
      unsigned char Hdata,Ldata;
      Hdata=Command&0xf0;		   //取高四位
      Ldata=(Command<<4)&0xf0;     //取低四位
      LCD12864_WriteByte(0xf8);    //写指令，起始数据 1111 1000
      LCD12864_Delay(50);          //延时是必须的
      LCD12864_WriteByte(Hdata);   //发送高四位
      LCD12864_Delay(50);          //延时是必须的
      LCD12864_WriteByte(Ldata);   //发送低四位
      LCD12864_Delay(50);          //延时是必须的
}

/**
  * @brief  LCD12864_WriteData    向LCD写入需要显示的数据
  * @param  void
  * @retval void
  */
void LCD12864_WriteData(unsigned char Data)
{
      unsigned char Hdata,Ldata;
      Hdata=Data&0xf0;		       //取高四位
      Ldata=(Data<<4)&0xf0;        //取低四位
      LCD12864_WriteByte(0xfa);   //写数据，起始数据 1111 1010
      LCD12864_Delay(40);         //延时是必须的
      LCD12864_WriteByte(Hdata);  //发送高四位
      LCD12864_Delay(40);         //延时是必须的
      LCD12864_WriteByte(Ldata);  //发送低四位
      LCD12864_Delay(40);         //延时是必须的
}

/**
  * @brief  LCD12864_SetPosition    指定LCD的显示位置 x:0~3 y:0~7
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
  * @brief  LCD12864_DispChar    在LCD的指定位置显示一个字符
  * @param  void
  * @retval void
  */
void LCD12864_DispChar(unsigned char x,unsigned char y,unsigned char Char)
{
      LCD12864_SetPosition(x,y);
      LCD12864_WriteData(Char);
}

/**
  * @brief  LCD12864_DispString    在LCD的指定位置显示一个字符串
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
  * @brief  LCD12864_Clear    12864液晶屏清屏
  * @param  void
  * @retval void
  */
void LCD12864_Clear(void)
{
	LCD12864_WriteCmd(Command_ClearScreen);   // LCD清屏
	LCD12864_WriteCmd(Command_CursorReturn);  // LCD光标归位	
}

/**
  * @brief  LCD12864_Init    初始化12864液晶屏
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
	LCD12864_WriteCmd(Command_ClearScreen);         // LCD清屏
	LCD12864_WriteCmd(Command_CursorReturn);        // LCD光标归位
	LCD12864_WriteCmd(Command_EntryMode_ACup);      // 进入点设置,AC增加，画面不移位
	LCD12864_WriteCmd(Command_DispControl_DispOn);  // 显示开，游标关，游标位置反白关
	LCD12864_WriteCmd(Command_Function_8bits_basic);// 8位数据，基本指令操作
}

/**
  * @brief  LCD12864_Printf    12864格式化输出
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
