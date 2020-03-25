#ifndef __12864_H__
#define __12864_H__

void LCD12864_Clear(void);
void LCD12864_Init(void);
void LCD12864_Printf(unsigned char x,unsigned char y,const char *fmt,...);

#endif 
