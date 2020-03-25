/******************************************************************************* 
*                      CRC16  functions algorithm  liboray                     * 
*   file : MF_CRC16.h                                                          *
*                                                                              *
*   history :                                                                  * 
*     v1.0 2010-07-07   Motorfeng                                              *
*******************************************************************************/ 
#ifndef  __MF_CRC16_CPP__
#define  __MF_CRC16_CPP__

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef __FALSE
#define __FALSE   (0)
#endif

#ifndef __TRUE
#define __TRUE    (1)
#endif

typedef char               S8;
typedef unsigned char      U8;
typedef short              S16;
typedef unsigned short     U16;
typedef int                S32;
typedef unsigned int       U32;
typedef long long          S64;
typedef unsigned long long U64;
/*
** global function
*/ 
U16 Get_CRC16_Check_Sum(U8 *pchMessage,U32 dwLength,U16 wCRC); 
U32 Verify_CRC16_Check_Sum(U8* pchMessage, U32 dwLength); 
void Append_CRC16_Check_Sum(U8* pchMessage, U32 dwLength); 

/*
**  Descriptions: append CRC8 to the end of data                                
**  Input:        Data to CRC and append,Stream length = Data + checksum                    
**  Output:       True or False (CRC Verify Result)                                                        
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);


/*
** global const variable 
*/ 
extern const U16 wCRC_Table[256];
extern U16 CRC_INIT;  

#endif
/*
********************************************************************************
*                        END
********************************************************************************
*/
