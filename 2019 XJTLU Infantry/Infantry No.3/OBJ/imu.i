#line 1 "..\\HAL\\imu.c"
#line 1 "..\\USER\\main.h"



#line 1 "..\\USER\\stm32f4xx.h"










































  



 



 
    






  


 
  


 



      

 

     
 

     


 

         
 

          

          

         

 
                                      
     


 

         
 

     
 



 




 






 






#line 133 "..\\USER\\stm32f4xx.h"







            
#line 151 "..\\USER\\stm32f4xx.h"



 










 
#line 174 "..\\USER\\stm32f4xx.h"



 



 



 









 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      

#line 293 "..\\USER\\stm32f4xx.h"


  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,      
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FMC_IRQn                    = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  CRYP_IRQn                   = 79,      
  HASH_RNG_IRQn               = 80,      
  FPU_IRQn                    = 81,      
  UART7_IRQn                  = 82,      
  UART8_IRQn                  = 83,      
  SPI4_IRQn                   = 84,      
  SPI5_IRQn                   = 85,      
  SPI6_IRQn                   = 86,      
  SAI1_IRQn                   = 87,      
  DMA2D_IRQn                  = 90       

    
#line 441 "..\\USER\\stm32f4xx.h"

#line 477 "..\\USER\\stm32f4xx.h"
   
#line 523 "..\\USER\\stm32f4xx.h"

#line 600 "..\\USER\\stm32f4xx.h"

#line 670 "..\\USER\\stm32f4xx.h"

#line 733 "..\\USER\\stm32f4xx.h"

#line 811 "..\\USER\\stm32f4xx.h"
} IRQn_Type;



 

#line 1 "..\\CORE\\core_cm4.h"
 







 

























 
























 




 


 

 













#line 110 "..\\CORE\\core_cm4.h"



 
#line 125 "..\\CORE\\core_cm4.h"

#line 186 "..\\CORE\\core_cm4.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 188 "..\\CORE\\core_cm4.h"
#line 1 "..\\CORE\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 










 










 












 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 









 
#line 209 "..\\CORE\\core_cmInstr.h"








 











 









 









 











 











 











 







 










 










 










 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}









 









 









 









 









 









 





#line 913 "..\\CORE\\core_cmInstr.h"

   

#line 189 "..\\CORE\\core_cm4.h"
#line 1 "..\\CORE\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}








 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xff);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}











 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}







 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}




#line 661 "..\\CORE\\core_cmFunc.h"

 

#line 190 "..\\CORE\\core_cm4.h"
#line 1 "..\\CORE\\core_cmSimd.h"
 







 

























 
















 


 



 


 
#line 122 "..\\CORE\\core_cmSimd.h"











#line 689 "..\\CORE\\core_cmSimd.h"

 






#line 191 "..\\CORE\\core_cm4.h"
















 
#line 234 "..\\CORE\\core_cm4.h"

 






 
#line 250 "..\\CORE\\core_cm4.h"

 













 


 





 


 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 




















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 





 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 





























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     
  volatile uint32_t ACTLR;                    
} SCnSCB_Type;

 



 















 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 



























 



 



 



 









   






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   






 


 
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 






























 








 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile uint32_t FPCCR;                    
  volatile uint32_t FPCAR;                    
  volatile uint32_t FPDSCR;                   
  volatile const  uint32_t MVFR0;                    
  volatile const  uint32_t MVFR1;                    
} FPU_Type;

 



























 



 












 
























 












 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 






 

 
#line 1461 "..\\CORE\\core_cm4.h"

#line 1470 "..\\CORE\\core_cm4.h"











 










 

 



 




 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16) | (7UL << 8)));              
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16) |
                (PriorityGroupTmp << 8)                       );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}







 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8));
}







 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}










 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if((int32_t)IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8 - 4)) & (uint32_t)0xFFUL);
  }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8 - 4)) & (uint32_t)0xFFUL);
  }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if((int32_t)IRQn < 0) {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8 - 4)));
  }
  else {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8 - 4)));
  }
}













 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}













 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                            (1UL << 2)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0);                                                           
  while(1) { __nop(); }                                              
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL )) { return (1UL); }     

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 




 

extern volatile int32_t ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0UL) { __nop(); }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t)ch;
  }
  return (ch);
}








 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}








 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 










#line 818 "..\\USER\\stm32f4xx.h"
#line 1 "..\\USER\\system_stm32f4xx.h"

























  



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 819 "..\\USER\\stm32f4xx.h"
#line 820 "..\\USER\\stm32f4xx.h"



   
 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vuint16_t;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;



 



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;           
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;

#line 987 "..\\USER\\stm32f4xx.h"



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;

#line 1062 "..\\USER\\stm32f4xx.h"


 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;
 


 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t ISR;            
  volatile uint32_t IFCR;           
  volatile uint32_t FGMAR;          
  volatile uint32_t FGOR;           
  volatile uint32_t BGMAR;          
  volatile uint32_t BGOR;           
  volatile uint32_t FGPFCCR;        
  volatile uint32_t FGCOLR;         
  volatile uint32_t BGPFCCR;        
  volatile uint32_t BGCOLR;         
  volatile uint32_t FGCMAR;         
  volatile uint32_t BGCMAR;         
  volatile uint32_t OPFCCR;         
  volatile uint32_t OCOLR;          
  volatile uint32_t OMAR;           
  volatile uint32_t OOR;            
  volatile uint32_t NLR;            
  volatile uint32_t LWR;            
  volatile uint32_t AMTCR;          
  uint32_t      RESERVED[236];  
  volatile uint32_t FGCLUT[256];    
  volatile uint32_t BGCLUT[256];    
} DMA2D_TypeDef;

#line 1228 "..\\USER\\stm32f4xx.h"



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;

#line 1392 "..\\USER\\stm32f4xx.h"




 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];     
} FMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
} FMC_Bank2_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR3;       
} FMC_Bank3_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FMC_Bank4_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t SDCR[2];         
  volatile uint32_t SDTR[2];         
  volatile uint32_t SDCMR;        
  volatile uint32_t SDRTR;        
  volatile uint32_t SDSR;         
} FMC_Bank5_6_TypeDef; 




 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint16_t BSRRL;     
  volatile uint16_t BSRRH;     
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
#line 1501 "..\\USER\\stm32f4xx.h"
  uint32_t      RESERVED[2];    
  volatile uint32_t CMPCR;         




} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t OAR1;        
  uint16_t      RESERVED2;   
  volatile uint16_t OAR2;        
  uint16_t      RESERVED3;   
  volatile uint16_t DR;          
  uint16_t      RESERVED4;   
  volatile uint16_t SR1;         
  uint16_t      RESERVED5;   
  volatile uint16_t SR2;         
  uint16_t      RESERVED6;   
  volatile uint16_t CCR;         
  uint16_t      RESERVED7;   
  volatile uint16_t TRISE;       
  uint16_t      RESERVED8;   
  volatile uint16_t FLTR;        
  uint16_t      RESERVED9;   
} I2C_TypeDef;

#line 1557 "..\\USER\\stm32f4xx.h"



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;



 
  
typedef struct
{
  uint32_t      RESERVED0[2];   
  volatile uint32_t SSCR;           
  volatile uint32_t BPCR;           
  volatile uint32_t AWCR;           
  volatile uint32_t TWCR;           
  volatile uint32_t GCR;            
  uint32_t      RESERVED1[2];   
  volatile uint32_t SRCR;           
  uint32_t      RESERVED2[1];   
  volatile uint32_t BCCR;           
  uint32_t      RESERVED3[1];   
  volatile uint32_t IER;            
  volatile uint32_t ISR;            
  volatile uint32_t ICR;            
  volatile uint32_t LIPCR;          
  volatile uint32_t CPSR;           
  volatile uint32_t CDSR;          
} LTDC_TypeDef;  



 
  
typedef struct
{  
  volatile uint32_t CR;             
  volatile uint32_t WHPCR;          
  volatile uint32_t WVPCR;          
  volatile uint32_t CKCR;           
  volatile uint32_t PFCR;           
  volatile uint32_t CACR;           
  volatile uint32_t DCCR;           
  volatile uint32_t BFCR;           
  uint32_t      RESERVED0[2];   
  volatile uint32_t CFBAR;          
  volatile uint32_t CFBLR;          
  volatile uint32_t CFBLNR;         
  uint32_t      RESERVED1[3];   
  volatile uint32_t CLUTWR;          

} LTDC_Layer_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
  volatile uint32_t PLLSAICFGR;     
  volatile uint32_t DCKCFGR;        
  volatile uint32_t CKGATENR;         
  volatile uint32_t DCKCFGR2;         

} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;




 
  
typedef struct
{
  volatile uint32_t GCR;       
} SAI_TypeDef;

typedef struct
{
  volatile uint32_t CR1;       
  volatile uint32_t CR2;       
  volatile uint32_t FRCR;      
  volatile uint32_t SLOTR;     
  volatile uint32_t IMR;       
  volatile uint32_t SR;        
  volatile uint32_t CLRFR;     
  volatile uint32_t DR;        
} SAI_Block_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t SR;          
  uint16_t      RESERVED2;   
  volatile uint16_t DR;          
  uint16_t      RESERVED3;   
  volatile uint16_t CRCPR;       
  uint16_t      RESERVED4;   
  volatile uint16_t RXCRCR;      
  uint16_t      RESERVED5;   
  volatile uint16_t TXCRCR;      
  uint16_t      RESERVED6;   
  volatile uint16_t I2SCFGR;     
  uint16_t      RESERVED7;   
  volatile uint16_t I2SPR;       
  uint16_t      RESERVED8;   
} SPI_TypeDef;

#line 1813 "..\\USER\\stm32f4xx.h"

#line 1835 "..\\USER\\stm32f4xx.h"

#line 1854 "..\\USER\\stm32f4xx.h"



 

typedef struct
{
  volatile uint16_t CR1;          
  uint16_t      RESERVED0;    
  volatile uint16_t CR2;          
  uint16_t      RESERVED1;    
  volatile uint16_t SMCR;         
  uint16_t      RESERVED2;    
  volatile uint16_t DIER;         
  uint16_t      RESERVED3;    
  volatile uint16_t SR;           
  uint16_t      RESERVED4;    
  volatile uint16_t EGR;          
  uint16_t      RESERVED5;    
  volatile uint16_t CCMR1;        
  uint16_t      RESERVED6;    
  volatile uint16_t CCMR2;        
  uint16_t      RESERVED7;    
  volatile uint16_t CCER;         
  uint16_t      RESERVED8;    
  volatile uint32_t CNT;          
  volatile uint16_t PSC;          
  uint16_t      RESERVED9;    
  volatile uint32_t ARR;          
  volatile uint16_t RCR;          
  uint16_t      RESERVED10;   
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint16_t BDTR;         
  uint16_t      RESERVED11;   
  volatile uint16_t DCR;          
  uint16_t      RESERVED12;   
  volatile uint16_t DMAR;         
  uint16_t      RESERVED13;   
  volatile uint16_t OR;           
  uint16_t      RESERVED14;   
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;          
  uint16_t      RESERVED0;   
  volatile uint16_t DR;          
  uint16_t      RESERVED1;   
  volatile uint16_t BRR;         
  uint16_t      RESERVED2;   
  volatile uint16_t CR1;         
  uint16_t      RESERVED3;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED4;   
  volatile uint16_t CR3;         
  uint16_t      RESERVED5;   
  volatile uint16_t GTPR;        
  uint16_t      RESERVED6;   
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;          
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t DOUT;        
  volatile uint32_t DMACR;       
  volatile uint32_t IMSCR;       
  volatile uint32_t RISR;        
  volatile uint32_t MISR;        
  volatile uint32_t K0LR;        
  volatile uint32_t K0RR;        
  volatile uint32_t K1LR;        
  volatile uint32_t K1RR;        
  volatile uint32_t K2LR;        
  volatile uint32_t K2RR;        
  volatile uint32_t K3LR;        
  volatile uint32_t K3RR;        
  volatile uint32_t IV0LR;       
  volatile uint32_t IV0RR;       
  volatile uint32_t IV1LR;       
  volatile uint32_t IV1RR;       
  volatile uint32_t CSGCMCCM0R;  
  volatile uint32_t CSGCMCCM1R;  
  volatile uint32_t CSGCMCCM2R;  
  volatile uint32_t CSGCMCCM3R;  
  volatile uint32_t CSGCMCCM4R;  
  volatile uint32_t CSGCMCCM5R;  
  volatile uint32_t CSGCMCCM6R;  
  volatile uint32_t CSGCMCCM7R;  
  volatile uint32_t CSGCM0R;     
  volatile uint32_t CSGCM1R;     
  volatile uint32_t CSGCM2R;     
  volatile uint32_t CSGCM3R;     
  volatile uint32_t CSGCM4R;     
  volatile uint32_t CSGCM5R;     
  volatile uint32_t CSGCM6R;     
  volatile uint32_t CSGCM7R;     
} CRYP_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;                
  volatile uint32_t DIN;               
  volatile uint32_t STR;               
  volatile uint32_t HR[5];             
  volatile uint32_t IMR;               
  volatile uint32_t SR;                
       uint32_t RESERVED[52];      
  volatile uint32_t CSR[54];           
} HASH_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t HR[8];       
} HASH_DIGEST_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;

#line 2029 "..\\USER\\stm32f4xx.h"


 
  


 

#line 2052 "..\\USER\\stm32f4xx.h"
      












#line 2079 "..\\USER\\stm32f4xx.h"

 




 





 
#line 2136 "..\\USER\\stm32f4xx.h"

 
#line 2205 "..\\USER\\stm32f4xx.h"

 
#line 2245 "..\\USER\\stm32f4xx.h"

 






#line 2261 "..\\USER\\stm32f4xx.h"


 
#line 2271 "..\\USER\\stm32f4xx.h"

 




 
  


 
#line 2424 "..\\USER\\stm32f4xx.h"

#line 2432 "..\\USER\\stm32f4xx.h"

#line 2441 "..\\USER\\stm32f4xx.h"





 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 2472 "..\\USER\\stm32f4xx.h"

 
#line 2498 "..\\USER\\stm32f4xx.h"
  
 
#line 2524 "..\\USER\\stm32f4xx.h"

 
#line 2562 "..\\USER\\stm32f4xx.h"

 
#line 2604 "..\\USER\\stm32f4xx.h"

 


 


 


 


 


 


 
#line 2653 "..\\USER\\stm32f4xx.h"

 
#line 2691 "..\\USER\\stm32f4xx.h"

 
#line 2729 "..\\USER\\stm32f4xx.h"

 
#line 2758 "..\\USER\\stm32f4xx.h"

 


 


 


 


 



 
#line 2794 "..\\USER\\stm32f4xx.h"

 




 
#line 2821 "..\\USER\\stm32f4xx.h"

 



 
 
 
 
 
 
 
#line 2842 "..\\USER\\stm32f4xx.h"

 
#line 2853 "..\\USER\\stm32f4xx.h"

 
#line 2871 "..\\USER\\stm32f4xx.h"











 





 





 
#line 2909 "..\\USER\\stm32f4xx.h"

 












 
#line 2930 "..\\USER\\stm32f4xx.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 3070 "..\\USER\\stm32f4xx.h"

 
#line 3087 "..\\USER\\stm32f4xx.h"

 
#line 3104 "..\\USER\\stm32f4xx.h"

 
#line 3121 "..\\USER\\stm32f4xx.h"

 
#line 3155 "..\\USER\\stm32f4xx.h"

 
#line 3189 "..\\USER\\stm32f4xx.h"

 
#line 3223 "..\\USER\\stm32f4xx.h"

 
#line 3257 "..\\USER\\stm32f4xx.h"

 
#line 3291 "..\\USER\\stm32f4xx.h"

 
#line 3325 "..\\USER\\stm32f4xx.h"

 
#line 3359 "..\\USER\\stm32f4xx.h"

 
#line 3393 "..\\USER\\stm32f4xx.h"

 
#line 3427 "..\\USER\\stm32f4xx.h"

 
#line 3461 "..\\USER\\stm32f4xx.h"

 
#line 3495 "..\\USER\\stm32f4xx.h"

 
#line 3529 "..\\USER\\stm32f4xx.h"

 
#line 3563 "..\\USER\\stm32f4xx.h"

 
#line 3597 "..\\USER\\stm32f4xx.h"

 
#line 3631 "..\\USER\\stm32f4xx.h"

 
#line 3665 "..\\USER\\stm32f4xx.h"

 
#line 3699 "..\\USER\\stm32f4xx.h"

 
#line 3733 "..\\USER\\stm32f4xx.h"

 
#line 3767 "..\\USER\\stm32f4xx.h"

 
#line 3801 "..\\USER\\stm32f4xx.h"

 
#line 3835 "..\\USER\\stm32f4xx.h"

 
#line 3869 "..\\USER\\stm32f4xx.h"

 
#line 3903 "..\\USER\\stm32f4xx.h"

 
#line 3937 "..\\USER\\stm32f4xx.h"

 
#line 3971 "..\\USER\\stm32f4xx.h"

 
#line 4005 "..\\USER\\stm32f4xx.h"

 
#line 4039 "..\\USER\\stm32f4xx.h"

 
#line 4073 "..\\USER\\stm32f4xx.h"

#line 4133 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 



 



 


 
 
 
 
 
 


#line 4170 "..\\USER\\stm32f4xx.h"

#line 4179 "..\\USER\\stm32f4xx.h"






 





 


 


 


 



 
 
 
 
 
 











































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 
 
 
 
 
 
#line 4323 "..\\USER\\stm32f4xx.h"

 




 





 
#line 4342 "..\\USER\\stm32f4xx.h"

 






 


 






 






 






 


 





 





 



 



 





 
 
 
 
 

 

 
#line 4431 "..\\USER\\stm32f4xx.h"

 



 
#line 4443 "..\\USER\\stm32f4xx.h"

 


 



 

 
#line 4473 "..\\USER\\stm32f4xx.h"

 
#line 4484 "..\\USER\\stm32f4xx.h"

 
#line 4495 "..\\USER\\stm32f4xx.h"

 





 


 
#line 4512 "..\\USER\\stm32f4xx.h"

 



 




 



 



 



 



 



 



 


 
 
 
 
 
  
#line 4590 "..\\USER\\stm32f4xx.h"

 
#line 4609 "..\\USER\\stm32f4xx.h"

  
#line 4620 "..\\USER\\stm32f4xx.h"

  
#line 4642 "..\\USER\\stm32f4xx.h"

  
#line 4664 "..\\USER\\stm32f4xx.h"

  
#line 4686 "..\\USER\\stm32f4xx.h"

  
#line 4708 "..\\USER\\stm32f4xx.h"

 
 
 
 
 

 

#line 4727 "..\\USER\\stm32f4xx.h"

 

#line 4736 "..\\USER\\stm32f4xx.h"

 

#line 4745 "..\\USER\\stm32f4xx.h"

 
#line 4753 "..\\USER\\stm32f4xx.h"

 



 



 



 



 

#line 4784 "..\\USER\\stm32f4xx.h"

 





 

#line 4805 "..\\USER\\stm32f4xx.h"

 





 



 



 






 

 






 




 





 





 



 



 




 



 






 
                                                                     
 


 
 
 
 
 
 
#line 4909 "..\\USER\\stm32f4xx.h"

 
#line 4932 "..\\USER\\stm32f4xx.h"

 
#line 4955 "..\\USER\\stm32f4xx.h"

 
#line 4978 "..\\USER\\stm32f4xx.h"

 
#line 5001 "..\\USER\\stm32f4xx.h"

 
#line 5024 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 
#line 5048 "..\\USER\\stm32f4xx.h"

#line 5056 "..\\USER\\stm32f4xx.h"

 
#line 5065 "..\\USER\\stm32f4xx.h"

 
#line 5084 "..\\USER\\stm32f4xx.h"

 
#line 5092 "..\\USER\\stm32f4xx.h"

#line 5118 "..\\USER\\stm32f4xx.h"



                                             
 
#line 5136 "..\\USER\\stm32f4xx.h"

#line 5919 "..\\USER\\stm32f4xx.h"


 
 
 
 
 
 











#line 5949 "..\\USER\\stm32f4xx.h"

 











#line 5972 "..\\USER\\stm32f4xx.h"

 











#line 5995 "..\\USER\\stm32f4xx.h"

 











#line 6018 "..\\USER\\stm32f4xx.h"

 












#line 6041 "..\\USER\\stm32f4xx.h"























 












#line 6086 "..\\USER\\stm32f4xx.h"























 












#line 6131 "..\\USER\\stm32f4xx.h"























 












#line 6176 "..\\USER\\stm32f4xx.h"























 












#line 6221 "..\\USER\\stm32f4xx.h"











 












#line 6254 "..\\USER\\stm32f4xx.h"











 












#line 6287 "..\\USER\\stm32f4xx.h"











 












#line 6320 "..\\USER\\stm32f4xx.h"











 



























 



























 



























 
#line 6423 "..\\USER\\stm32f4xx.h"

 
#line 6432 "..\\USER\\stm32f4xx.h"

 
#line 6441 "..\\USER\\stm32f4xx.h"

 
#line 6452 "..\\USER\\stm32f4xx.h"

#line 6462 "..\\USER\\stm32f4xx.h"

#line 6472 "..\\USER\\stm32f4xx.h"

#line 6482 "..\\USER\\stm32f4xx.h"

 
#line 6493 "..\\USER\\stm32f4xx.h"

#line 6503 "..\\USER\\stm32f4xx.h"

#line 6513 "..\\USER\\stm32f4xx.h"

#line 6523 "..\\USER\\stm32f4xx.h"

 
#line 6534 "..\\USER\\stm32f4xx.h"

#line 6544 "..\\USER\\stm32f4xx.h"

#line 6554 "..\\USER\\stm32f4xx.h"

#line 6564 "..\\USER\\stm32f4xx.h"

 
#line 6575 "..\\USER\\stm32f4xx.h"

#line 6585 "..\\USER\\stm32f4xx.h"

#line 6595 "..\\USER\\stm32f4xx.h"

#line 6605 "..\\USER\\stm32f4xx.h"

 
#line 6616 "..\\USER\\stm32f4xx.h"

#line 6626 "..\\USER\\stm32f4xx.h"

#line 6636 "..\\USER\\stm32f4xx.h"

#line 6646 "..\\USER\\stm32f4xx.h"

 
#line 6657 "..\\USER\\stm32f4xx.h"

#line 6667 "..\\USER\\stm32f4xx.h"

#line 6677 "..\\USER\\stm32f4xx.h"

#line 6687 "..\\USER\\stm32f4xx.h"

 
#line 6698 "..\\USER\\stm32f4xx.h"

#line 6708 "..\\USER\\stm32f4xx.h"

#line 6718 "..\\USER\\stm32f4xx.h"

#line 6728 "..\\USER\\stm32f4xx.h"

 


 


 






























 






























 





                                            
































 





                                            
































 




                                            












 






 














 
 
 
 
 
 
































































 
#line 7002 "..\\USER\\stm32f4xx.h"

 
































































 
































































 
#line 7150 "..\\USER\\stm32f4xx.h"
 
#line 7167 "..\\USER\\stm32f4xx.h"

 
#line 7185 "..\\USER\\stm32f4xx.h"
 
#line 7202 "..\\USER\\stm32f4xx.h"

 
#line 7236 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 
#line 7260 "..\\USER\\stm32f4xx.h"

 
#line 7269 "..\\USER\\stm32f4xx.h"

 



 





 
 
 
 
 
 
#line 7300 "..\\USER\\stm32f4xx.h"

 
#line 7309 "..\\USER\\stm32f4xx.h"







 



#line 7330 "..\\USER\\stm32f4xx.h"



 



 


 
#line 7355 "..\\USER\\stm32f4xx.h"

 
#line 7365 "..\\USER\\stm32f4xx.h"

 




 


 



#line 7481 "..\\USER\\stm32f4xx.h"
 
 
 
 
 
 


 





 


 



 
 
 
 
 

 




 




 




 




 

#line 7539 "..\\USER\\stm32f4xx.h"

 


 




 





 






 






 






 



 




 






 





 




 




 





 



 



 





                                
 




 



 




 



 






#line 8843 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 











 
#line 8870 "..\\USER\\stm32f4xx.h"
























 


 
#line 8909 "..\\USER\\stm32f4xx.h"

 


#line 9048 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 



#line 9064 "..\\USER\\stm32f4xx.h"

#line 9074 "..\\USER\\stm32f4xx.h"

#line 9085 "..\\USER\\stm32f4xx.h"

 
#line 9094 "..\\USER\\stm32f4xx.h"

#line 9105 "..\\USER\\stm32f4xx.h"















#line 9126 "..\\USER\\stm32f4xx.h"

 
 




#line 9139 "..\\USER\\stm32f4xx.h"

 




#line 9151 "..\\USER\\stm32f4xx.h"

 






#line 9168 "..\\USER\\stm32f4xx.h"

#line 9175 "..\\USER\\stm32f4xx.h"
 











 











 
#line 9206 "..\\USER\\stm32f4xx.h"

 




















 
#line 9252 "..\\USER\\stm32f4xx.h"

 
#line 9271 "..\\USER\\stm32f4xx.h"

 



  




 




#line 9292 "..\\USER\\stm32f4xx.h"

 
#line 9331 "..\\USER\\stm32f4xx.h"

 
#line 9360 "..\\USER\\stm32f4xx.h"




 



 
#line 9392 "..\\USER\\stm32f4xx.h"

 






 













 
#line 9452 "..\\USER\\stm32f4xx.h"

 
#line 9487 "..\\USER\\stm32f4xx.h"
 
#line 9514 "..\\USER\\stm32f4xx.h"

 






 




#line 9533 "..\\USER\\stm32f4xx.h"

 
#line 9572 "..\\USER\\stm32f4xx.h"

 
#line 9606 "..\\USER\\stm32f4xx.h"

 












 
#line 9631 "..\\USER\\stm32f4xx.h"

 





 
#line 9646 "..\\USER\\stm32f4xx.h"

#line 9657 "..\\USER\\stm32f4xx.h"






















 
#line 9689 "..\\USER\\stm32f4xx.h"

#line 9700 "..\\USER\\stm32f4xx.h"


















 









#line 9744 "..\\USER\\stm32f4xx.h"

#line 9753 "..\\USER\\stm32f4xx.h"

#line 9762 "..\\USER\\stm32f4xx.h"

#line 9769 "..\\USER\\stm32f4xx.h"

#line 9806 "..\\USER\\stm32f4xx.h"







#line 9822 "..\\USER\\stm32f4xx.h"
 
 
 
 
 
 



 






 
 
 
 
 
 
#line 9871 "..\\USER\\stm32f4xx.h"

 
#line 9901 "..\\USER\\stm32f4xx.h"

 
#line 9929 "..\\USER\\stm32f4xx.h"

 
#line 9947 "..\\USER\\stm32f4xx.h"

 



 


 



 
#line 10000 "..\\USER\\stm32f4xx.h"

 
#line 10042 "..\\USER\\stm32f4xx.h"

 


 


 



 
#line 10081 "..\\USER\\stm32f4xx.h"

 
#line 10101 "..\\USER\\stm32f4xx.h"

 


 
#line 10119 "..\\USER\\stm32f4xx.h"

 
#line 10141 "..\\USER\\stm32f4xx.h"

 
#line 10149 "..\\USER\\stm32f4xx.h"

 
#line 10157 "..\\USER\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 
 
 
 
 
 








 
































 









#line 10282 "..\\USER\\stm32f4xx.h"







 
#line 10299 "..\\USER\\stm32f4xx.h"

#line 10308 "..\\USER\\stm32f4xx.h"




 


 
#line 10322 "..\\USER\\stm32f4xx.h"
                                     












 
#line 10343 "..\\USER\\stm32f4xx.h"

 
#line 10352 "..\\USER\\stm32f4xx.h"






 
#line 10366 "..\\USER\\stm32f4xx.h"

 


#line 10447 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 




 












 


 






#line 10488 "..\\USER\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 10558 "..\\USER\\stm32f4xx.h"

 
#line 10573 "..\\USER\\stm32f4xx.h"

 
#line 10599 "..\\USER\\stm32f4xx.h"

 


 


 
 
 
 
 
 









#line 10631 "..\\USER\\stm32f4xx.h"

 
#line 10639 "..\\USER\\stm32f4xx.h"

 
#line 10649 "..\\USER\\stm32f4xx.h"

 


 


 


 


 
























 




 
 
 
 
 
   












 






 


 






  
#line 10739 "..\\USER\\stm32f4xx.h"



  
#line 10754 "..\\USER\\stm32f4xx.h"



  
#line 10769 "..\\USER\\stm32f4xx.h"



  
#line 10784 "..\\USER\\stm32f4xx.h"

 






  
#line 10804 "..\\USER\\stm32f4xx.h"



  
#line 10819 "..\\USER\\stm32f4xx.h"



  
#line 10834 "..\\USER\\stm32f4xx.h"



  
#line 10849 "..\\USER\\stm32f4xx.h"

 




           


  
#line 10869 "..\\USER\\stm32f4xx.h"



  
#line 10883 "..\\USER\\stm32f4xx.h"



  
#line 10897 "..\\USER\\stm32f4xx.h"



  
#line 10911 "..\\USER\\stm32f4xx.h"

 






  
#line 10930 "..\\USER\\stm32f4xx.h"



  
#line 10944 "..\\USER\\stm32f4xx.h"



  
#line 10958 "..\\USER\\stm32f4xx.h"



  
#line 10972 "..\\USER\\stm32f4xx.h"












   



#line 11010 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 
















 









#line 11051 "..\\USER\\stm32f4xx.h"

 

























 
#line 11094 "..\\USER\\stm32f4xx.h"

 
#line 11108 "..\\USER\\stm32f4xx.h"

 
#line 11118 "..\\USER\\stm32f4xx.h"

 




























 





















 




























 





















 
#line 11237 "..\\USER\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 
#line 11272 "..\\USER\\stm32f4xx.h"





#line 11283 "..\\USER\\stm32f4xx.h"

 
#line 11291 "..\\USER\\stm32f4xx.h"

#line 11298 "..\\USER\\stm32f4xx.h"

 


 
#line 11309 "..\\USER\\stm32f4xx.h"

#line 11398 "..\\USER\\stm32f4xx.h"

 
 
 
 
 
 
#line 11415 "..\\USER\\stm32f4xx.h"

 


 



 
#line 11439 "..\\USER\\stm32f4xx.h"

 
#line 11448 "..\\USER\\stm32f4xx.h"







 
#line 11468 "..\\USER\\stm32f4xx.h"

 
#line 11479 "..\\USER\\stm32f4xx.h"



 
 
 
 
 
 
#line 11496 "..\\USER\\stm32f4xx.h"
 
#line 11504 "..\\USER\\stm32f4xx.h"



 
#line 11516 "..\\USER\\stm32f4xx.h"
 
#line 11524 "..\\USER\\stm32f4xx.h"




 





 



 
 
 
 
 
 



 









 
#line 11575 "..\\USER\\stm32f4xx.h"
 


 






 
 
 
 
 
 
#line 11619 "..\\USER\\stm32f4xx.h"

 
#line 11635 "..\\USER\\stm32f4xx.h"

 


 


 
#line 11653 "..\\USER\\stm32f4xx.h"
  
 


 
#line 11669 "..\\USER\\stm32f4xx.h"

 



  


 








 

  
#line 11696 "..\\USER\\stm32f4xx.h"

 






 



 


 


 
#line 11725 "..\\USER\\stm32f4xx.h"

 


 
#line 11740 "..\\USER\\stm32f4xx.h"

 


 
#line 11755 "..\\USER\\stm32f4xx.h"

 


 
 
 

 
#line 11770 "..\\USER\\stm32f4xx.h"

 




 




 




 




 


 


 


 


 


 


 
 
 

 
#line 11823 "..\\USER\\stm32f4xx.h"

#line 11830 "..\\USER\\stm32f4xx.h"

 


 


 



 


 



 


 


 


 



 
 
 

 
#line 11905 "..\\USER\\stm32f4xx.h"

 


 


 


 


 




   
#line 11956 "..\\USER\\stm32f4xx.h"

 
#line 11982 "..\\USER\\stm32f4xx.h"

 
#line 11999 "..\\USER\\stm32f4xx.h"

 





 


 


 


 




 

 

  

#line 1 "..\\USER\\stm32f4xx_conf.h"

























 

 



 
 
#line 1 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


























 

 







 
#line 1 "..\\USER\\stm32f4xx.h"










































  



 



 
    
#line 12057 "..\\USER\\stm32f4xx.h"



 

  

 

 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_adc.h"



 



  

 



  
typedef struct
{
  uint32_t ADC_Resolution;                
                                    
  FunctionalState ADC_ScanConvMode;       


  
  FunctionalState ADC_ContinuousConvMode; 

 
  uint32_t ADC_ExternalTrigConvEdge;      


 
  uint32_t ADC_ExternalTrigConv;          


 
  uint32_t ADC_DataAlign;                 

 
  uint8_t  ADC_NbrOfConversion;           


 
}ADC_InitTypeDef;
  


  
typedef struct 
{
  uint32_t ADC_Mode;                      

                                               
  uint32_t ADC_Prescaler;                 

 
  uint32_t ADC_DMAAccessMode;             


 
  uint32_t ADC_TwoSamplingDelay;          

 
  
}ADC_CommonInitTypeDef;


 



  






  
#line 141 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 157 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 173 "..\\FWLIB\\inc\\stm32f4xx_adc.h"
                                     


  




  
#line 214 "..\\FWLIB\\inc\\stm32f4xx_adc.h"
                                     


  




  
#line 231 "..\\FWLIB\\inc\\stm32f4xx_adc.h"
                                      


  




  
#line 248 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 288 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  






  




  
#line 327 "..\\FWLIB\\inc\\stm32f4xx_adc.h"












#line 358 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 382 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 398 "..\\FWLIB\\inc\\stm32f4xx_adc.h"
                                            


  




  
#line 439 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 455 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 477 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 491 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  
#line 505 "..\\FWLIB\\inc\\stm32f4xx_adc.h"
  
#line 513 "..\\FWLIB\\inc\\stm32f4xx_adc.h"


  




  



  




  



  




  



  




  



  




  



  




  



  




  



  




  

 
   

   
void ADC_DeInit(void);

 
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_CommonInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_CommonStructInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);

 
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
void ADC_VBATCmd(FunctionalState NewState);

 
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_EOCOnEachRegularChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetMultiModeConversionValue(void);

 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_MultiModeDMARequestAfterLastTransferCmd(FunctionalState NewState);

 
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConvEdge);
void ADC_SoftwareStartInjectedConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);

 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









  



  

 
#line 35 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_crc.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_crc.h"



 



 

 
 



 



 

 
   

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);









 



 

 
#line 36 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_dbgmcu.h"

























 

 







 
#line 38 "..\\FWLIB\\inc\\stm32f4xx_dbgmcu.h"



 



  

 
 



  





#line 76 "..\\FWLIB\\inc\\stm32f4xx_dbgmcu.h"

#line 83 "..\\FWLIB\\inc\\stm32f4xx_dbgmcu.h"


  

 
  
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);









  



  

 
#line 37 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_dma.h"


























  

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_dma.h"



 



 

 



 

typedef struct
{
  uint32_t DMA_Channel;            
 
 
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_Memory0BaseAddr;    

 

  uint32_t DMA_DIR;                

 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_FIFOMode;          


 

  uint32_t DMA_FIFOThreshold;      
 

  uint32_t DMA_MemoryBurst;        


 

  uint32_t DMA_PeripheralBurst;    


   
}DMA_InitTypeDef;

 



 

#line 134 "..\\FWLIB\\inc\\stm32f4xx_dma.h"






  
#line 149 "..\\FWLIB\\inc\\stm32f4xx_dma.h"

#line 158 "..\\FWLIB\\inc\\stm32f4xx_dma.h"


  




  









  




  



  




  







  




  







  




  









  




  









  




  







  




  











  




  







  




  











  




  











  




  











  




 
#line 346 "..\\FWLIB\\inc\\stm32f4xx_dma.h"

#line 353 "..\\FWLIB\\inc\\stm32f4xx_dma.h"


  



 
#line 400 "..\\FWLIB\\inc\\stm32f4xx_dma.h"




#line 424 "..\\FWLIB\\inc\\stm32f4xx_dma.h"


  




  









  




  
#line 487 "..\\FWLIB\\inc\\stm32f4xx_dma.h"





#line 512 "..\\FWLIB\\inc\\stm32f4xx_dma.h"


  




  







  




  







  




  






  



  

 
  

  
void DMA_DeInit(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_Init(DMA_Stream_TypeDef* DMAy_Streamx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);

 
void DMA_PeriphIncOffsetSizeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_Pincos);
void DMA_FlowControllerConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FlowCtrl);

 
void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter);
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_DoubleBufferModeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t Memory1BaseAddr,
                                uint32_t DMA_CurrentMemory);
void DMA_DoubleBufferModeCmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);
void DMA_MemoryTargetConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t MemoryBaseAddr,
                            uint32_t DMA_MemoryTarget);
uint32_t DMA_GetCurrentMemoryTarget(DMA_Stream_TypeDef* DMAy_Streamx);

 
FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx);
uint32_t DMA_GetFIFOStatus(DMA_Stream_TypeDef* DMAy_Streamx);
FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ClearFlag(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState);
ITStatus DMA_GetITStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);
void DMA_ClearITPendingBit(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);









 



 


 
#line 38 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_exti.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_exti.h"



 



 

 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;

 



 



 

#line 129 "..\\FWLIB\\inc\\stm32f4xx_exti.h"

                                          


#line 145 "..\\FWLIB\\inc\\stm32f4xx_exti.h"
                    


 



 

 
 

 
void EXTI_DeInit(void);

 
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);

 
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);









 



 

 
#line 39 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_flash.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_flash.h"



 



  

 


  
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_RD,
  FLASH_ERROR_PGS,
  FLASH_ERROR_PGP,
  FLASH_ERROR_PGA,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_ERROR_OPERATION,
  FLASH_COMPLETE
}FLASH_Status;

 



   



  
#line 90 "..\\FWLIB\\inc\\stm32f4xx_flash.h"


#line 108 "..\\FWLIB\\inc\\stm32f4xx_flash.h"


  



  











  



 
#line 155 "..\\FWLIB\\inc\\stm32f4xx_flash.h"

#line 168 "..\\FWLIB\\inc\\stm32f4xx_flash.h"
































  



  
#line 230 "..\\FWLIB\\inc\\stm32f4xx_flash.h"




 



 





 



  
#line 274 "..\\FWLIB\\inc\\stm32f4xx_flash.h"




 



 


  
 





  



  





  



  





  




  





 
  


   
#line 334 "..\\FWLIB\\inc\\stm32f4xx_flash.h"


 
  


 





 



  





  



  
#line 374 "..\\FWLIB\\inc\\stm32f4xx_flash.h"


 



 







  



  







  



  



  



  



  



  




  




  

 
  
 
 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
void FLASH_InstructionCacheCmd(FunctionalState NewState);
void FLASH_DataCacheCmd(FunctionalState NewState);
void FLASH_InstructionCacheReset(void);
void FLASH_DataCacheReset(void);

    
void         FLASH_Unlock(void);
void         FLASH_Lock(void);
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllBank1Sectors(uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllBank2Sectors(uint8_t VoltageRange);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data);

  
void         FLASH_OB_Unlock(void);
void         FLASH_OB_Lock(void);
void         FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
void         FLASH_OB_WRP1Config(uint32_t OB_WRP, FunctionalState NewState);
void         FLASH_OB_PCROPSelectionConfig(uint8_t OB_PcROP);
void         FLASH_OB_PCROPConfig(uint32_t OB_PCROP, FunctionalState NewState);
void         FLASH_OB_PCROP1Config(uint32_t OB_PCROP, FunctionalState NewState);
void         FLASH_OB_RDPConfig(uint8_t OB_RDP);
void         FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
void         FLASH_OB_BORConfig(uint8_t OB_BOR);
void         FLASH_OB_BootConfig(uint8_t OB_BOOT);
FLASH_Status FLASH_OB_Launch(void);
uint8_t      FLASH_OB_GetUser(void);
uint16_t     FLASH_OB_GetWRP(void);
uint16_t     FLASH_OB_GetWRP1(void);
uint16_t     FLASH_OB_GetPCROP(void);
uint16_t     FLASH_OB_GetPCROP1(void);
FlagStatus   FLASH_OB_GetRDP(void);
uint8_t      FLASH_OB_GetBOR(void);

 
void         FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus   FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void         FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(void);









  



  

 
#line 40 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



 



  

 

#line 61 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



    
typedef enum
{ 
  GPIO_Mode_IN   = 0x00,  
  GPIO_Mode_OUT  = 0x01,  
  GPIO_Mode_AF   = 0x02,  
  GPIO_Mode_AN   = 0x03   
}GPIOMode_TypeDef;





   
typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;





   
typedef enum
{ 
  GPIO_Low_Speed     = 0x00,  
  GPIO_Medium_Speed  = 0x01,  
  GPIO_Fast_Speed    = 0x02,  
  GPIO_High_Speed    = 0x03   
}GPIOSpeed_TypeDef;

 




  





  
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;





  
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;





  
typedef struct
{
  uint32_t GPIO_Pin;              
 

  GPIOMode_TypeDef GPIO_Mode;     
 

  GPIOSpeed_TypeDef GPIO_Speed;   
 

  GPIOOType_TypeDef GPIO_OType;   
 

  GPIOPuPd_TypeDef GPIO_PuPd;     
 
}GPIO_InitTypeDef;

 



  



  
#line 176 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"

#line 195 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"


  




  
#line 219 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"

#line 236 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"


  



  


  
#line 254 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  







  






  
#line 283 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"


  
#line 295 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  
#line 305 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  
#line 322 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  
#line 334 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  




  
#line 357 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  
#line 378 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  
#line 398 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"


  
#line 409 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



  













  






 







  


#line 464 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"

#line 481 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"









#line 514 "..\\FWLIB\\inc\\stm32f4xx_gpio.h"



















  



 
    








 



 

 
 

 
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

 
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);









  



  

 
#line 41 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"


























  

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"



 



 

 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;

 




 





 




 




 

#line 104 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 178 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"


 



 







  



 







 



 







  



 







  



 

#line 248 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"



#line 258 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"


 



 



 

#line 277 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"



 

#line 296 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"



#line 310 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"


 



 





 








 
 

























 

 


 





























 

  
 


 
 

 







 

























 

    
 



 



 



























 

  
 

 


 
 


 






 

#line 516 "..\\FWLIB\\inc\\stm32f4xx_i2c.h"


 



 




 



 




 



 

 
  

 
void I2C_DeInit(I2C_TypeDef* I2Cx);

 
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DigitalFilterConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DigitalFilter);
void I2C_AnalogFilterCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

  
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

  
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);

 
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

 
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);




















































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);


void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);









  



  

 
#line 42 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_iwdg.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_iwdg.h"



 



 

 
 



 
  


 






 



 
#line 83 "..\\FWLIB\\inc\\stm32f4xx_iwdg.h"


 



 






 



 

 
 

 
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

 
void IWDG_Enable(void);

 
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);









 



 

 
#line 43 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_pwr.h"


























  

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_pwr.h"



 



  

 
 



  



  
#line 66 "..\\FWLIB\\inc\\stm32f4xx_pwr.h"







 

  


 



 








 



 








 
#line 126 "..\\FWLIB\\inc\\stm32f4xx_pwr.h"



 





 



 
#line 146 "..\\FWLIB\\inc\\stm32f4xx_pwr.h"


 



 
#line 161 "..\\FWLIB\\inc\\stm32f4xx_pwr.h"

 













 



 

 
  

  
void PWR_DeInit(void);

  
void PWR_BackupAccessCmd(FunctionalState NewState);

  
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_PVDCmd(FunctionalState NewState);

 

void PWR_WakeUpPinCmd(FunctionalState NewState);




  
void PWR_BackupRegulatorCmd(FunctionalState NewState);
void PWR_MainRegulatorModeConfig(uint32_t PWR_Regulator_Voltage);
void PWR_OverDriveCmd(FunctionalState NewState);
void PWR_OverDriveSWCmd(FunctionalState NewState);
void PWR_UnderDriveCmd(FunctionalState NewState);


void PWR_MainRegulatorUnderDriveCmd(FunctionalState NewState);
void PWR_LowRegulatorUnderDriveCmd(FunctionalState NewState);







  
void PWR_FlashPowerDownCmd(FunctionalState NewState);

  
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterUnderDriveSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);

  
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);









 



 

 
#line 44 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

























 

 







 
#line 38 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"



 



  

 
typedef struct
{
  uint32_t SYSCLK_Frequency;  
  uint32_t HCLK_Frequency;    
  uint32_t PCLK1_Frequency;   
  uint32_t PCLK2_Frequency;   
}RCC_ClocksTypeDef;

 



 
  


 







  



 






 



 
#line 96 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


 



 
#line 114 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 132 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"










  
  


 

#line 160 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 169 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 190 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 205 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 220 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 227 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"



  
  


 







  
  


 
#line 311 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


  

#line 347 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 428 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"




 






  



 









  



 









  




 






 

#line 493 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 526 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 567 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 582 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 624 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"



  
#line 661 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"



  
  


   
#line 677 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


  
  


  
























  
  


  
#line 754 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


  
  


  
#line 795 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

 







  



 
#line 820 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"
                                   





  
  


 
#line 842 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"
                                   





  
  


 
#line 867 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 875 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"




  



  

 
  

 
void        RCC_DeInit(void);

 
void        RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void        RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void        RCC_HSICmd(FunctionalState NewState);
void        RCC_LSEConfig(uint8_t RCC_LSE);
void        RCC_LSICmd(FunctionalState NewState);

void        RCC_PLLCmd(FunctionalState NewState);






void        RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);


void        RCC_PLLI2SCmd(FunctionalState NewState);

#line 918 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"
void        RCC_PLLI2SConfig(uint32_t PLLI2SN, uint32_t PLLI2SQ, uint32_t PLLI2SR);





void        RCC_PLLSAICmd(FunctionalState NewState);
#line 932 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"
void        RCC_PLLSAIConfig(uint32_t PLLSAIN, uint32_t PLLSAIQ, uint32_t PLLSAIR);


void        RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void        RCC_MCO1Config(uint32_t RCC_MCO1Source, uint32_t RCC_MCO1Div);
void        RCC_MCO2Config(uint32_t RCC_MCO2Source, uint32_t RCC_MCO2Div);

 
void        RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t     RCC_GetSYSCLKSource(void);
void        RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void        RCC_PCLK1Config(uint32_t RCC_HCLK);
void        RCC_PCLK2Config(uint32_t RCC_HCLK);
void        RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

 
void        RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void        RCC_RTCCLKCmd(FunctionalState NewState);
void        RCC_BackupResetCmd(FunctionalState NewState);

#line 962 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"


void        RCC_I2SCLKConfig(uint32_t RCC_I2SCLKSource);



void        RCC_SAIBlockACLKConfig(uint32_t RCC_SAIBlockACLKSource);
void        RCC_SAIBlockBCLKConfig(uint32_t RCC_SAIBlockBCLKSource);


void        RCC_SAIPLLI2SClkDivConfig(uint32_t RCC_PLLI2SDivQ);
void        RCC_SAIPLLSAIClkDivConfig(uint32_t RCC_PLLSAIDivQ);






void        RCC_LTDCCLKDivConfig(uint32_t RCC_PLLSAIDivR);
void        RCC_TIMCLKPresConfig(uint32_t RCC_TIMCLKPrescaler);

void        RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void        RCC_AHB2PeriphClockCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void        RCC_AHB3PeriphClockCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void        RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void        RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void        RCC_AHB1PeriphResetCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void        RCC_AHB2PeriphResetCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void        RCC_AHB3PeriphResetCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void        RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void        RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void        RCC_AHB1PeriphClockLPModeCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void        RCC_AHB2PeriphClockLPModeCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void        RCC_AHB3PeriphClockLPModeCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void        RCC_APB1PeriphClockLPModeCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void        RCC_APB2PeriphClockLPModeCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

 
void        RCC_LSEModeConfig(uint8_t RCC_Mode);

 




 





 






 




 
#line 1035 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"

#line 1045 "..\\FWLIB\\inc\\stm32f4xx_rcc.h"
 
void        RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus  RCC_GetFlagStatus(uint8_t RCC_FLAG);
void        RCC_ClearFlag(void);
ITStatus    RCC_GetITStatus(uint8_t RCC_IT);
void        RCC_ClearITPendingBit(uint8_t RCC_IT);









  



  

 
#line 45 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



 



  

 



  
typedef struct
{
  uint32_t RTC_HourFormat;   
 
  
  uint32_t RTC_AsynchPrediv; 
 
  
  uint32_t RTC_SynchPrediv;  
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t RTC_Hours;    


 

  uint8_t RTC_Minutes;  
 
  
  uint8_t RTC_Seconds;  
 

  uint8_t RTC_H12;      
 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint8_t RTC_WeekDay; 
 
  
  uint8_t RTC_Month;   
 

  uint8_t RTC_Date;     
 
  
  uint8_t RTC_Year;     
 
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef RTC_AlarmTime;      

  uint32_t RTC_AlarmMask;            
 

  uint32_t RTC_AlarmDateWeekDaySel;  
 
  
  uint8_t RTC_AlarmDateWeekDay;      



 
}RTC_AlarmTypeDef;

 



  




  






  



  

 


  




  




  



  







  



  






  



  




  



  

 
#line 211 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



  



  
  
#line 234 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


  




  
#line 250 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



  




  








  




  
#line 280 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



  



  







  

  

  
#line 349 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


  



  





  



  
#line 379 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


  



  






  



  




 







  



  






  




  








  

 

  






  



  
#line 459 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"
                                          


  



  
#line 474 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



  



  




 



  











  



  
#line 515 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



  



  


#line 535 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


  



  
#line 566 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"



 

  

  
#line 582 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"







 



  






 



  




 




  



  







  



  







  



  






  



  




 



 

#line 704 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


  



  






  



  
#line 745 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"


  



  
#line 759 "..\\FWLIB\\inc\\stm32f4xx_rtc.h"









  



  





  



  

 
  

 
ErrorStatus RTC_DeInit(void);

 
ErrorStatus RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
void RTC_StructInit(RTC_InitTypeDef* RTC_InitStruct);
void RTC_WriteProtectionCmd(FunctionalState NewState);
ErrorStatus RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);
void RTC_BypassShadowCmd(FunctionalState NewState);

 
ErrorStatus RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_TimeStructInit(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
uint32_t RTC_GetSubSecond(void);
ErrorStatus RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
void RTC_DateStructInit(RTC_DateTypeDef* RTC_DateStruct);
void RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);

 
void RTC_SetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_AlarmStructInit(RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_GetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);
void RTC_AlarmSubSecondConfig(uint32_t RTC_Alarm, uint32_t RTC_AlarmSubSecondValue, uint32_t RTC_AlarmSubSecondMask);
uint32_t RTC_GetAlarmSubSecond(uint32_t RTC_Alarm);

 
void RTC_WakeUpClockConfig(uint32_t RTC_WakeUpClock);
void RTC_SetWakeUpCounter(uint32_t RTC_WakeUpCounter);
uint32_t RTC_GetWakeUpCounter(void);
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState);

 
void RTC_DayLightSavingConfig(uint32_t RTC_DayLightSaving, uint32_t RTC_StoreOperation);
uint32_t RTC_GetStoreOperation(void);

 
void RTC_OutputConfig(uint32_t RTC_Output, uint32_t RTC_OutputPolarity);

 
ErrorStatus RTC_CoarseCalibConfig(uint32_t RTC_CalibSign, uint32_t Value);
ErrorStatus RTC_CoarseCalibCmd(FunctionalState NewState);
void RTC_CalibOutputCmd(FunctionalState NewState);
void RTC_CalibOutputConfig(uint32_t RTC_CalibOutput);
ErrorStatus RTC_SmoothCalibConfig(uint32_t RTC_SmoothCalibPeriod, 
                                  uint32_t RTC_SmoothCalibPlusPulses,
                                  uint32_t RTC_SmouthCalibMinusPulsesValue);

 
void RTC_TimeStampCmd(uint32_t RTC_TimeStampEdge, FunctionalState NewState);
void RTC_GetTimeStamp(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_StampTimeStruct,
                                      RTC_DateTypeDef* RTC_StampDateStruct);
uint32_t RTC_GetTimeStampSubSecond(void);

 
void RTC_TamperTriggerConfig(uint32_t RTC_Tamper, uint32_t RTC_TamperTrigger);
void RTC_TamperCmd(uint32_t RTC_Tamper, FunctionalState NewState);
void RTC_TamperFilterConfig(uint32_t RTC_TamperFilter);
void RTC_TamperSamplingFreqConfig(uint32_t RTC_TamperSamplingFreq);
void RTC_TamperPinsPrechargeDuration(uint32_t RTC_TamperPrechargeDuration);
void RTC_TimeStampOnTamperDetectionCmd(FunctionalState NewState);
void RTC_TamperPullUpCmd(FunctionalState NewState);

 
void RTC_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t RTC_ReadBackupRegister(uint32_t RTC_BKP_DR);


 
void RTC_TamperPinSelection(uint32_t RTC_TamperPin);
void RTC_TimeStampPinSelection(uint32_t RTC_TimeStampPin);
void RTC_OutputTypeConfig(uint32_t RTC_OutputType);

 
ErrorStatus RTC_SynchroShiftConfig(uint32_t RTC_ShiftAdd1S, uint32_t RTC_ShiftSubFS);

 
void RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
FlagStatus RTC_GetFlagStatus(uint32_t RTC_FLAG);
void RTC_ClearFlag(uint32_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint32_t RTC_IT);
void RTC_ClearITPendingBit(uint32_t RTC_IT);









  



  

 
#line 46 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"



 



 

 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;


 



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 225 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"


  



 




 



 

#line 248 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"


 



 








 



 






  



 

#line 286 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"


 



 




 



 

#line 333 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"


 



 







 



 







 



 






 



 

#line 424 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"



#line 451 "..\\FWLIB\\inc\\stm32f4xx_sdio.h"





 



 







 



 

 
 
 
void SDIO_DeInit(void);

 
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);

 
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);

 
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);

 
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);

 
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);

 
void SDIO_DMACmd(FunctionalState NewState);

 
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);









 



 

 
#line 47 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


























  

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_spi.h"



 



  

 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;

 



 

#line 125 "..\\FWLIB\\inc\\stm32f4xx_spi.h"

#line 134 "..\\FWLIB\\inc\\stm32f4xx_spi.h"















 
  
#line 159 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 243 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


  



 







 



 

#line 271 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


 
  



 

#line 290 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


 
  


 

#line 306 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


 



 







 



 

#line 336 "..\\FWLIB\\inc\\stm32f4xx_spi.h"






 
            


 







 



 






 



 







 



 






 



 







 



 























 



 

#line 443 "..\\FWLIB\\inc\\stm32f4xx_spi.h"

#line 450 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


 



 




 



 

#line 486 "..\\FWLIB\\inc\\stm32f4xx_spi.h"


 
  


 

 
  

  
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);

 
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TIModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);

void I2S_FullDuplexConfig(SPI_TypeDef* I2Sxext, I2S_InitTypeDef* I2S_InitStruct);

  
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);

 
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);

 
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);

 
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);









 



 

 
#line 48 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"



 



  

 
 


  
#line 160 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"



  
#line 175 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"

#line 187 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"
                                         


  




  
#line 228 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"


  




  

















#line 259 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"







#line 273 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"

#line 281 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"

#line 289 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"


  




  







  



  

 
  
 
void       SYSCFG_DeInit(void);
void       SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void       SYSCFG_MemorySwappingBank(FunctionalState NewState);
void       SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
void       SYSCFG_ETH_MediaInterfaceConfig(uint32_t SYSCFG_ETH_MediaInterface); 
void       SYSCFG_CompensationCellCmd(FunctionalState NewState); 
FlagStatus SYSCFG_GetCompensationCellStatus(void);
#line 339 "..\\FWLIB\\inc\\stm32f4xx_syscfg.h"





  



  

 
#line 49 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_tim.h"



 



  

 




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint32_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef; 



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint32_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;

 



 

#line 189 "..\\FWLIB\\inc\\stm32f4xx_tim.h"
                                          
#line 202 "..\\FWLIB\\inc\\stm32f4xx_tim.h"
                                     
 
#line 212 "..\\FWLIB\\inc\\stm32f4xx_tim.h"
 
#line 219 "..\\FWLIB\\inc\\stm32f4xx_tim.h"
 


 
#line 231 "..\\FWLIB\\inc\\stm32f4xx_tim.h"
                                






 

#line 260 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


 



 







  



 





                                 




                                 







  



 

#line 309 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


 



 

#line 327 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 451 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 513 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 529 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 545 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 562 "..\\FWLIB\\inc\\stm32f4xx_tim.h"

#line 571 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 619 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 663 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 679 "..\\FWLIB\\inc\\stm32f4xx_tim.h"



  



 

#line 696 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 724 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 







  



  






 



 







  



 







  



 

#line 785 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  




 

#line 803 "..\\FWLIB\\inc\\stm32f4xx_tim.h"
  


  



 

#line 818 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 879 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 

#line 895 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


  



 







  


 














#line 937 "..\\FWLIB\\inc\\stm32f4xx_tim.h"



  


 

#line 969 "..\\FWLIB\\inc\\stm32f4xx_tim.h"



  



 




  



 




  



 

#line 1014 "..\\FWLIB\\inc\\stm32f4xx_tim.h"


 



 

 
  

 
void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);

 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

 
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);

 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);

    
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap);









  



 

 
#line 50 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_usart.h"


























  

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_usart.h"



 



  

  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            



 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;

 



  
  
#line 120 "..\\FWLIB\\inc\\stm32f4xx_usart.h"








  
  


                                    




  



  
  
#line 151 "..\\FWLIB\\inc\\stm32f4xx_usart.h"


  



  
  
#line 165 "..\\FWLIB\\inc\\stm32f4xx_usart.h"


  



  
  





  



  
#line 192 "..\\FWLIB\\inc\\stm32f4xx_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 259 "..\\FWLIB\\inc\\stm32f4xx_usart.h"



 



 

#line 280 "..\\FWLIB\\inc\\stm32f4xx_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 352 "..\\FWLIB\\inc\\stm32f4xx_usart.h"
                              








  



  

 
   

  
void USART_DeInit(USART_TypeDef* USARTx);

 
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);

  
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendBreak(USART_TypeDef* USARTx);

 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);

 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);

 
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);









  



  

 
#line 51 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_wwdg.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_wwdg.h"



 



  

 
 



  
  


 
  
#line 69 "..\\FWLIB\\inc\\stm32f4xx_wwdg.h"



  



  

 
 
  
   
void WWDG_DeInit(void);

 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);

 
void WWDG_Enable(uint8_t Counter);

 
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  

 
#line 52 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\misc.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\misc.h"



 



 

 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  


 

  uint8_t NVIC_IRQChannelSubPriority;         


 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 
 



 



 







 



 

#line 104 "..\\FWLIB\\inc\\misc.h"


 



 

#line 122 "..\\FWLIB\\inc\\misc.h"















 



 







 



 

 
 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 

 
#line 53 "..\\USER\\stm32f4xx_conf.h"

#line 66 "..\\USER\\stm32f4xx_conf.h"

#line 1 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"



 



  

 



  
typedef struct
{
  uint32_t CRYP_AlgoDir;   
 
  uint32_t CRYP_AlgoMode;  

 
  uint32_t CRYP_DataType;  
  
  uint32_t CRYP_KeySize;   

 
}CRYP_InitTypeDef;



  
typedef struct
{
  uint32_t CRYP_Key0Left;   
  uint32_t CRYP_Key0Right;  
  uint32_t CRYP_Key1Left;   
  uint32_t CRYP_Key1Right;  
  uint32_t CRYP_Key2Left;   
  uint32_t CRYP_Key2Right;  
  uint32_t CRYP_Key3Left;   
  uint32_t CRYP_Key3Right;  
}CRYP_KeyInitTypeDef;


  
typedef struct
{
  uint32_t CRYP_IV0Left;   
  uint32_t CRYP_IV0Right;  
  uint32_t CRYP_IV1Left;   
  uint32_t CRYP_IV1Right;  
}CRYP_IVInitTypeDef;



  
typedef struct
{
   
  uint32_t CR_CurrentConfig;
   
  uint32_t CRYP_IV0LR;
  uint32_t CRYP_IV0RR;
  uint32_t CRYP_IV1LR;
  uint32_t CRYP_IV1RR;
   
  uint32_t CRYP_K0LR;
  uint32_t CRYP_K0RR;
  uint32_t CRYP_K1LR;
  uint32_t CRYP_K1RR;
  uint32_t CRYP_K2LR;
  uint32_t CRYP_K2RR;
  uint32_t CRYP_K3LR;
  uint32_t CRYP_K3RR;
  uint32_t CRYP_CSGCMCCMR[8];
  uint32_t CRYP_CSGCMR[8];
}CRYP_Context;


 



 



 







  
 


 

 



 



 
#line 155 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"

#line 166 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"


  



 

 












  



 
#line 200 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"


 
                                     


 
#line 213 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"


 



 
#line 232 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"

#line 240 "..\\FWLIB\\inc\\stm32f4xx_cryp.h"


 



 







 



 





 



 





  



  

 
 

 
void CRYP_DeInit(void);

 
void CRYP_Init(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_StructInit(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_KeyInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_KeyStructInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_IVInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_IVStructInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_Cmd(FunctionalState NewState);
void CRYP_PhaseConfig(uint32_t CRYP_Phase);
void CRYP_FIFOFlush(void);
 
void CRYP_DataIn(uint32_t Data);
uint32_t CRYP_DataOut(void);

 
ErrorStatus CRYP_SaveContext(CRYP_Context* CRYP_ContextSave,
                             CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_RestoreContext(CRYP_Context* CRYP_ContextRestore);

 
void CRYP_DMACmd(uint8_t CRYP_DMAReq, FunctionalState NewState);

 
void CRYP_ITConfig(uint8_t CRYP_IT, FunctionalState NewState);
ITStatus CRYP_GetITStatus(uint8_t CRYP_IT);
FunctionalState CRYP_GetCmdStatus(void);
FlagStatus CRYP_GetFlagStatus(uint8_t CRYP_FLAG);

 
ErrorStatus CRYP_AES_ECB(uint8_t Mode,
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CBC(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CTR(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_GCM(uint8_t Mode, uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t ILength,
                         uint8_t *Header, uint32_t HLength,
                         uint8_t *Output, uint8_t *AuthTAG);

ErrorStatus CRYP_AES_CCM(uint8_t Mode, 
                         uint8_t* Nonce, uint32_t NonceSize,
                         uint8_t* Key, uint16_t Keysize,
                         uint8_t* Input, uint32_t ILength,
                         uint8_t* Header, uint32_t HLength, uint8_t *HBuffer,
                         uint8_t* Output,
                         uint8_t* AuthTAG, uint32_t TAGSize);

 
ErrorStatus CRYP_TDES_ECB(uint8_t Mode,
                           uint8_t Key[24], 
                           uint8_t *Input, uint32_t Ilength,
                           uint8_t *Output);

ErrorStatus CRYP_TDES_CBC(uint8_t Mode,
                          uint8_t Key[24],
                          uint8_t InitVectors[8],
                          uint8_t *Input, uint32_t Ilength,
                          uint8_t *Output);

 
ErrorStatus CRYP_DES_ECB(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_DES_CBC(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t InitVectors[8],
                         uint8_t *Input,uint32_t Ilength,
                         uint8_t *Output);









 



  

 
#line 69 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_hash.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_hash.h"



 



  

 



  
typedef struct
{
  uint32_t HASH_AlgoSelection; 
 
  uint32_t HASH_AlgoMode;      
 
  uint32_t HASH_DataType;      

 
  uint32_t HASH_HMACKeyType;   
 
}HASH_InitTypeDef;



  
typedef struct
{
  uint32_t Data[8];      


 
} HASH_MsgDigest; 



  
typedef struct
{
  uint32_t HASH_IMR; 
  uint32_t HASH_STR;      
  uint32_t HASH_CR;     
  uint32_t HASH_CSR[54];       
}HASH_Context;

 



  



  











 



  







 



   











 



  







 



   




 



   





				   


 



   

















  



  

 
  
  
 
void HASH_DeInit(void);

 
void HASH_Init(HASH_InitTypeDef* HASH_InitStruct);
void HASH_StructInit(HASH_InitTypeDef* HASH_InitStruct);
void HASH_Reset(void);

 
void HASH_DataIn(uint32_t Data);
uint8_t HASH_GetInFIFOWordsNbr(void);
void HASH_SetLastWordValidBitsNbr(uint16_t ValidNumber);
void HASH_StartDigest(void);
void HASH_AutoStartDigest(FunctionalState NewState);
void HASH_GetDigest(HASH_MsgDigest* HASH_MessageDigest);

 
void HASH_SaveContext(HASH_Context* HASH_ContextSave);
void HASH_RestoreContext(HASH_Context* HASH_ContextRestore);

 
void HASH_DMACmd(FunctionalState NewState);

 
void HASH_ITConfig(uint32_t HASH_IT, FunctionalState NewState);
FlagStatus HASH_GetFlagStatus(uint32_t HASH_FLAG);
void HASH_ClearFlag(uint32_t HASH_FLAG);
ITStatus HASH_GetITStatus(uint32_t HASH_IT);
void HASH_ClearITPendingBit(uint32_t HASH_IT);

 
ErrorStatus HASH_SHA1(uint8_t *Input, uint32_t Ilen, uint8_t Output[20]);
ErrorStatus HMAC_SHA1(uint8_t *Key, uint32_t Keylen,
                      uint8_t *Input, uint32_t Ilen,
                      uint8_t Output[20]);

 
ErrorStatus HASH_MD5(uint8_t *Input, uint32_t Ilen, uint8_t Output[16]);
ErrorStatus HMAC_MD5(uint8_t *Key, uint32_t Keylen,
                     uint8_t *Input, uint32_t Ilen,
                     uint8_t Output[16]);









  



  

 
#line 70 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_rng.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_rng.h"



 



  

 
  



 
  


  











  



   







  



  

 
  

  
void RNG_DeInit(void);

 
void RNG_Cmd(FunctionalState NewState);

 
uint32_t RNG_GetRandomNumber(void);

 
void RNG_ITConfig(FunctionalState NewState);
FlagStatus RNG_GetFlagStatus(uint8_t RNG_FLAG);
void RNG_ClearFlag(uint8_t RNG_FLAG);
ITStatus RNG_GetITStatus(uint8_t RNG_IT);
void RNG_ClearITPendingBit(uint8_t RNG_IT);










  



  

 
#line 71 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_can.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_can.h"



 



 

 
#line 57 "..\\FWLIB\\inc\\stm32f4xx_can.h"



 
typedef struct
{
  uint16_t CAN_Prescaler;   
 
  
  uint8_t CAN_Mode;         
 

  uint8_t CAN_SJW;          


 

  uint8_t CAN_BS1;          

 

  uint8_t CAN_BS2;          
 
  
  FunctionalState CAN_TTCM; 
 
  
  FunctionalState CAN_ABOM;  
 

  FunctionalState CAN_AWUM;  
 

  FunctionalState CAN_NART;  
 

  FunctionalState CAN_RFLM;  
 

  FunctionalState CAN_TXFP;  
 
} CAN_InitTypeDef;



 
typedef struct
{
  uint16_t CAN_FilterIdHigh;         

 

  uint16_t CAN_FilterIdLow;          

 

  uint16_t CAN_FilterMaskIdHigh;     


 

  uint16_t CAN_FilterMaskIdLow;      


 

  uint16_t CAN_FilterFIFOAssignment; 
 
  
  uint8_t CAN_FilterNumber;           

  uint8_t CAN_FilterMode;            
 

  uint8_t CAN_FilterScale;           
 

  FunctionalState CAN_FilterActivation; 
 
} CAN_FilterInitTypeDef;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     

 

  uint8_t Data[8]; 
 
} CanTxMsg;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     
 

  uint8_t Data[8]; 
 

  uint8_t FMI;     

 
} CanRxMsg;

 



 



 





 




 



 












 


 


   










 
  



   





 



 









 



 
#line 294 "..\\FWLIB\\inc\\stm32f4xx_can.h"




 



 
#line 311 "..\\FWLIB\\inc\\stm32f4xx_can.h"




 



 



 



 



 



 







 



 







 



 





 




 



 



 



 






 



 





 




 



 




 




 



 





 	






 



 






 



 



 	




 



 



 




 




                                                          
#line 486 "..\\FWLIB\\inc\\stm32f4xx_can.h"


 



 

 

 

 




 
#line 510 "..\\FWLIB\\inc\\stm32f4xx_can.h"

 



 

 





#line 531 "..\\FWLIB\\inc\\stm32f4xx_can.h"








 

  


  


 
#line 554 "..\\FWLIB\\inc\\stm32f4xx_can.h"

 



 






 





#line 579 "..\\FWLIB\\inc\\stm32f4xx_can.h"

#line 586 "..\\FWLIB\\inc\\stm32f4xx_can.h"


 



 

 
   

  
void CAN_DeInit(CAN_TypeDef* CANx);

  
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);



void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);

void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);



void CAN_SlaveStartBank(uint8_t CAN_BankNumber);

void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState);

 
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);

 
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);

 
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);

 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

 
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);









 



 

 
#line 72 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_dac.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_dac.h"



 



 

 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;

 



 



 

#line 89 "..\\FWLIB\\inc\\stm32f4xx_dac.h"




#line 102 "..\\FWLIB\\inc\\stm32f4xx_dac.h"



 



 

#line 117 "..\\FWLIB\\inc\\stm32f4xx_dac.h"


 



 

#line 149 "..\\FWLIB\\inc\\stm32f4xx_dac.h"

#line 174 "..\\FWLIB\\inc\\stm32f4xx_dac.h"


 



 







 



 







 



 

#line 212 "..\\FWLIB\\inc\\stm32f4xx_dac.h"


 



 







 



 




 
  


    





  



  
  





 



 

 
   

   
void DAC_DeInit(void);

 
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

 
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);

 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









 



 

 
#line 73 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"

























 

 







 
#line 38 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"



 



  

 


  
typedef struct
{
  uint16_t DCMI_CaptureMode;      
 

  uint16_t DCMI_SynchroMode;      
 

  uint16_t DCMI_PCKPolarity;      
 

  uint16_t DCMI_VSPolarity;       
 

  uint16_t DCMI_HSPolarity;       
 

  uint16_t DCMI_CaptureRate;      
 

  uint16_t DCMI_ExtendedDataMode; 
 
} DCMI_InitTypeDef;



  
typedef struct
{
  uint16_t DCMI_VerticalStartLine;      
 

  uint16_t DCMI_HorizontalOffsetCount;  
 

  uint16_t DCMI_VerticalLineCount;      
 

  uint16_t DCMI_CaptureCount;           

 
} DCMI_CROPInitTypeDef;



  
typedef struct
{
  uint8_t DCMI_FrameStartCode;  
  uint8_t DCMI_LineStartCode;   
  uint8_t DCMI_LineEndCode;     
  uint8_t DCMI_FrameEndCode;    
} DCMI_CodesInitTypeDef;

 



 



  
#line 120 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"


  




  
#line 134 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"


  




  






  




  






  




  






  




  
#line 184 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"


  




  
#line 200 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"


  




  
#line 219 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"


  




  


  





  







  
#line 262 "..\\FWLIB\\inc\\stm32f4xx_dcmi.h"
                                



  



  

 
  

  
void DCMI_DeInit(void);

 
void DCMI_Init(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_StructInit(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_CROPConfig(DCMI_CROPInitTypeDef* DCMI_CROPInitStruct);
void DCMI_CROPCmd(FunctionalState NewState);
void DCMI_SetEmbeddedSynchroCodes(DCMI_CodesInitTypeDef* DCMI_CodesInitStruct);
void DCMI_JPEGCmd(FunctionalState NewState);

 
void DCMI_Cmd(FunctionalState NewState);
void DCMI_CaptureCmd(FunctionalState NewState);
uint32_t DCMI_ReadData(void);

 
void DCMI_ITConfig(uint16_t DCMI_IT, FunctionalState NewState);
FlagStatus DCMI_GetFlagStatus(uint16_t DCMI_FLAG);
void DCMI_ClearFlag(uint16_t DCMI_FLAG);
ITStatus DCMI_GetITStatus(uint16_t DCMI_IT);
void DCMI_ClearITPendingBit(uint16_t DCMI_IT);









  



  

 
#line 74 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"



 



  

 
 


 

typedef struct
{
  uint32_t DMA2D_Mode;                           
 
  
  uint32_t DMA2D_CMode;                          
 

  uint32_t DMA2D_OutputBlue;                     





 

  uint32_t DMA2D_OutputGreen;                    





 
            
  uint32_t DMA2D_OutputRed;                      





 
  
  uint32_t DMA2D_OutputAlpha;                    



 

  uint32_t DMA2D_OutputMemoryAdd;                
 

  uint32_t DMA2D_OutputOffset;                   
 

  uint32_t DMA2D_NumberOfLine;                   
 
            
  uint32_t DMA2D_PixelPerLine;                   
 
} DMA2D_InitTypeDef;



typedef struct
{
  uint32_t DMA2D_FGMA;                           
 
  
  uint32_t DMA2D_FGO;                            
 

  uint32_t DMA2D_FGCM;                           
 

  uint32_t DMA2D_FG_CLUT_CM;                     
 
            
  uint32_t DMA2D_FG_CLUT_SIZE;                   
 
  
  uint32_t DMA2D_FGPFC_ALPHA_MODE;               
 

  uint32_t DMA2D_FGPFC_ALPHA_VALUE;              
 

  uint32_t DMA2D_FGC_BLUE;                       
 

  uint32_t DMA2D_FGC_GREEN;                      
 

  uint32_t DMA2D_FGC_RED;                        
 
            
  uint32_t DMA2D_FGCMAR;                         
 
} DMA2D_FG_InitTypeDef;


typedef struct
{
  uint32_t DMA2D_BGMA;                           
 
  
  uint32_t DMA2D_BGO;                            
 

  uint32_t DMA2D_BGCM;                           
 

  uint32_t DMA2D_BG_CLUT_CM;                     
 
            
  uint32_t DMA2D_BG_CLUT_SIZE;                   
 
  
  uint32_t DMA2D_BGPFC_ALPHA_MODE;               
 

  uint32_t DMA2D_BGPFC_ALPHA_VALUE;              
 

  uint32_t DMA2D_BGC_BLUE;                       
 

  uint32_t DMA2D_BGC_GREEN;                      
 

  uint32_t DMA2D_BGC_RED;                        
 
            
  uint32_t DMA2D_BGCMAR;                         
 
} DMA2D_BG_InitTypeDef;



 



   



 













   



 













   



 









   



 







   



 










   



 








   




 

#line 294 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"

#line 301 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"

#line 308 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"



 



 










 



 



















 



 















 



 

#line 382 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"







 
      


 

#line 401 "..\\FWLIB\\inc\\stm32f4xx_dma2d.h"









 
      


 


  









 
  


 

 
 

 
void DMA2D_DeInit(void);

 
void DMA2D_Init(DMA2D_InitTypeDef* DMA2D_InitStruct);
void DMA2D_StructInit(DMA2D_InitTypeDef* DMA2D_InitStruct);
void DMA2D_StartTransfer(void);
void DMA2D_AbortTransfer(void);
void DMA2D_Suspend(FunctionalState NewState);
void DMA2D_FGConfig(DMA2D_FG_InitTypeDef* DMA2D_FG_InitStruct);
void DMA2D_FG_StructInit(DMA2D_FG_InitTypeDef* DMA2D_FG_InitStruct);
void DMA2D_BGConfig(DMA2D_BG_InitTypeDef* DMA2D_BG_InitStruct);
void DMA2D_BG_StructInit(DMA2D_BG_InitTypeDef* DMA2D_BG_InitStruct);
void DMA2D_FGStart(FunctionalState NewState);
void DMA2D_BGStart(FunctionalState NewState);
void DMA2D_DeadTimeConfig(uint32_t DMA2D_DeadTime, FunctionalState NewState);
void DMA2D_LineWatermarkConfig(uint32_t DMA2D_LWatermarkConfig);

 
void DMA2D_ITConfig(uint32_t DMA2D_IT, FunctionalState NewState);
FlagStatus DMA2D_GetFlagStatus(uint32_t DMA2D_FLAG);
void DMA2D_ClearFlag(uint32_t DMA2D_FLAG);
ITStatus DMA2D_GetITStatus(uint32_t DMA2D_IT);
void DMA2D_ClearITPendingBit(uint32_t DMA2D_IT);









 



 

 
#line 75 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"


























 

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"



 



 

   



 
typedef struct
{
  uint32_t FMC_AddressSetupTime;       


 

  uint32_t FMC_AddressHoldTime;        


 

  uint32_t FMC_DataSetupTime;          


 

  uint32_t FMC_BusTurnAroundDuration;  


 

  uint32_t FMC_CLKDivision;            

 

  uint32_t FMC_DataLatency;            





 

  uint32_t FMC_AccessMode;             
 
}FMC_NORSRAMTimingInitTypeDef;



 
typedef struct
{
  uint32_t FMC_Bank;                
 

  uint32_t FMC_DataAddressMux;      

 

  uint32_t FMC_MemoryType;          

 

  uint32_t FMC_MemoryDataWidth;     
 

  uint32_t FMC_BurstAccessMode;     

                                         

  uint32_t FMC_WaitSignalPolarity;  

 

  uint32_t FMC_WrapMode;            

 

  uint32_t FMC_WaitSignalActive;    


 

  uint32_t FMC_WriteOperation;      
 

  uint32_t FMC_WaitSignal;          

 

  uint32_t FMC_ExtendedMode;        
 
  
  uint32_t FMC_AsynchronousWait;     

   

  uint32_t FMC_WriteBurst;          
  

  uint32_t FMC_ContinousClock;       


  

  
  FMC_NORSRAMTimingInitTypeDef* FMC_ReadWriteTimingStruct;    

  FMC_NORSRAMTimingInitTypeDef* FMC_WriteTimingStruct;            
}FMC_NORSRAMInitTypeDef;



 
typedef struct
{
  uint32_t FMC_SetupTime;      



 

  uint32_t FMC_WaitSetupTime;  



 

  uint32_t FMC_HoldSetupTime;  




 

  uint32_t FMC_HiZSetupTime;   



 
}FMC_NAND_PCCARDTimingInitTypeDef;



 
typedef struct
{
  uint32_t FMC_Bank;              
 

  uint32_t FMC_Waitfeature;      
 

  uint32_t FMC_MemoryDataWidth;  
 

  uint32_t FMC_ECC;              
 

  uint32_t FMC_ECCPageSize;      
 

  uint32_t FMC_TCLRSetupTime;    

 

  uint32_t FMC_TARSetupTime;     

  

  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_CommonSpaceTimingStruct;     

  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_AttributeSpaceTimingStruct;  
}FMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FMC_Waitfeature;    
 

  uint32_t FMC_TCLRSetupTime;  

 

  uint32_t FMC_TARSetupTime;   

  

  
  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_CommonSpaceTimingStruct;  

  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_AttributeSpaceTimingStruct;    
  
  FMC_NAND_PCCARDTimingInitTypeDef*  FMC_IOSpaceTimingStruct;    
}FMC_PCCARDInitTypeDef;



 
  
typedef struct
{
  uint32_t FMC_LoadToActiveDelay;      

 
  
  uint32_t FMC_ExitSelfRefreshDelay;   

 
   
  uint32_t FMC_SelfRefreshTime;        

 
                                            
  uint32_t FMC_RowCycleDelay;          


 
                                            
  uint32_t FMC_WriteRecoveryTime;      
 
                                            
  uint32_t FMC_RPDelay;                

 
                                            
  uint32_t FMC_RCDDelay;               

 
                                            
}FMC_SDRAMTimingInitTypeDef;



 


typedef struct
{
  uint32_t FMC_CommandMode;            
 
                                            
  uint32_t FMC_CommandTarget;          
 
                                            
  uint32_t FMC_AutoRefreshNumber;      

                                            
                                                                                                             
  uint32_t FMC_ModeRegisterDefinition;  
  
}FMC_SDRAMCommandTypeDef;



 

typedef struct
{
  uint32_t FMC_Bank;                   
 

  uint32_t FMC_ColumnBitsNumber;       
 
                                            
  uint32_t FMC_RowBitsNumber;          
 
                                            
  uint32_t FMC_SDMemoryDataWidth;        
 
                                            
  uint32_t FMC_InternalBankNumber;     
 
                                            
  uint32_t FMC_CASLatency;             
 
                                            
  uint32_t FMC_WriteProtection;        
 
                                            
  uint32_t FMC_SDClockPeriod;          

 
                                            
  uint32_t FMC_ReadBurst;              

 
                                            
  uint32_t FMC_ReadPipeDelay;          
 
                                            
  FMC_SDRAMTimingInitTypeDef* FMC_SDRAMTimingStruct;                                                
  
}FMC_SDRAMInitTypeDef;


 



  



 











 



   







 



     



 



 








                                

                              


 



 








 



 










 



 










 



 








 
    


 







 



 







 



 







 



 







 



 







 



 







 



 







 



 








 
  


 








   



 



 



 



 



 



 



 



 



 



 



 



 



 











 



 
  


 



 







 



 







 



 







 



 
#line 687 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"

#line 694 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"


 



 



 



 



 



 



 



 



 



 



 



 



 



   




 
        


 












 
  


 










   



 










 
  


 








   
  
  


 










   



 








   
  



 










  
  


 








 



 










 
  


 



 
  


 



  
     


   



 
  


   



   
  


   



          
  


   



  
  


   




   
  


 
#line 955 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"

#line 963 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"



 



 










    
  


   




 



 




 
  



 











       



   



 










                           







 



 
#line 1054 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"

#line 1061 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"

#line 1068 "..\\FWLIB\\inc\\stm32f4xx_fmc.h"
                                   





 



 




 



 


 
  

 
void FMC_NORSRAMDeInit(uint32_t FMC_Bank);
void FMC_NORSRAMInit(FMC_NORSRAMInitTypeDef* FMC_NORSRAMInitStruct);
void FMC_NORSRAMStructInit(FMC_NORSRAMInitTypeDef* FMC_NORSRAMInitStruct);
void FMC_NORSRAMCmd(uint32_t FMC_Bank, FunctionalState NewState);

 
void     FMC_NANDDeInit(uint32_t FMC_Bank);
void     FMC_NANDInit(FMC_NANDInitTypeDef* FMC_NANDInitStruct);
void     FMC_NANDStructInit(FMC_NANDInitTypeDef* FMC_NANDInitStruct);
void     FMC_NANDCmd(uint32_t FMC_Bank, FunctionalState NewState);
void     FMC_NANDECCCmd(uint32_t FMC_Bank, FunctionalState NewState);
uint32_t FMC_GetECC(uint32_t FMC_Bank);

 
void FMC_PCCARDDeInit(void);
void FMC_PCCARDInit(FMC_PCCARDInitTypeDef* FMC_PCCARDInitStruct);
void FMC_PCCARDStructInit(FMC_PCCARDInitTypeDef* FMC_PCCARDInitStruct);
void FMC_PCCARDCmd(FunctionalState NewState);

 
void     FMC_SDRAMDeInit(uint32_t FMC_Bank);
void     FMC_SDRAMInit(FMC_SDRAMInitTypeDef* FMC_SDRAMInitStruct);
void     FMC_SDRAMStructInit(FMC_SDRAMInitTypeDef* FMC_SDRAMInitStruct);
void     FMC_SDRAMCmdConfig(FMC_SDRAMCommandTypeDef* FMC_SDRAMCommandStruct);
uint32_t FMC_GetModeStatus(uint32_t SDRAM_Bank);
void     FMC_SetRefreshCount(uint32_t FMC_Count);
void     FMC_SetAutoRefresh_Number(uint32_t FMC_Number);
void     FMC_SDRAMWriteProtectionConfig(uint32_t SDRAM_Bank, FunctionalState NewState);

 
void       FMC_ITConfig(uint32_t FMC_Bank, uint32_t FMC_IT, FunctionalState NewState);
FlagStatus FMC_GetFlagStatus(uint32_t FMC_Bank, uint32_t FMC_FLAG);
void       FMC_ClearFlag(uint32_t FMC_Bank, uint32_t FMC_FLAG);
ITStatus   FMC_GetITStatus(uint32_t FMC_Bank, uint32_t FMC_IT);
void       FMC_ClearITPendingBit(uint32_t FMC_Bank, uint32_t FMC_IT);








 



  

 
#line 76 "..\\USER\\stm32f4xx_conf.h"
#line 1 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


























  

 







 
#line 39 "..\\FWLIB\\inc\\stm32f4xx_sai.h"



 



  



 



 

typedef struct
{
  uint32_t SAI_AudioMode;           
 

  uint32_t SAI_Protocol;             
 

  uint32_t SAI_DataSize;            

 

  uint32_t SAI_FirstBit;            

 

  uint32_t SAI_ClockStrobing;       
 

  uint32_t SAI_Synchro;             
 
                                           
  uint32_t SAI_SynchroExt;          



 
 
  uint32_t SAI_OUTDRIV;             


 

  uint32_t SAI_NoDivider;            
 

  uint32_t SAI_MasterDivider;       

 
                                               
  uint32_t SAI_FIFOThreshold;      
                                                                                              
}SAI_InitTypeDef;



 

typedef struct
{

  uint32_t SAI_FrameLength;         





 
                                   
  uint32_t SAI_ActiveFrameLength;   



 

  uint32_t SAI_FSDefinition;        

 

  uint32_t SAI_FSPolarity;          

 

  uint32_t SAI_FSOffset;            

 

}SAI_FrameInitTypeDef;



     

typedef struct
{
  uint32_t SAI_FirstBitOffset;      

 

  uint32_t SAI_SlotSize;            

 

  uint32_t SAI_SlotNumber;          

 

  uint32_t SAI_SlotActive;          

  
}SAI_SlotInitTypeDef;

 



 

#line 172 "..\\FWLIB\\inc\\stm32f4xx_sai.h"











 
#line 192 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 



 

#line 206 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 



 

#line 226 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


  



 







 



 







 



 

#line 264 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


  



 
#line 277 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 



 







  





 







 
  



 




 
  


 




 
    


 




 



 







 



 







 
            


 
  






 
  


 




 

  

 
#line 389 "..\\FWLIB\\inc\\stm32f4xx_sai.h"



 



 




 
  


 
#line 424 "..\\FWLIB\\inc\\stm32f4xx_sai.h"





 



 







 



 







 



 

#line 469 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 
  


 
  
#line 487 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 



 
  






 



 
  




 



 

#line 524 "..\\FWLIB\\inc\\stm32f4xx_sai.h"

#line 532 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 



 

#line 547 "..\\FWLIB\\inc\\stm32f4xx_sai.h"

#line 555 "..\\FWLIB\\inc\\stm32f4xx_sai.h"
                                   
#line 563 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 
  


 
#line 576 "..\\FWLIB\\inc\\stm32f4xx_sai.h"

#line 583 "..\\FWLIB\\inc\\stm32f4xx_sai.h"


 

  


 

 
  

  
void SAI_DeInit(SAI_TypeDef* SAIx);

 
void SAI_Init(SAI_Block_TypeDef* SAI_Block_x, SAI_InitTypeDef* SAI_InitStruct);
void SAI_FrameInit(SAI_Block_TypeDef* SAI_Block_x, SAI_FrameInitTypeDef* SAI_FrameInitStruct);
void SAI_SlotInit(SAI_Block_TypeDef* SAI_Block_x, SAI_SlotInitTypeDef* SAI_SlotInitStruct);
void SAI_StructInit(SAI_InitTypeDef* SAI_InitStruct);
void SAI_FrameStructInit(SAI_FrameInitTypeDef* SAI_FrameInitStruct);
void SAI_SlotStructInit(SAI_SlotInitTypeDef* SAI_SlotInitStruct);

void SAI_Cmd(SAI_Block_TypeDef* SAI_Block_x, FunctionalState NewState);
void SAI_MonoModeConfig(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_Mono_StreoMode);
void SAI_TRIStateConfig(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_TRIState);
void SAI_CompandingModeConfig(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_CompandingMode);
void SAI_MuteModeCmd(SAI_Block_TypeDef* SAI_Block_x, FunctionalState NewState);
void SAI_MuteValueConfig(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_MuteValue);
void SAI_MuteFrameCounterConfig(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_MuteCounter);
void SAI_FlushFIFO(SAI_Block_TypeDef* SAI_Block_x);


void SAI_BlockSynchroConfig(SAI_InitTypeDef* SAI_InitStruct, SAI_TypeDef* SAIx);

  
void SAI_SendData(SAI_Block_TypeDef* SAI_Block_x, uint32_t Data);
uint32_t SAI_ReceiveData(SAI_Block_TypeDef* SAI_Block_x);

 
void SAI_DMACmd(SAI_Block_TypeDef* SAI_Block_x, FunctionalState NewState);

 
void SAI_ITConfig(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_IT, FunctionalState NewState);
FlagStatus SAI_GetFlagStatus(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_FLAG);
void SAI_ClearFlag(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_FLAG);
ITStatus SAI_GetITStatus(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_IT);
void SAI_ClearITPendingBit(SAI_Block_TypeDef* SAI_Block_x, uint32_t SAI_IT);
FunctionalState SAI_GetCmdStatus(SAI_Block_TypeDef* SAI_Block_x);
uint32_t SAI_GetFIFOStatus(SAI_Block_TypeDef* SAI_Block_x);




 



 







 
#line 77 "..\\USER\\stm32f4xx_conf.h"


#line 88 "..\\USER\\stm32f4xx_conf.h"































#line 127 "..\\USER\\stm32f4xx_conf.h"

#line 138 "..\\USER\\stm32f4xx_conf.h"

 
 



 
   



 
 

 
#line 169 "..\\USER\\stm32f4xx_conf.h"



 
#line 12028 "..\\USER\\stm32f4xx.h"




 

















 









 

  

 

 
#line 5 "..\\USER\\main.h"
#line 1 "..\\SYSTEM\\usart\\usart.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 985 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 4 "..\\SYSTEM\\usart\\usart.h"
#line 5 "..\\SYSTEM\\usart\\usart.h"
#line 1 "..\\SYSTEM\\sys\\sys.h"
#line 4 "..\\SYSTEM\\sys\\sys.h"




















																	    
	 







#line 42 "..\\SYSTEM\\sys\\sys.h"

#line 52 "..\\SYSTEM\\sys\\sys.h"
 






























void WFI_SET(void);		
void INTX_DISABLE(void);
void INTX_ENABLE(void);	
void MSR_MSP(u32 addr);	












#line 6 "..\\SYSTEM\\usart\\usart.h"

























	  	
extern u8  USART_RX_BUF[200]; 
extern u16 USART_RX_STA;         		

void uart_init(u32 bound);



#line 6 "..\\USER\\main.h"
#line 1 "..\\SYSTEM\\delay\\delay.h"
#line 4 "..\\SYSTEM\\delay\\delay.h"
















void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);































#line 7 "..\\USER\\main.h"
#line 1 "..\\BSP\\bsp.h"



#line 5 "..\\BSP\\bsp.h"

void BSP_Init(void);





#line 8 "..\\USER\\main.h"
#line 1 "..\\BSP\\can1.h"
#line 4 "..\\BSP\\can1.h"

uint8_t CAN1_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
void CAN1_STOP(void);
void CNA1_START(void);
#line 9 "..\\USER\\main.h"
#line 1 "..\\BSP\\can2.h"
#line 4 "..\\BSP\\can2.h"


uint8_t CAN2_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
void CAN2_STOP(void);
void CAN2_START(void);
#line 10 "..\\USER\\main.h"
#line 1 "..\\BSP\\timer.h"



#line 1 "..\\USER\\main.h"
#line 5 "..\\BSP\\timer.h"

void TIM6_Configure(void);
void TIM6_Start(void);
void TIM6_Stop(void);
void TIM4_FireMotor_Configure(void);
void TIM3_Int_Init(void);

#line 11 "..\\USER\\main.h"
#line 1 "..\\BSP\\rm_pid.h"



#line 5 "..\\BSP\\rm_pid.h"

#line 15 "..\\BSP\\rm_pid.h"




















typedef struct PID
{
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	
	float poutmax;
	float ioutmax;
	float doutmax;
	float outmax;
	
	float set;
	float real;
	float out;
	
	float err;							
	float err_last;					
	float err_llast;				
	float integral;					
	void(*f_pid_init)(struct PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);			
	void(*f_pid_reset)(struct PID *pid);
}PID;

typedef struct
{
	 
    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 
	
    float set;
    float fdb;	

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  
    float error[3]; 
	
}PID_INCR;


void pid_incr_init(PID_INCR *pid, const float PID[3], float max_out, float max_iout);
float pid_incr_calc(PID_INCR *pid, float ref, float set);
void ALL_Pid_Incr_Configuration(void);
extern PID_INCR pid_incr[6];
extern float out_incr[6];


extern PID pid[17];
extern float out[17];
static void pid_init(PID *pid, float kp, float ki, float kd, float poutmax, float ioutmax, float doutmax, float outmax);
static void pid_reset(PID *pid);

void All_Pid_Configuration(PID pid[]);
float Calculate_Current_Value(PID *pid, float set, float real);
float Calculate_Current_Value_For_Err(PID *pid, float err);
void Look(void);


#line 12 "..\\USER\\main.h"
#line 1 "..\\BSP\\encoder.h"



#line 5 "..\\BSP\\encoder.h"














typedef struct{
	int32_t raw_value;   					 
	int32_t last_raw_value;					 
	int32_t ecd_value;                       
	int32_t diff;							 
	int32_t temp_count;                      
	uint8_t buf_count;						 
	int32_t ecd_bias;						 
	int32_t ecd_raw_rate;					 
	int32_t rate_buf[6];	     
	int32_t round_cnt;						 
	int32_t filter_rate;					 
	int32_t filter_rate_max;				 
	float ecd_angle;						 
	float init_angle; 						 
	
	int16_t current;
	
}Encoder;

void CanReceiveMsgProcess(CanRxMsg *message);
void getEncoderData(volatile Encoder *v, CanRxMsg * msg);

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder CMFeedEncoder;


#line 13 "..\\USER\\main.h"
#line 1 "..\\HAL\\cloudmotor.h"



#line 5 "..\\HAL\\cloudmotor.h"


typedef struct Angle
{
	float yaw_target;
	float pitch_target;							
	float yaw_set;
	float pitch_set;								
	float pitch_low;
	float pitch_top;
	float yaw_right;
	float yaw_left;									




	int back_flag;									
	float pitch_remote_last;
	float pitch_remote_now;
	float pitch_differ;
	float pitch_set_last;


	float pitch_first;
	float yaw_first;
}Angle;

void angle_init(struct Angle *angle);
void angle_out_update(struct Angle *angle);
void angle_set_update(struct Angle *angle);
void Set_CloudMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);


extern int fly_flag;
extern Angle t_angle;









#line 14 "..\\USER\\main.h"
#line 1 "..\\HAL\\chassismotor.h"



#line 5 "..\\HAL\\chassismotor.h"





































#line 51 "..\\HAL\\chassismotor.h"















typedef __packed struct
{
    float input;        
    float out;          
    float num[1];       
    float frame_period; 
} first_order_filter_type_t;

typedef struct Chassis_set
{
	
	float cm_set[4];
	
	float cm1_real;
	float cm2_real;
	float cm3_real;
	float cm4_real;
	float follow_set;
	float follow_real;
	float YAW_CENTRE;
	float YAW_CENTRE_Init;
	float YAW_DIR;
	float total_current_real;
	float total_current_set;
	float total_current;
	float RAD_RC_last;
	float RAD_RC_now;
	float RAD_RC_diff;
	float current_ratio;
	first_order_filter_type_t chassis_cmd_slow_set_vx;
	first_order_filter_type_t chassis_cmd_slow_set_vy;
	first_order_filter_type_t chassis_cmd_slow_set_follow;
	first_order_filter_type_t chassis_cmd_slow_set_follow_second;
	first_order_filter_type_t chassis_cmd_slow_set_second_vx;
	first_order_filter_type_t chassis_cmd_slow_set_second_vy;
	first_order_filter_type_t chassis_cmd_stop_set_vx;
	first_order_filter_type_t chassis_cmd_stop_set_vy;
	
	float chassis_max_current;
	float x_filted;
	float y_filted;
	float follow_filted;
}Chassis_set;



extern Chassis_set chassis_set;
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void chassis_init(void);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input, u8 filter);
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void chassis_set_update(void);
void chassis_out_update(void);
void mecanum_Resolving(Chassis_set *set, int z);    
void Chassis_fllow(float *x, float *y);
void remote_fifo(void);


#line 15 "..\\USER\\main.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"




 





 












 






   









 






#line 61 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 75 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   




 















 
#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











 





extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double  , double  );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 230 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   
  typedef float float_t;
  typedef double double_t;
#line 251 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"



extern const int math_errhandling;
#line 261 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }


    inline float _sqrtf(float __x) { return __sqrtf(__x); }



    



 

extern __declspec(__nothrow) __attribute__((const)) double ceil(double  );
    
    
extern __declspec(__nothrow) __attribute__((const)) double fabs(double  );
    
    

extern __declspec(__nothrow) __attribute__((const)) double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __attribute__((const)) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __attribute__((const)) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 479 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __attribute__((const)) double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __attribute__((const)) float _fabsf(float);  
inline __declspec(__nothrow) __attribute__((const)) float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __attribute__((const)) float ceilf(float  );
extern __declspec(__nothrow) __attribute__((const)) float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 803 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );

inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __declspec(__nothrow) __attribute__((const)) double fmax(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __attribute__((const)) double fmin(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );

inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }


extern __declspec(__nothrow) long long llrint(double  );
extern __declspec(__nothrow) long long llrintf(float  );

inline __declspec(__nothrow) long long llrintl(long double __x)     { return llrint((double)__x); }


extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );

inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }


extern __declspec(__nothrow) long long llround(double  );
extern __declspec(__nothrow) long long llroundf(float  );

inline __declspec(__nothrow) long long llroundl(long double __x)     { return llround((double)__x); }


extern __declspec(__nothrow) __attribute__((const)) double nan(const char * );
extern __declspec(__nothrow) __attribute__((const)) float nanf(const char * );

inline __declspec(__nothrow) __attribute__((const)) long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 856 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) __attribute__((const)) double nearbyint(double  );
extern __declspec(__nothrow) __attribute__((const)) float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int * );
extern  float remquof(float  , float  , int * );

inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }


extern __declspec(__nothrow) __attribute__((const)) double round(double  );
extern __declspec(__nothrow) __attribute__((const)) float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __attribute__((const)) double trunc(double  );
extern __declspec(__nothrow) __attribute__((const)) float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 1034 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











#line 1250 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





 
#line 16 "..\\USER\\main.h"
#line 1 "..\\TASK\\control_task.h"



#line 5 "..\\TASK\\control_task.h"

void control_task(void);

#line 17 "..\\USER\\main.h"
#line 1 "..\\BSP\\remote.h"
















 

#line 22 "..\\BSP\\remote.h"

typedef struct
{
	int16_t R_x;  
	int16_t R_y;	 
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			
	uint8_t sr;
	
	int16_t mouse_x;	
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	
	uint8_t mouse_r;
	uint16_t key;		
	u8 W;
	u8 S;
	u8 A;
	u8 D;
	u8 Q;
	u8 E;
	u8 Shift;
	u8 Ctrl;
	
	uint64_t cnts;
	uint64_t off_line_times;
	uint64_t last_cnts;
	uint64_t now_cnts;
	uint64_t off_line_flag;
}RC;
extern RC rc;
void Remote_uart1_init(void);	
void remote_off_line(void);


#line 18 "..\\USER\\main.h"
#line 1 "..\\HAL\\firemotor.h"
#line 4 "..\\HAL\\firemotor.h"






typedef struct 
{
	float input_L;
	float input_R;
	float out_L;
	float out_R;
	float pid_out_L;
	float pid_out_R;
	uint8_t back_flag;
	
	float frame_period;
	uint8_t add_each_second;
	float add_real;
}Ramp_Start;

typedef struct encoder__
{
    int32_t value;
    int32_t value_last;
    int32_t diff;
    uint8_t buf_count;						 
	int32_t rate_buf[5];	     
	int32_t ecd_raw_rate;					 
	int32_t filter_rate;					 
	
}Fire_Enconder;     

extern Ramp_Start ramp_start;
extern Fire_Enconder fire_enconderL;
extern Fire_Enconder fire_enconderR;

void fireMotor_stop(void);
void fireMotor_fire(void);
void fireMotor(void);
void ramp_calc(void);
void ramp_init(void);
void fireMotor_keep(void);
void enconderSys_Configure(void);
void getEnconderData_fire(Fire_Enconder *Enc, char TIM_temp);








#line 19 "..\\USER\\main.h"
#line 1 "..\\HAL\\feedmotor.h"
#line 4 "..\\HAL\\feedmotor.h"





typedef struct Feed_Set
{
	
	float feed_angle_set;
	float feed_speed_set;
	
	
	float feed_angle_real;
	float feed_speed_real;
	
	float feed_angle_and;
}Feed_set;

typedef struct Feed_Motor_State
{	
	int16_t if_dance;
	int16_t dance_flag;				
	int16_t last_dance_flag;		
	int16_t dance_time;				
	uint64_t presstime_1ms;			
	int16_t press_flag;				
	int16_t now_mouse_l;			
	int64_t sum_angle;				
	int64_t press_number;			
	int16_t pressState;				
	int64_t jamTime_1ms;			
	int16_t now_Jam;				
	int16_t last_Jam;				
	uint8_t shoot_plan;				
	uint8_t shoot_plan_back;
	uint8_t heat_control_state;
}Feed_Motor_State ;

extern Feed_set feed_set;
extern Feed_Motor_State feed_state;
void shoot(int t,int speed);
int feedBullet(int t,int speed);
void stopFeedBullet(void);
int checkStop(void);
int returNormal(int t,int speed);
float fabsValue(float a);
void shoot2(void);
void fireMotorPidReal_update(void);
void fireMotorDoubleLoopPid_out_update(void);
void fireMotorSingleLoopPid_out_update(void);
void Shoot(void);
int16_t IfJam(void);
int16_t IfJam2(int16_t state);
void Set_CloudMotor_FeedMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t feed_motor_iq);
int16_t Judge_Dance(void);
void shoot_plan(void);
void Set_FeedMotor_Current(int16_t feed_motor_iq);






#line 20 "..\\USER\\main.h"
#line 1 "..\\BSP\\crc.h"



#line 1 "..\\BSP\\serial.h"
#line 4 "..\\BSP\\serial.h"









typedef union {
	int16_t d;
	unsigned char c[2];
}int16uchar;


typedef union {
	float f;
	unsigned char c[4];
}float2uchar;


typedef struct {
	unsigned char mode; 
	int16uchar SEE_yaw_angle;
	int16uchar SEE_pitch_angle;
	int16uchar SEE_yaw_speed;
	float2uchar SEE_shoot_speed;
	uint8_t    which;
	uint8_t    pos;

	float2uchar x;
	float2uchar y;
	float2uchar z;
}VisionData;

typedef struct{
	float yaw_offset;
	float pitch_offset;

}Final_Offset_Angle;
extern VisionData enemy_position;

extern u8 SEE_RXBuff[18];		
extern u8 SEE_rx_data[18];





extern unsigned char mode;
extern int16_t SEE_yaw_angle;
extern int16_t SEE_pitch_angle;
extern float z;


void getVisionData(u8* data,VisionData* enemy_position);
void miniPC_uart6_init(void);
void miniPC_uart6_tx(u8 *USART_RX_BUF ,int len);

float usart_IIRLowPass(float x);




#line 5 "..\\BSP\\crc.h"









typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length; 
  uint8_t  seq;  
  uint8_t  crc8;  
	
} frame_header_t;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);



#line 21 "..\\USER\\main.h"
#line 22 "..\\USER\\main.h"
#line 1 "..\\HAL\\miniPC.h"



#line 5 "..\\HAL\\miniPC.h"

void miniPC_Init(void);
void miniPC_rx(void);

#line 23 "..\\USER\\main.h"
#line 1 "..\\BSP\\stmflash.h"
#line 4 "..\\BSP\\stmflash.h"
























#line 40 "..\\BSP\\stmflash.h"

extern float OFFSET_Buffer[6];
u32 STMFLASH_ReadWord(u32 faddr);		  	
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		
void flash_write(void);	
void Read_Offset(void);


















#line 24 "..\\USER\\main.h"
#line 25 "..\\USER\\main.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 26 "..\\USER\\main.h"
#line 27 "..\\USER\\main.h"
#line 28 "..\\USER\\main.h"
#line 1 "..\\BSP\\spi.h"
#line 4 "..\\BSP\\spi.h"
void SPI5Init(void);
void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize);
void SPI5_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI5_DMA_Enable(uint16_t ndtr);
void SPI5_DMA_DISABLE(void);
#line 29 "..\\USER\\main.h"
#line 1 "..\\AHRS\\mpu6500driver.h"



#line 5 "..\\AHRS\\mpu6500driver.h"











#line 31 "..\\AHRS\\mpu6500driver.h"

















typedef struct mpu6500_real_data_t
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
} mpu6500_real_data_t;


extern uint8_t mpu6500_init(void);

extern void mpu6500_read_over(uint8_t *status_buf, mpu6500_real_data_t *mpu6500_real_data);

extern void gyro_offset(float gyro_offset[3], float gyro[3], uint8_t imu_status, uint16_t *offset_time_count);

extern void mpu6500_read_gyro(float gyro[3]);
extern void mpu6500_read_accel(float accel[3]);
extern void mpu6500_read_temp(float *temperature);
#line 30 "..\\USER\\main.h"
#line 1 "..\\AHRS\\mpu6500driver_middleware.h"



#line 5 "..\\AHRS\\mpu6500driver_middleware.h"









extern void mpu6500_GPIO_init(void);
extern void mpu6500_com_init(void);

extern void mpu6500_middleware_delay_ms(uint16_t ms);
extern void mpu6500_middleware_delay_us(uint32_t us);







extern void mpu6500_SPI_NS_H(void);
extern void mpu6500_SPI_NS_L(void);


extern void mpu6500_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t mpu6500_read_single_reg(uint8_t reg);
extern void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);





#line 31 "..\\USER\\main.h"
#line 1 "..\\AHRS\\ist8310driver.h"


















 

#line 24 "..\\AHRS\\ist8310driver.h"
typedef float fp32;







typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(fp32 mag[3]);
#line 32 "..\\USER\\main.h"
#line 1 "..\\AHRS\\ist8310driver_middleware.h"
















 




#line 23 "..\\AHRS\\ist8310driver_middleware.h"





extern void ist8310_GPIO_init(void); 
extern void ist8310_com_init(void);  
extern void ist8310_auto_com_by_mpu6500(void);
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); 
extern void ist8310_RST_L(void); 

#line 33 "..\\USER\\main.h"
#line 1 "..\\AHRS\\mpu6500reg.h"





















#line 28 "..\\AHRS\\mpu6500reg.h"






#line 44 "..\\AHRS\\mpu6500reg.h"








#line 61 "..\\AHRS\\mpu6500reg.h"

#line 71 "..\\AHRS\\mpu6500reg.h"






#line 83 "..\\AHRS\\mpu6500reg.h"
















#line 105 "..\\AHRS\\mpu6500reg.h"














#line 128 "..\\AHRS\\mpu6500reg.h"




#line 141 "..\\AHRS\\mpu6500reg.h"



















































#line 209 "..\\AHRS\\mpu6500reg.h"

#line 227 "..\\AHRS\\mpu6500reg.h"





















































































#line 326 "..\\AHRS\\mpu6500reg.h"

#line 351 "..\\AHRS\\mpu6500reg.h"



































































































































#line 488 "..\\AHRS\\mpu6500reg.h"



#line 34 "..\\USER\\main.h"
#line 1 "..\\HAL\\imu.h"



#line 5 "..\\HAL\\imu.h"





void IMU_Configure(void);
void IMU_INT_Configure(void);
void IMU_EXIT_STOP(void);
void IMU_EXIT_START(void);
void IMU_control_decide(void);

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef struct packed_angle
{
	float last_yaw;
	float new_yaw;
	float diff_yaw;
	float yaw_counts;
	float yaw;
	float pitch;
	float roll;
	float gx;
	float gy;
	float gz;
	float temperature;
	float state;
}packed_angle;

typedef struct
{
	float default_temp;
	float all_temp;
	uint8_t cnt;
}Imu_Temp;
extern GyroCaliStruct_t GyroSavedCaliData; 
extern volatile packed_angle real_angle;
extern float Gyro_z_err;
extern float Gyro_x_err;
extern float Gyro_y_err;

#line 35 "..\\USER\\main.h"
#line 1 "..\\BSP\\pwm.h"



#line 5 "..\\BSP\\pwm.h"


void TIM1_FireMotor_Configure(void);
void TIM3_Init(uint16_t arr, uint16_t psc);
void heat_output(uint16_t pwm);


#line 36 "..\\USER\\main.h"
#line 1 "..\\BSP\\laser.h"
#line 4 "..\\BSP\\laser.h"

void laser_configuration(void);
void laser_on(void);
void laser_off(void);
#line 37 "..\\USER\\main.h"
#line 1 "..\\BSP\\buzzer.h"
#line 4 "..\\BSP\\buzzer.h"
extern void buzzer_init(uint16_t arr, uint16_t psc);
extern void buzzer_on(void);
extern void buzzer_off(void);



#line 38 "..\\USER\\main.h"
#line 1 "..\\AHRS\\INS_task.h"
	



#line 6 "..\\AHRS\\INS_task.h"
#line 7 "..\\AHRS\\INS_task.h"

typedef float fp32;























typedef struct 
{
    fp32 offset[3]; 
    fp32 scale[3];  
} imu_cali_t;

int8_t cali_gyro_hook(void);
extern void INSTask(void);

extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[6]);
extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);
extern fp32 INS_Angle[3];
extern uint8_t mpu6500_spi_rxbuf[23]; 
extern mpu6500_real_data_t mpu6500_real_data; 


#line 39 "..\\USER\\main.h"
#line 1 "..\\AHRS\\AHRS.h"



#line 1 "..\\AHRS\\AHRS_MiddleWare.h"

















 


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

 
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;






















extern void AHRS_get_height(fp32 *high);
extern void AHRS_get_latitude(fp32 *latitude);
extern fp32 AHRS_invSqrt(fp32 num);
extern fp32 AHRS_sinf(fp32 angle);
extern fp32 AHRS_cosf(fp32 angle);
extern fp32 AHRS_tanf(fp32 angle);
extern fp32 AHRS_asinf(fp32 sin);
extern fp32 AHRS_acosf(fp32 cos);
extern fp32 AHRS_atan2f(fp32 y, fp32 x);
#line 5 "..\\AHRS\\AHRS.h"








 
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);










 
extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);






 
extern fp32 get_yaw(const fp32 quat[4]);






 
extern fp32 get_pitch(const fp32 quat[4]);





 
extern fp32 get_roll(const fp32 quat[4]);








 
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);





 

extern fp32 get_carrier_gravity(void);

#line 40 "..\\USER\\main.h"
#line 1 "..\\BSP\\power_24V.h"



#line 5 "..\\BSP\\power_24V.h"









void power_ctrl_configuration(void);

void power_ctrl_on(uint8_t num);
void power_ctrl_off(uint8_t num);
void power_ctrl_toggle(uint8_t num);


#line 41 "..\\USER\\main.h"
#line 1 "..\\BSP\\shoot_plan.h"



#line 5 "..\\BSP\\shoot_plan.h"




void SHOOT_PLAN_Init(void);



#line 42 "..\\USER\\main.h"
#line 1 "..\\AHRS\\adc.h"
#line 4 "..\\AHRS\\adc.h"

typedef float fp32;

typedef struct 
{
	fp32 temp_set;
	fp32 temp_real;
	fp32 temp_max_set;
}Temperature;



void temperature_ADC_init(void);
void temperature_ADC_Reset(void);
fp32 get_temprate(void);
uint16_t get_ADC(uint8_t ch);
u8 get_set_temp(void);
extern Temperature temperature;



#line 43 "..\\USER\\main.h"
#line 1 "..\\HAL\\communicate.h"




#line 6 "..\\HAL\\communicate.h"
#line 1 "..\\BSP\\data_fifo.h"















 








 
 



#line 31 "..\\BSP\\data_fifo.h"


 


typedef struct
{
  uint8_t   *start_addr;                   
  uint8_t   *end_addr;                     
  uint32_t  free;                         
  uint32_t  buf_size;                     
  uint32_t  used;                         
  uint8_t   read_index;                   
  uint8_t   write_index;                  
} fifo_s_t;


fifo_s_t* fifo_s_create(uint32_t unit_cnt);
void     fifo_s_destory(fifo_s_t* pfifo);

int32_t fifo_s_init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt);

int32_t fifo_s_put(fifo_s_t* pfifo, uint8_t element);
int32_t fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint32_t number);

uint8_t  fifo_s_get(fifo_s_t* pfifo);
uint16_t fifo_s_gets(fifo_s_t* pfifo, uint8_t* source, uint8_t len);

uint8_t  fifo_s_pre_read(fifo_s_t* pfifo, uint8_t offset);
uint8_t  fifo_is_empty(fifo_s_t* pfifo);
uint8_t  fifo_is_full(fifo_s_t* pfifo);
uint32_t fifo_used_count(fifo_s_t* pfifo);
uint32_t fifo_free_count(fifo_s_t* pfifo);
uint8_t  fifo_flush(fifo_s_t* pfifo);





#line 7 "..\\HAL\\communicate.h"
#line 8 "..\\HAL\\communicate.h"



	
typedef enum				
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;		

typedef enum
{
  UART_IDLE_IT     = 0,
  UART_DMA_HALF_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef struct
{
  USART_InitTypeDef *huart;	
  fifo_s_t           *data_fifo;
  uint16_t           buff_size;
  uint8_t            *buff[2];
  uint16_t           read_index;
  uint16_t           write_index;
} uart_dma_rxdata_t;

typedef struct			
{
  fifo_s_t       *data_fifo;		
  frame_header_t *p_header;			
  uint16_t       data_len;			
  uint8_t        protocol_packet[200];		
  unpack_step_e  unpack_step;		
  uint16_t       index;					
} unpack_data_t;

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);		

 


void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof);		





void client_show_data_send(float data1, float data2, float data3, uint8_t mask);		

#line 44 "..\\USER\\main.h"
#line 1 "..\\BSP\\judgement_info.h"



#line 5 "..\\BSP\\judgement_info.h"




typedef enum
{	

	  GAME_STATUS_DATA_ID   = 0x0001,  
		GAME_RESULT_ID   			= 0x0002,  
		ROBO_EXIST_DATA_ID 		= 0x0003,  
		COURT_EVENT_DATA_ID   = 0x0101,  
		SUPPLY_STA_ACTION_DATA = 0x0102, 
		SUPPLY_STA_REQUEST_DATA = 0x103, 
		ROBO_STATE_DATA = 0x0201,				 
		REAL_POWER_ID = 0x0202,				   
		ROBO_POS_DATA_ID  = 0x0203,			 
		ROBO_BUFF_DATA = 0x0204,				 
		SPACE_ROBO_ENERGY = 0x0205,			 
		HURT_DATA = 0x0206,							 
		REAL_SHOOT_DATA = 0x0207,				 
		ROBO_CONNECTION_DATA = 0x0301,	 
		
} judge_data_id_e;





 
typedef __packed struct {   
	uint8_t game_type : 4;      
	uint8_t game_progress : 4;   
	uint16_t stage_remain_time;  
} ext_game_state_t; 






 
typedef __packed struct 
{    
	uint8_t winner;     
} ext_game_result_t; 






















 
typedef __packed struct 
{   
	uint16_t robot_legion;   
} ext_game_robot_survivors_t; 




 
typedef __packed struct 
{   
	uint32_t event_type; 
} ext_event_data_t; 




 
typedef __packed struct 
{   
	uint8_t supply_projectile_id; 
	uint8_t supply_robot_id;    
															
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t; 




 
typedef __packed struct 
{   
	uint8_t supply_projectile_id;    
	uint8_t supply_robot_id;  
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 




 
typedef __packed struct 
{   
	uint8_t robot_id;    
	uint8_t robot_level;   
	uint16_t remain_HP;   
	uint16_t max_HP;     
	uint16_t shooter_heat0_cooling_rate;   
	uint16_t shooter_heat0_cooling_limit;   
	uint16_t shooter_heat1_cooling_rate;   
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t mains_power_gimbal_output : 1;   
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1;  
} ext_game_robot_state_t; 




 
typedef __packed struct 
{   
	uint16_t chassis_volt;    
	uint16_t chassis_current;    
	float chassis_power;        
	uint16_t chassis_power_buffer;
	uint16_t shooter_heat0;    
	uint16_t shooter_heat1;  
} ext_power_heat_data_t; 




 
typedef __packed struct 
{  
  float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 








 
typedef __packed struct 
{   
	uint8_t power_rune_buff; 
}ext_buff_musk_t; 




 
typedef __packed struct 
{   
	uint8_t energy_point;  
	uint8_t attack_time;  
} aerial_robot_energy_t; 









 
typedef __packed struct 
{   
	uint8_t armor_id : 4;   
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t;



 
typedef __packed struct 
{   
	uint8_t bullet_type;  
	uint8_t bullet_freq;    
	float bullet_speed;  
} ext_shoot_data_t; 



 














typedef __packed struct 
{ 
  uint16_t data_cmd_id;
  uint16_t send_ID;    
  uint16_t receiver_ID; 
	float data1; 
	float data2; 
	float data3;
	uint8_t masks; 
} client_custom_data_t; 












typedef struct
{

	
	ext_game_state_t      game_rstatus_data;  
	ext_game_result_t      game_result_data;   
	ext_game_robot_survivors_t robot_exis_data;    

	ext_event_data_t			court_event_data;		
	ext_supply_projectile_action_t			supply_act_data;		
	ext_supply_projectile_booking_t		supply_request_data;	

	ext_game_robot_state_t	game_information;   
	ext_power_heat_data_t		power_heat_data;    
	ext_game_robot_pos_t		robot_pos_data;     
	ext_buff_musk_t			get_buff_data;      
	aerial_robot_energy_t space_father_data;	
	ext_robot_hurt_t		blood_changed_data; 
	ext_shoot_data_t			real_shoot_data;    

  client_custom_data_t robot_connection_data; 
} receive_judge_t;

extern receive_judge_t judge_rece_mesg;		

void  judgement_data_handler(uint8_t *p_frame);	
uint8_t shooter_heat_control(void);
uint8_t chassis_heat_control(void);


#line 45 "..\\USER\\main.h"
#line 46 "..\\USER\\main.h"
#line 1 "..\\BSP\\JudgeSystem.h"




#line 6 "..\\BSP\\JudgeSystem.h"



void JudgeSystem_uart7_init(void);

extern receive_judge_t judge_rece_mesg;




#line 47 "..\\USER\\main.h"
#line 1 "..\\BSP\\led.h"
#line 4 "..\\BSP\\led.h"

void led_configuration(void);

extern void led_green_on(void);
extern void led_green_off(void);
extern void led_green_toggle(void);

extern void led_red_on(void);
extern void led_red_off(void);
extern void led_red_toggle(void);

extern void flow_led_on(uint16_t num);
extern void flow_led_off(uint16_t num);
extern void flow_led_toggle(uint16_t num);


void led_io_cinfiguration(void);
#line 48 "..\\USER\\main.h"




#line 2 "..\\HAL\\imu.c"
			

volatile packed_angle real_angle = {0};





#line 17 "..\\HAL\\imu.c"







static const uint8_t mpu6500_spi_DMA_txbuf[23] =
    {
        (0X3A) | 0x80};
	

		

void IMU_Configure()
{
	IMU_INT_Configure();
	{ SPI5_DMA_Init((uint32_t)mpu6500_spi_DMA_txbuf, (uint32_t)mpu6500_spi_rxbuf, 23); SPI_I2S_DMACmd(((SPI_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x5000)), ((uint16_t)0x0001), ENABLE); SPI_I2S_DMACmd(((SPI_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x5000)), ((uint16_t)0x0002), ENABLE); SPI5SetSpeedAndDataSize(((uint16_t)0x0010), ((uint16_t)0x0000)); };
}



void IMU_INT_Configure()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(((uint32_t)0x00000002), ENABLE);
    RCC_APB2PeriphClockCmd(((uint32_t)0x00004000), ENABLE);

    SYSCFG_EXTILineConfig(((uint8_t)0x01), ((uint8_t)0x08));

    GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0100);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x0400)), &GPIO_InitStructure);

    EXTI_InitStructure.EXTI_Line = ((uint32_t)0x00100);
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void IMU_EXIT_STOP()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void IMU_EXIT_START()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(((uint32_t)0x00100)) != RESET)
    {

        EXTI_ClearITPendingBit(((uint32_t)0x00100));


        mpu6500_SPI_NS_L();
        SPI5_DMA_Enable(23);

    }
}

void DMA2_Stream5_IRQHandler(void)
{
    if (DMA_GetFlagStatus(((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6400) + 0x088)), ((uint32_t)0x20000800)))
    {
        DMA_ClearFlag(((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6400) + 0x088)), ((uint32_t)0x20000800));
        mpu6500_SPI_NS_H();
		
        
		real_angle.last_yaw = real_angle.new_yaw;
		real_angle.new_yaw   = INS_Angle[0] * 57.3f;
		real_angle.diff_yaw = real_angle.new_yaw - real_angle.last_yaw;
		if(real_angle.diff_yaw > 180.0f)
				real_angle.yaw_counts--;
		else if(real_angle.diff_yaw < -180.0f)
			real_angle.yaw_counts++;
		real_angle.yaw = real_angle.new_yaw + real_angle.yaw_counts * 360.0f;
		real_angle.pitch = -INS_Angle[2] * 57.3f;
		real_angle.roll  = INS_Angle[1] * 57.3f;
		real_angle.gx = -mpu6500_real_data.gyro[1] * 57.3f;
		real_angle.gy = mpu6500_real_data.gyro[0] * 57.3f;
		real_angle.gz = mpu6500_real_data.gyro[2] * 57.3f;
    }
}
u8 run_flag = 0;
u8 offset_OK_flag = 0;
u8 offset_OK_flag1 = 0;

void IMU_control_decide()
{
	
	while(1)
	{
		if(rc.sr == 1 )
		{
			run_flag = 2;
			offset_OK_flag = 2;
			offset_OK_flag1 = 2;
			break;
		}    
		else if(rc.sr == 2 )
		{
			run_flag = 1;
			offset_OK_flag = 2;
			offset_OK_flag1 = 2;
			break;
		}
		else
		{
			run_flag = 0;
			offset_OK_flag = 0;
			offset_OK_flag1 = 0;
		}
	}
}
