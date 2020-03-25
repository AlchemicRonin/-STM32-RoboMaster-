#ifndef __SENSOR_H__
#define __SENSOR_H__

#define  Read_Sensor_1()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)
#define  Read_Sensor_2()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2)
#define  Read_Sensor_3()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)
#define  Read_Sensor_4()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)

#define  Read_Sensor_5()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)
#define  Read_Sensor_6()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)
#define  Read_Sensor_7()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)
#define  Read_Sensor_8()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)

#define  Read_Sensor_9()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)
#define  Read_Sensor_10()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)
#define  Read_Sensor_11()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)
#define  Read_Sensor_12()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)

#define  Read_Sensor_13()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)
#define  Read_Sensor_14()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)
#define  Read_Sensor_15()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)
#define  Read_Sensor_16()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)

void Sensor_Configuration(void);

#endif 
