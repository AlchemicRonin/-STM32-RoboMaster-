#ifndef MPU6500DRIVERMIDDLEWARE_H
#define MPU6500DRIVERMIDDLEWARE_H

#include "main.h"

#define MPU6500_USE_SPI
//to do list 尚未适配IIC通信
//#define MPU6500_USE_IIC

#define MPU6500_GYRO_RANGE_1000
#define MPU6500_ACCEL_RANGE_2G

//mpu6500 初始化
extern void mpu6500_GPIO_init(void);
extern void mpu6500_com_init(void);
//mpu6500 延时函数
extern void mpu6500_middleware_delay_ms(uint16_t ms);
extern void mpu6500_middleware_delay_us(uint32_t us);

#if defined(MPU6500_USE_SPI)

//mpu6500 SPI读取寄存器时候，需要在寄存器地址最高 置1
#define MPU_SPI_READ_MSB 0x80

//陀螺仪SPI NS 高低电平
extern void mpu6500_SPI_NS_H(void);
extern void mpu6500_SPI_NS_L(void);

//陀螺仪读取，写入寄存器地址的数据
extern void mpu6500_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t mpu6500_read_single_reg(uint8_t reg);
extern void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(MPU6500_USE_IIC)

#endif

#endif
