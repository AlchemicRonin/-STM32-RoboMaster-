#ifndef MPU6500DRIVERMIDDLEWARE_H
#define MPU6500DRIVERMIDDLEWARE_H

#include "stm32f4xx.h"

#define MPU6500_USE_SPI
//to do list ��δ����IICͨ��
//#define MPU6500_USE_IIC

#define MPU6500_GYRO_RANGE_1000
#define MPU6500_ACCEL_RANGE_2G

//mpu6500 ��ʼ��
extern void mpu6500_GPIO_init(void);
extern void mpu6500_com_init(void);
//mpu6500 ��ʱ����
extern void mpu6500_middleware_delay_ms(uint16_t ms);
extern void mpu6500_middleware_delay_us(uint32_t us);

#if defined(MPU6500_USE_SPI)

//mpu6500 SPI��ȡ�Ĵ���ʱ����Ҫ�ڼĴ�����ַ��� ��1
#define MPU_SPI_READ_MSB 0x80

//������SPI NS �ߵ͵�ƽ
extern void mpu6500_SPI_NS_H(void);
extern void mpu6500_SPI_NS_L(void);

//�����Ƕ�ȡ��д��Ĵ�����ַ������
extern void mpu6500_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t mpu6500_read_single_reg(uint8_t reg);
extern void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(MPU6500_USE_IIC)

#endif

#endif
