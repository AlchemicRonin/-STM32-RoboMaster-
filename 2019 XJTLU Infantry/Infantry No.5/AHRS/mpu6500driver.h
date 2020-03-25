#ifndef MPU6500DRIVER_H
#define MPU6500DRIVER_H

#include "stm32f4xx.h"

#define MPU6500

#define GYRO_OFFSET_KP 0.0003f //����������Ե���������У׼�ٶȣ�Խ��������У׼�仯Խ�죬����������

#define MPU_DATA_READY_BIT 0 //����������׼��
#define MPU_MOT_BIT 1        //mpu6500 �˶����

//�޴���
#define MPU6500_NO_ERROR 0x00
//����Ϊ�����룬
#define NO_Sensor 0x80
#define PWR_MGMT_1_ERROR 0x01
#define PWR_MGMT_2_ERROR 0x02
#define SMPLRT_DIV_ERROR 0x03
#define CONFIG_ERROR 0x04
#define INTBP_CFG_ERROR 0x05
#define GYRO_CONFIG_ERROR 0x06
#define ACCEL_CONFIG_ERROR 0x07
#define ACCEL_CONFIG_2_ERROR 0x08
#define I2C_MST_CTRL_ERROR 0x09
#define USER_CTRL_ERROR 0x0A
#define INT_ENABLE_ERROR 0x0B
#define I2C_MST_DELAY_CTRL_ERROR 0x0C
#define MOT_DETECT_CTRL_ERROR 0x0D
#define WOM_THR_ERROR 0x0E

//�����ǿ���У׼��ʱ��
#define GYRO_OFFSET_START_TIME 500

//���¶�������mpu6500 ���ݷ�Χ����Ҫ���ĸ���Χ��ȡ��ע�ͣ���������Χע�͵�
//#define MPU6500_ACCEL_RANGE_16G
//#define MPU6500_ACCEL_RANGE_8G
//#define MPU6500_ACCEL_RANGE_4G
#define MPU6500_ACCEL_RANGE_2G

//���¶�������mpu6500 ���ݷ�Χ����Ҫ���ĸ���Χ��ȡ��ע�ͣ���������Χע�͵�
//#define MPU6500_GYRO_RANGE_2000
#define MPU6500_GYRO_RANGE_1000
//#define MPU6500_GYRO_RANGE_500
//#define MPU6500_GYRO_RANGE_250

//���������ݽṹ��
typedef struct mpu6500_real_data_t
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
} mpu6500_real_data_t;

//�����ǳ�ʼ��
extern uint8_t mpu6500_init(void);
//�����Ƕ�ȡ
extern void mpu6500_read_over(uint8_t *status_buf, mpu6500_real_data_t *mpu6500_real_data);
//������У׼
extern void gyro_offset(float gyro_offset[3], float gyro[3], uint8_t imu_status, uint16_t *offset_time_count);

extern void mpu6500_read_gyro(float gyro[3]);
extern void mpu6500_read_accel(float accel[3]);
extern void mpu6500_read_temp(float *temperature);
#endif
