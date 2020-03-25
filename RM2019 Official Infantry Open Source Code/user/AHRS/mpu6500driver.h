#ifndef MPU6500DRIVER_H
#define MPU6500DRIVER_H

#include "main.h"

#define MPU6500

#define GYRO_OFFSET_KP 0.0003f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大

#define MPU_DATA_READY_BIT 0 //陀螺仪数据准备
#define MPU_MOT_BIT 1        //mpu6500 运动检测

//无错误
#define MPU6500_NO_ERROR 0x00
//以下为错误码，
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

//陀螺仪开机校准的时间
#define GYRO_OFFSET_START_TIME 500

//以下定义设置mpu6500 数据范围，需要用哪个范围，取消注释，将其他范围注释掉
//#define MPU6500_ACCEL_RANGE_16G
//#define MPU6500_ACCEL_RANGE_8G
//#define MPU6500_ACCEL_RANGE_4G
#define MPU6500_ACCEL_RANGE_2G

//以下定义设置mpu6500 数据范围，需要用哪个范围，取消注释，将其他范围注释掉
//#define MPU6500_GYRO_RANGE_2000
#define MPU6500_GYRO_RANGE_1000
//#define MPU6500_GYRO_RANGE_500
//#define MPU6500_GYRO_RANGE_250

//陀螺仪数据结构体
typedef struct mpu6500_real_data_t
{
    uint8_t status;
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
} mpu6500_real_data_t;

//陀螺仪初始化
extern uint8_t mpu6500_init(void);
//陀螺仪读取
extern void mpu6500_read_over(uint8_t *status_buf, mpu6500_real_data_t *mpu6500_real_data);
//陀螺仪校准
extern void gyro_offset(fp32 gyro_offset[3], fp32 gyro[3], uint8_t imu_status, uint16_t *offset_time_count);

#endif
