	
#ifndef INS_TASK_H
#define INS_TASK_H

#include "stm32f4xx.h"
#include "mpu6500driver.h"

typedef float fp32;


#define DMA_RX_NUM 23
#define MPU6500_RX_BUF_DATA_OFFSET 1
#define IST8310_RX_BUF_DATA_OFFSET 16

#define MPU6500_TEMPERATURE_PWM_INIT() TIM3_Init(MPU6500_TEMP_PWM_MAX, 1) //陀螺仪温度控制PWM初始化

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //温度控制PID的max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


#define INS_DELTA_TICK 1 //任务调用的间隔

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间


typedef struct 
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

int8_t cali_gyro_hook(void);
extern void INSTask(void);

extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[6]);
extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);
extern fp32 INS_Angle[3];
extern uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //保存接收的原始数据
extern mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据


#endif
