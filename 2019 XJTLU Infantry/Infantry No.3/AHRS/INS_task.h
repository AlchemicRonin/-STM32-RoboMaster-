	
#ifndef INS_TASK_H
#define INS_TASK_H

#include "stm32f4xx.h"
#include "mpu6500driver.h"

typedef float fp32;


#define DMA_RX_NUM 23
#define MPU6500_RX_BUF_DATA_OFFSET 1
#define IST8310_RX_BUF_DATA_OFFSET 16

#define MPU6500_TEMPERATURE_PWM_INIT() TIM3_Init(MPU6500_TEMP_PWM_MAX, 1) //�������¶ȿ���PWM��ʼ��

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //�¶ȿ���PID��ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //�¶ȿ���PID��max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //�¶ȿ���PID��max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1


#define INS_DELTA_TICK 1 //������õļ��

#define INS_TASK_INIT_TIME 7 //����ʼ���� delay һ��ʱ��


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
extern uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //������յ�ԭʼ����
extern mpu6500_real_data_t mpu6500_real_data; //ת���ɹ��ʵ�λ��MPU6500����


#endif
