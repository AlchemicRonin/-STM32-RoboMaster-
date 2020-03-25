/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       mpu6500driver.c/h
  * @brief      MPU6500驱动层，完成MPU6500的初始化，以及处理数据函数，校准函数以及
  *             读取函数。
  * @note       mpu6500支持IIC和SPI
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "mpu6500driver.h"
#include "mpu6500driver_middleware.h"
#include "mpu6500reg.h"

#if defined(MPU6050)
#define DEVICE_ID MPU6050_ID
#elif defined(MPU6500)
#define DEVICE_ID MPU6500_ID
#elif defined(MPU9250)
#define DEVICE_ID MPU9250_ID
#elif defined(MPU9255)
#define DEVICE_ID MPU9255_ID
#endif

#ifndef NULL
#define NULL 0
#endif

//  陀螺仪原始数据转换成rad/s 陀螺仪范围可以在h文件中修改
#ifdef MPU6500_GYRO_RANGE_2000
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_2000
#define GYRO_SEN 0.00106526443603169529841533860381f

#elif defined(MPU6500_GYRO_RANGE_1000)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_1000
#define GYRO_SEN 0.0005326322180158476492076f

#elif defined(MPU6500_GYRO_RANGE_500)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_500
#define GYRO_SEN 0.0002663161090079238246038f

#elif defined(MPU6500_GYRO_RANGE_250)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_250
#define GYRO_SEN 0.000133158054503961923019f

#else
#error "Please set the right range of gyro (2000 , 1000, 500 or 250)"
#endif

//  加速度计原始数据转换成m/s2 加速度计范围可以在h文件中修改
#ifdef MPU6500_ACCEL_RANGE_2G
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_2G
#define ACCEL_SEN 0.00059814453125f

#elif defined(MPU6500_ACCEL_RANGE_4G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_4G
#define ACCEL_SEN 0.0011962890625f

#elif defined(MPU6500_ACCEL_RANGE_8G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_8G
#define ACCEL_SEN 0.002392578125f

#elif defined(MPU6500_ACCEL_RANGE_4G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_4G
#define ACCEL_SEN 0.00478515625f

#else
#error "Please set the right range of accel (16G , 8G, 4G or 2G)"
#define

#endif

#define mpu6500_delay_ms(ms) mpu6500_middleware_delay_ms(ms)
#define mpu6500_delay_us(us) mpu6500_middleware_delay_us(us)

// 0x01 means 4 mg, 0xFF means 1020mg, set 0x90 means 144mg
#define WOM_THR_Set 0x0F

#define MPU6500_TEMPERATURE_FACTOR 0.002f
#define MPU6500_TEMPERATURE_OFFSET 23.0f

#define MPU6500_Write_Reg_Num 14

static const uint8_t write_mpu6500_reg_data_error[MPU6500_Write_Reg_Num][3] =
    {
        {MPU_PWR_MGMT_1,
         (((~MPU_DEVICE_RESET) & (~MPU_SLEEP) & (~MPU_CYCLE) & (~MPU_GYRO_STANDBY) & (~MPU_TEMP_DISABLE)) & (MPU_CLKSEL_INTERNAL)),
         PWR_MGMT_1_ERROR},

        {MPU_PWR_MGMT_2,
         (((~MPU_DISABLE_XA) & (~MPU_DISABLE_YA) & (~MPU_DISABLE_ZA) & (~MPU_DISABLE_XG) & (~MPU_DISABLE_YG) & (~MPU_DISABLE_ZG)) & (MPU_LP_WAKE_1_25_HZ)),
         PWR_MGMT_2_ERROR},

        {MPU_SMPLRT_DIV,
         MPU_SMPLRT_DIV_1,
         SMPLRT_DIV_ERROR},

        {MPU_CONFIG,
         ((~MPU_FIFO_MODE_OFF_REPLACE_OLD_DATA) & (MPU_EXT_SYNC_DISABLE | MPU_DLPF_CFG_2_SET)),
         CONFIG_ERROR},

        {MPU_GYRO_CONFIG,
         (((~MPU_XG_SELF_TEST_SET) & (~MPU_YG_SELF_TEST_SET) & (~MPU_ZG_SELF_TEST_SET)) & (MPU_GYRO_RANGLE)),
         GYRO_CONFIG_ERROR},

        {MPU_ACCEL_CONFIG,
         (((~MPU_XA_SELF_TEST_SET) & (~MPU_YA_SELF_TEST_SET) & (~MPU_ZA_SELF_TEST_SET)) & (MPU_ACCEL_RANGLE)),
         ACCEL_CONFIG_ERROR},

        {MPU_ACCEL_CONFIG_2,
         (MPU_ACCEL_FCHOICE_B_0_SET | MPU_A_DLPL_CFG_0_SET),
         ACCEL_CONFIG_2_ERROR},

        {MPU_WOM_THR,
         WOM_THR_Set,
         MOT_DETECT_CTRL_ERROR},

        {MPU_I2C_MST_CTRL,
         (((~MPU_MULT_MST_EN) & (~MPU_SLV_3_FIFO_EN) & (~MPU_I2C_MST_P_NSR)) & (MPU_WAIT_FOR_ES_EN | MPU_I2C_MST_CLK_400_KHZ)),
         I2C_MST_CTRL_ERROR},

        {MPU_INTBP_CFG,
         ((~MPU_INTBP_ACTL) & (~MPU_INTBP_OPEN) & (~MPU_LATCH_INT_EN) & (~MPU_INT_ANYRD_2CLEAR) & (~MPU_ACTL_FSYNC) & (~MPU_FSYNC_INT_MODE_EN)) & (MPU_BYPASS_EN),
         INTBP_CFG_ERROR},

        {MPU_INT_ENABLE,
         (((~MPU_FIFO_OVERFLOW_EN) & (~MPU_FSYNC_INT_EN)) & (MPU_RAW_RDY_EN )),
         INT_ENABLE_ERROR},

        {MPU_I2C_MST_DELAY_CTRL,
         MPU_I2C_SLV0_DLY_EN,
         I2C_MST_DELAY_CTRL_ERROR},

        {MPU_MOT_DETECT_CTRL,
         MPU_ACCEL_INTEL_EN | MPU_ACCEL_INTEL_MODE_COMPARE,
         MOT_DETECT_CTRL_ERROR},

        {MPU_USER_CTRL,
         (((~MPU_DMP_EN) & (~MPU_FIFO_MODE_EN) & (~MPU_DMP_RST) & (~MPU_FIFO_RST) & (~MPU_I2C_MST_RST) & (~MPU_SIG_COND_RST)) & (MPU_I2C_MST_EN | MPU_I2C_IF_DIS)),
         USER_CTRL_ERROR},

};

uint8_t mpu6500_init(void)
{
    uint8_t res = 0;
    uint8_t wait_time = 150;
    uint8_t sleepTime = 50;

    uint8_t writeNum = 0;

    //mpu6500 gpio init
    mpu6500_GPIO_init();
    // mpu6500 com init
    mpu6500_com_init();

    //check commiunication is normal
    mpu6500_read_single_reg(MPU_WHO_AM_I);
    mpu6500_delay_us(wait_time);
    mpu6500_read_single_reg(MPU_WHO_AM_I);
    mpu6500_delay_us(wait_time);

    mpu6500_write_single_reg(MPU_PWR_MGMT_1, MPU_DEVICE_RESET);
    mpu6500_delay_ms(sleepTime);

    //check commiunication is normal after reset
    mpu6500_read_single_reg(MPU_WHO_AM_I);
    mpu6500_delay_us(wait_time);
    mpu6500_read_single_reg(MPU_WHO_AM_I);
    mpu6500_delay_us(wait_time);

    //read the register "WHO AM I"
    res = mpu6500_read_single_reg(MPU_WHO_AM_I);
    mpu6500_delay_us(wait_time);
    if (res != DEVICE_ID)
    {
        return NO_Sensor;
    }

    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < MPU6500_Write_Reg_Num; writeNum++)
    {
        mpu6500_write_single_reg(write_mpu6500_reg_data_error[writeNum][0], write_mpu6500_reg_data_error[writeNum][1]);
        mpu6500_delay_us(wait_time);
        res = mpu6500_read_single_reg(write_mpu6500_reg_data_error[writeNum][0]);
        mpu6500_delay_us(wait_time);
        if (res != write_mpu6500_reg_data_error[writeNum][1])
        {
            return write_mpu6500_reg_data_error[writeNum][2];
        }
    }
    // NO error
    return MPU6500_NO_ERROR;
}

void mpu6500_read_over(uint8_t *status_buf, mpu6500_real_data_t *mpu6500_real_data)
{
    // check point null
    if (status_buf == NULL || mpu6500_real_data == NULL)
    {
        return;
    }

    if ((*status_buf) & MPU_INT_WOM_INT)
    {
        mpu6500_real_data->status |= (uint8_t)(1 << MPU_MOT_BIT);
    }
    if (mpu6500_real_data->status & (1 << MPU_MOT_BIT))
    {
        static uint8_t motion_time = 0;
        motion_time++;
        if (motion_time > 10)
        {
            motion_time = 0;
            mpu6500_real_data->status &= ~(1 << MPU_MOT_BIT);
        }
    }

    if ((*status_buf) & MPU_RAW_RDY_INT)
    {
        int16_t temp_imu_data = 0;
        mpu6500_real_data->status |= (1 << MPU_DATA_READY_BIT);

        temp_imu_data = (int16_t)((status_buf[1]) << 8) | status_buf[2];
        mpu6500_real_data->accel[0] = temp_imu_data * ACCEL_SEN;
        temp_imu_data = (int16_t)((status_buf[3]) << 8) | status_buf[4];
        mpu6500_real_data->accel[1] = temp_imu_data * ACCEL_SEN;
        temp_imu_data = (int16_t)((status_buf[5]) << 8) | status_buf[6];
        mpu6500_real_data->accel[2] = temp_imu_data * ACCEL_SEN;

        temp_imu_data = (int16_t)((status_buf[7]) << 8) | status_buf[8];
        mpu6500_real_data->temp = temp_imu_data * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;

        temp_imu_data = (int16_t)((status_buf[9]) << 8) | status_buf[10];
        mpu6500_real_data->gyro[0] = temp_imu_data * GYRO_SEN;
        temp_imu_data = (int16_t)((status_buf[11]) << 8) | status_buf[12];
        mpu6500_real_data->gyro[1] = temp_imu_data * GYRO_SEN;
        temp_imu_data = (int16_t)((status_buf[13]) << 8) | status_buf[14];
        mpu6500_real_data->gyro[2] = temp_imu_data * GYRO_SEN;
    }
}

void gyro_offset(fp32 gyro_offset[3], fp32 gyro[3], uint8_t imu_status, uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

    if (imu_status & (1 << MPU_MOT_BIT))
    {
        (*offset_time_count) = 0;
        return;
    }

    if (imu_status & (1 << MPU_DATA_READY_BIT))
    {
        gyro_offset[0] = gyro_offset[0] - GYRO_OFFSET_KP * gyro[0];
        gyro_offset[1] = gyro_offset[1] - GYRO_OFFSET_KP * gyro[1];
        gyro_offset[2] = gyro_offset[2] - GYRO_OFFSET_KP * gyro[2];
        (*offset_time_count)++;
    }
}

void mpu6500_read_gyro(fp32 gyro[3])
{
    uint8_t buf[6];
    int16_t temp_imu_data = 0;
    mpu6500_read_muli_reg(MPU_GYRO_XOUT_H, buf, 6);

    temp_imu_data = (int16_t)((buf[0]) << 8) | buf[1];
    gyro[0] = temp_imu_data * GYRO_SEN;
    temp_imu_data = (int16_t)((buf[2]) << 8) | buf[3];
    gyro[1] = temp_imu_data * GYRO_SEN;
    temp_imu_data = (int16_t)((buf[4]) << 8) | buf[5];
    gyro[2] = temp_imu_data * GYRO_SEN;
}

void mpu6500_read_accel(fp32 accel[3])
{
    uint8_t buf[6];
    int16_t temp_imu_data = 0;
    mpu6500_read_muli_reg(MPU_ACCEL_XOUT_H, buf, 6);

    temp_imu_data = (int16_t)((buf[0]) << 8) | buf[1];
    accel[0] = temp_imu_data * ACCEL_SEN;
    temp_imu_data = (int16_t)((buf[2]) << 8) | buf[3];
    accel[1] = temp_imu_data * ACCEL_SEN;
    temp_imu_data = (int16_t)((buf[4]) << 8) | buf[5];
    accel[2] = temp_imu_data * ACCEL_SEN;
}

void mpu6500_read_temp(fp32 *temperature)
{
    uint8_t buf[2];
    int16_t temp_imu_data = 0;
    mpu6500_read_muli_reg(MPU_TEMP_OUT_H, buf, 2);

    temp_imu_data = (int16_t)((buf[0]) << 8) | buf[1];

    *temperature = temp_imu_data * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;
}
