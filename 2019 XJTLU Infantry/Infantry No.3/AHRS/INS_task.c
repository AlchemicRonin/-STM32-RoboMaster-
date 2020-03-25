#include "main.h"

#define USE_ACCEL_OFFSET 0



#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))


#define GYRO_CALIBRATE_TIME 20000
#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \

//处理陀螺仪，加速度计，磁力计数据的线性度，零漂

#define IMUTempPWM(pwm) TIM_SetCompare2(TIM3, (pwm))                      //pwm给定
uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //保存接收的原始数据
mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据
static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
static fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
extern u8 bsp_flag;

static ist8310_real_data_t ist8310_real_data;                //转换成国际单位的IST8310数据
static fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //陀螺仪零漂
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
static fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //加速度零漂

static fp32 Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                      {0.0f, 1.0f, 0.0f},
                                      {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
static fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂
static const float TimingTime = INS_DELTA_TICK * 0.001f;   //任务运行的时间 单位 s

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
float PID_INCR_TEMP[3]    = {8000.00f, 0.5f, 0.0f};
fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数
static void IMU_temp_Control(fp32 temp);
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);

static uint8_t first_temperate = 0;
extern u8 offset_OK_flag1;

void INSTask()
{
				
        //将读取到的mpu6500原始数据处理成国际单位的数据
        mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);
//				ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
        //减去零漂以及旋转坐标系
        IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);


        //加速度计低通滤波
        static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
        static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


        //判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
        static uint8_t updata_count = 0;

        if( mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)
        {

            if (updata_count == 0)
            {
                MPU6500_TEMPERATURE_PWM_INIT();
				pid_incr_init(&pid_incr[TEMP], PID_INCR_TEMP, 4999.0f, 4400.0f);
                //初始化四元数		
                AHRS_init(INS_quat, INS_accel, INS_mag);
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

                accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
                accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
                accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
                updata_count++;
            }
            else
            {
                //加速度计低通滤波
                accel_fliter_1[0] = accel_fliter_2[0];
                accel_fliter_2[0] = accel_fliter_3[0];

                accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

                accel_fliter_1[1] = accel_fliter_2[1];
                accel_fliter_2[1] = accel_fliter_3[1];

                accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

                accel_fliter_1[2] = accel_fliter_2[2];
                accel_fliter_2[2] = accel_fliter_3[2];

                accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

							
                //更新四元数
                AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

                //陀螺仪开机校准
//								if(bsp_flag)
                {
                    static uint16_t start_gyro_cali_time = 0;
                    if(start_gyro_cali_time == 0)
                    {		
					led_red_on();
                        Gyro_Offset[0] = gyro_cali_offset[0];
                        Gyro_Offset[1] = gyro_cali_offset[1];
                        Gyro_Offset[2] = gyro_cali_offset[2];
                        start_gyro_cali_time++;
                    }
					if(first_temperate)
					{	
						offset_OK_flag1 = 1;
						 led_red_off();
					}
//                    else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
//                    {
//											
//                        IMUWarnBuzzerOn();
//                        if(first_temperate)
//                        {
//                            //当进入gyro_offset函数，如果无运动start_gyro_cali_time++，如果有运动 start_gyro_cali_time = 0
//                            gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
//                        }
//                    }
//                    else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
//                    {
//						
//                       
//                        start_gyro_cali_time++;
//                    }
                }       //陀螺仪开机校准   code end

            }           //update count if   code end
        }               //mpu6500 status  if end
				IMU_temp_Control(mpu6500_real_data.temp);
}

/**
  * @brief          校准陀螺仪
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[in]      陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         返回空
  */

void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (first_temperate)
    {
        if( *time_count == 0)
        {
            Gyro_Offset[0] = gyro_cali_offset[0];
            Gyro_Offset[1] = gyro_cali_offset[1];
            Gyro_Offset[2] = gyro_cali_offset[2];
        }
        gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, time_count);

        cali_offset[0] = Gyro_Offset[0];
        cali_offset[1] = Gyro_Offset[1];
        cali_offset[2] = Gyro_Offset[2];
    }
}

/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         返回空
  */

int16_t ins_set_counts = 0;
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[6])
{
	ins_set_counts++;
	
//		if()
    gyro_cali_offset[0]  = cali_offset[0];
    gyro_cali_offset[1]  = cali_offset[1];
    gyro_cali_offset[2]  = cali_offset[2];
	
	
//下面第一次校准用
//	gyro_cali_offset[0]  = 0;
//    gyro_cali_offset[1]  = 0;
//    gyro_cali_offset[2]  = 0;
}

const fp32 *get_INS_angle_point(void)
{
    return INS_Angle;
}
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
    }
}

extern u8 offset_OK_flag;
static uint16_t count_time = 0;

int8_t cali_gyro_hook(void)
{
	imu_cali_t local_cali_t;

	INS_cali_gyro(local_cali_t.scale, local_cali_t.offset, &count_time);
	if (count_time > GYRO_CALIBRATE_TIME)
	{
		count_time = 0;
		OFFSET_Buffer[0] = local_cali_t.offset[0];
		OFFSET_Buffer[1] = local_cali_t.offset[1];
		OFFSET_Buffer[2] = local_cali_t.offset[2];
		flash_write();
		led_red_off();
		offset_OK_flag = 1;
		return 1;
	}
	else
	{
		led_red_on();
		return 0;
	}
	
}

static void IMU_temp_Control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0 ;
    if (first_temperate)
    {
		out_incr[TEMP] = pid_incr_calc(&pid_incr[TEMP], temp, temperature.temp_set);
        if (out_incr[TEMP] < 0.0f)
        {
            out_incr[TEMP] = 0.0f;
        }
        tempPWM = (uint16_t)out_incr[TEMP];
        IMUTempPWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > temperature.temp_set)
        {
            temp_constant_time ++;
            if(temp_constant_time > 10)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
				pid[TEMP].iout = MPU6500_TEMP_PWM_MAX / 12.0f;

            }
        }

        IMUTempPWM(MPU6500_TEMP_PWM_MAX - 1);
    }
//		static uint8_t temp_constant_time = 0 ;
//		out[TEMP] = Calculate_Current_Value(&pid[TEMP], temperature.temp_set, temp);	
//		TIM3->CCR2 = out[TEMP];
//		if (temp > temperature.temp_set)
//		{
//				temp_constant_time++;
//				if(temp_constant_time > 200)
//				{
//					first_temperate = 1;
//				}
//		}
}

