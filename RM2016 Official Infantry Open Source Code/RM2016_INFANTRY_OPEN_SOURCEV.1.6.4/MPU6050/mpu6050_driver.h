#ifndef __MPU6050_DRIVER_H__
#define __MPU6050_DRIVER_H__
#include "main.h"

//����MPU6050�ڲ���ַ
#define	SMPLRT_DIV		          0x19	//�����ǲ����� ����ֵ 0X07 125Hz
#define	CONFIG			            0x1A	//��ͨ�˲�Ƶ�� ����ֵ 0x00 
#define	GYRO_CONFIG		          0x1B	//�������Լ켰������Χ                 ����ֵ 0x18 ���Լ� 2000deg/s
#define	ACCEL_CONFIG	          0x1C	//���ٶȼ��Լ켰������Χ����ͨ�˲�Ƶ�� ����ֵ 0x01 ���Լ� 2G 5Hz
#define INT_PIN_CFG               0x37
#define INT_ENABLE                0x38
#define INT_STATUS                0x3A    //ֻ��
#define	ACCEL_XOUT_H	          0x3B
#define	ACCEL_XOUT_L	          0x3C
#define	ACCEL_YOUT_H	          0x3D
#define	ACCEL_YOUT_L	          0x3E
#define	ACCEL_ZOUT_H	          0x3F
#define	ACCEL_ZOUT_L	          0x40
#define	TEMP_OUT_H		          0x41
#define	TEMP_OUT_L		          0x42
#define	GYRO_XOUT_H		          0x43
#define	GYRO_XOUT_L		          0x44	
#define	GYRO_YOUT_H		          0x45
#define	GYRO_YOUT_L		          0x46
#define	GYRO_ZOUT_H		          0x47
#define	GYRO_ZOUT_L		          0x48
#define	PWR_MGMT_1		          0x6B	//��Դ���� ����ֵ 0x00 ��������
#define	WHO_AM_I		            0x75	//ֻ��  Ĭ�϶���Ӧ���� MPU6050_ID = 0x68
#define MPU6050_ID                0x68
#define MPU6050_DEVICE_ADDRESS    0xD0


#define MPU6050_DATA_START        ACCEL_XOUT_H   //�������ݴ�ŵ�ַ�������ģ�����һ������
#define MPU6050_RA_SELF_TEST_X                  0x0D
#define MPU6050_RA_SLEF_TEST_Y                  0x0E
#define MPU6050_RA_SELF_TEST_Z                  0x0F
#define MPU6050_RA_SELF_TEST_A                  0x10
#define MPU6050_RA_SMPLRT_DIV                   0x19
#define MPU6050_RA_CONFIG                       0x1A
#define MPU6050_RA_GYRO_CONFIG                  0x1B
#define MPU6050_RA_ACCEL_CONFIG                 0x1C
#define MPU6050_RA_FIFO_EN                      0x23
#define MPU6050_RA_I2C_MST_CTRL                 0x24
#define MPU6050_RA_I2C_MST_STATUS               0x36
#define MPU6050_RA_INT_PIN_CFG                  0x37
#define MPU6050_RA_INT_ENABLE                   0x38
#define MPU6050_RA_INT_STATUS                   0x3A
#define MPU6050_RA_ACCEL_XOUT_H                 0x3B
#define MPU6050_RA_ACCEL_XOUT_L                 0x3C
#define MPU6050_RA_ACCEL_YOUT_H                 0x3D
#define MPU6050_RA_ACCEL_YOUT_L                 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H                 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L                 0x40
#define MPU6050_RA_TEMP_OUT_H                   0x41 
#define MPU6050_RA_TEMP_OUT_L                   0x42
#define MPU6050_RA_GYRO_XOUT_H                  0x43
#define MPU6050_RA_GYRO_XOUT_L                  0x44
#define MPU6050_RA_GYRO_YOUT_H                  0x45
#define MPU6050_RA_GYRO_YOUT_L                  0x46
#define MPU6050_RA_GYRO_ZOUT_H                  0x47
#define MPU6050_RA_GYRO_ZOUT_L                  0x48       
#define MPU6050_RA_I2C_MST_DELAY_CTRL           0x67
#define MPU6050_RA_SIGNAL_PATH_RESET            0x68
#define MPU6050_RA_USER_CTRL                    0x6A
#define MPU6050_RA_PWR_MGMT_1                   0x6B
#define MPU6050_RA_PWR_MGMT_2                   0x6C
#define MPU6050_RA_FIFO_COUNTH                  0x72
#define MPU6050_RA_FIFO_COUNTL                  0x73
#define MPU6050_RA_FIFO_R_W                     0x74
#define MPU6050_RA_WHO_AM_I                     0x75 
#define MPU6050_RA_SLV0_ADDR                    0x25
#define MPU6050_RA_SLV0_REG                     0x26
#define MPU6050_RA_SLV0_CTRL                    0x27 
#define MPU6050_RA_SLV0_DO                      0x63 
#define MPU6050_RA_SLV1_ADDR                    0x28
#define MPU6050_RA_SLV1_REG                     0x29
#define MPU6050_RA_SLV1_CTRL                    0x2A
#define MPU6050_RA_SLV1_DO                      0x64 
#define MPU6050_RA_SLV4_CTRL                    0x34 
#define MPU6050_RA_ES_DATA00                    0x49
#define MPU6050_RA_ES_DATA01                    0x4A
#define MPU6050_RA_ES_DATA02                    0x4B
#define MPU6050_RA_ES_DATA03                    0x4C
#define MPU6050_RA_ES_DATA04                    0x4D
#define MPU6050_RA_ES_DATA05                    0x4E
#define MPU6050_RA_ES_DATA06                    0x4F
#define MPU6050_RA_ES_DATA07                    0x50

/*Magnetometer HMC5883 register address */

#define HMC5883_ADDRESS                          0x3C 

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12


/*Sensitivities */
  
#define GYRO_SENSITIVITY                        32.8        //LSB/(deg/s)    +-2000    16-bit ADC   16.4
                                                            //LSB/(deg/s)    +-1000    16-bit ADC   32.8
#define GYRO_SENSITIVITY_RECIP                  0.0305      //65.5           +-500 
                                                             
#define ACC_SENSITIVITY                         8192.0   //LSB/g          +-4g      16-bit ADC
                                                         //4096           +-8g
#define ACC_SENSITIVITY_RECIP                   1.22e-4

#define MAG_SENSITIVITY                         0.3      //uT/LSB         +-1200    13-bit ADC   


/*MPU6050 ID value*/

#define MPU6050_DEVICE_ID                       0x68
#define HMC5883_DEVICE_ID_A                     0x48


typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
    short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
    short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	  short Mag_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Mag_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Mag_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    float Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
	  float Mag_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Mag_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Mag_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
	
}MPU6050_REAL_DATA;

//define the eluer angle
typedef struct AHRS
{
	float pitch;
	float roll;
	float yaw;
	
}AHRS;
extern AHRS ahrs;

//the max and min data of the mag
typedef __packed struct
{
	int16_t MaxMagX;
	int16_t MaxMagY;
	int16_t MaxMagZ;
	int16_t MinMagX;
	int16_t MinMagY;
	int16_t MinMagZ;
}MagMaxMinData_t;
extern MagMaxMinData_t MagMaxMinData;

#define GYRO_CALI_FLAG 	  		(1<<0)
#define HMC5883_CALI_FLAG 		(1<<1)
#define ENCODER_CALI_FLAG 		(1<<2)
#define ALL_SENSOR_CALI_FLAG	(GYRO_CALI_FLAG|HMC5883_CALI_FLAG|ENCODER_CALI_FLAG)

extern volatile MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
extern uint8_t mpu_buf[20];
extern int16_t  HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,HMC5883_minx,HMC5883_miny,HMC5883_minz;

int MPU6050_Init(void);
uint8_t HMC5883_Init(void);
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void MPU6050_Initialize(void);
void MPU6050_InitGyro_Offset_Start(void);
void MPU6050_InitGyro_Offset_Save(void);
void Init_Encoder_Offset_Start(void);
void Init_Encoder_Offset_Save(void);
int MPU6050_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t * Data, uint8_t Num);
int MPU6050_EnableInt(void);
void HMC58X3_mgetValues(volatile float *arry);
void HMC58X3_ReadData(u8 *vbuff);
void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz);
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z);
void HMC5883L_Start_Calib(void);
void HMC5883L_Save_Calib(void);
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void Reset_MAG_MinMaxVal(void);

#endif


