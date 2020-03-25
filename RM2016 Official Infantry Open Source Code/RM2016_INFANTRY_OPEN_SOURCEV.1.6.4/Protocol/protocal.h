#ifndef __PROTOCAL_H__
#define __PROTOCAL_H__
#include "stm32f4xx.h"
#include <stdint.h>

#define PROTOCAL_FRAME_MAX_SIZE  256
// Protocal Version
#define PROTOCAL_HEADER                                             0x5A
#define CMD_ID_SENSOR_INFO   									    0x40			//传感器数据发送ID
#define CMD_ID_IMU_INFO												0X41			//姿态结算角度发送ID
#define CMD_ID_MAG_CALI												0X42     //磁力计校准数据发送ID
#define CMD_ID_SENSOR_CALI         						            0x43			//接收校准命令ID
#define CMD_ID_UPLOAD_CALI_INFO								        0x44      //发送flash中记录的校准数据
#define CMD_ID_VERSION												0x45			//机器人固件版本号上传
#define CMD_ID_RROBOT_ERROR									       	0x46      //机器人错误代码上传
#define CMD_ID_PID_CALI												0x47      		//云台PID参数校准值
#define CMD_ID_PID_UPLOAD											0x48

/***************************************Cali_Type***********************************************/
#define GYRO_CALI_START										        0u
#define GYRO_CALI_END											    1u
#define MAG_CALI_START									            2u
#define MAG_CALI_END											    3u
#define Encoder_CALI_START								            4u
#define Encoder_CALI_END									        5u

#define PID_CALI_PITCH_POSITION                                     0
#define PID_CALI_PITCH_SPEED                                        0
#define PID_CALI_YAW_POSITION                                       0
#define PID_CALI_YAW_SPEED                                          0

/**************************************** Data Struct ****************************************/
//cmd_id = 0x40;
#pragma pack(1)
typedef struct Sensor_Data
{
	int16_t ax:16;		//加速度计
	int16_t ay:16;
	int16_t az:16;
	int16_t gx:16;		//陀螺仪
	int16_t gy:16;
	int16_t gz:16;
	int16_t mx:16;		//磁力计
	int16_t my:16;
	int16_t mz:16;
}Sensor_Data;

//cmd_id = 0x41;
#pragma pack(1)
typedef struct IMU_Angle
{
	int16_t pitch:16;		//姿态结算角度
	int16_t yaw:16;
	int16_t roll:16;
	int16_t pitch_encoder:16;     //码盘数据
	int16_t yaw_encoder:16;
	int16_t shoot_encoder:16;
}IMU_Angle;

//cmd_id = 0x42
#pragma pack(1)     //字节对齐方式
typedef struct Mag_Calc_Data     //磁力计校准数据
{
	int16_t hmc_max_x:16;
	int16_t hmc_max_y:16;
	int16_t hmc_max_z:16;
	int16_t hmc_min_x:16;
	int16_t hmc_min_y:16;
	int16_t hmc_min_z:16;
}Mag_Cali_Data;

//cmd_id 0x44
#pragma pack(1)
typedef struct Cali_Info  //校准数据
{
	int8_t error_code;   //错误代码，0：正常，1：陀螺仪错误 磁力计错误；2:编码器错误
	int16_t gx_offset:16;
	int16_t gy_offset:16;
	int16_t gz_offset:16;
	int16_t hx_offset:16;
	int16_t hy_offset:16;
	int16_t hz_offset:16;
	int16_t yaw_encoder_offset:16;
	int16_t pitch_encoder_offset:16;
}Cali_Info;

//cmd_id 0x43
#pragma pack(1)
typedef struct CALI_CMD  //校准帧中接收到的校准命令，实为其中的一个字节
{
	int8_t type;
	
}CALI_CMD;

#pragma pack(1)
typedef struct  IsCaliedInfo        //发送接收到按键后的确认校准信息
{
	int8_t error_code;
	int8_t cali_cmd;
}IsCaliedInfo;

#pragma pack(1)
typedef struct VersionInfo        //软件版本信息
{
	int8_t num[4];	
}VersionInfo;

#pragma pack(1)
typedef struct PID_OFFSET_DATA
{
	int16_t pitch_position_kp;
	int16_t pitch_position_ki;
	int16_t pitch_position_kd;
	
	int16_t pitch_speed_kp;
	int16_t pitch_speed_ki;
	int16_t pitch_speed_kd;
	
	int16_t yaw_position_kp;
	int16_t yaw_position_ki;
	int16_t yaw_position_kd;
	
	int16_t yaw_speed_kp;
	int16_t yaw_speed_ki;
	int16_t yaw_speed_kd;
}PID_OFFSET_DATA;

#define PID_TYPE_POSITION   0x00
#define PID_TYPE_SPEED 		0x01
#define PID_TYPE_CIRCUIT    0x02

#define MOTOR_TYPE_PITCH 	0x00
#define MOTOR_TYPE_YAW 		0x01
#define MOTOR_TYPE_201		0x02
#define MOTOR_TYPE_202		0x03
#define MOTOR_TYPE_203		0x04
#define MOTOR_TYPE_204		0x05

void SetFrameLength(void* head, uint32_t length);
void SetFrameCmd(void* head, uint32_t cmd);
void SetFrameType(void* head, uint32_t type);
uint32_t GetFrameCmd(void* head);
void *GetFrameDataAddress(void * buf);
uint8_t FrameUnpack(uint8_t token, uint8_t* pBuffer);
uint8_t FramePack(uint8_t* pDataIn, uint8_t len, uint8_t* pDataOut);

//发送相关函数
void UploadParameter(void);
void PID_Paremeter_Send(float pp_kp, float pp_ki,float pp_kd, float ps_kp, float ps_ki,float ps_kd, float yp_kp, float yp_ki, float yp_kd, float ys_kp, float ys_ki, float ys_kd);
void Version_Send(uint32_t ver);
void UART3_PrintBlock(uint8_t* pdata, uint8_t len);
void Robot_Error_Code_Send(uint32_t flag_grp);
void Cali_Feedback_Info_Send(uint8_t cmd_id, int8_t cali_type);
void Mag_Cali_Info_Send(int16_t max_x,int16_t max_y,int16_t max_z,int16_t min_x,int16_t min_y,int16_t min_z);
void UART3_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void Sensor_Info_Send(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,int16_t hx,int16_t hy,int16_t hz);
void UART3_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,int16_t hx,int16_t hy,int16_t hz);
void IMU_Info_Send(int16_t yaw, int16_t pitch, int16_t roll, int16_t yaw_encoder, int16_t pitch_encoder, int16_t shoot_encoder);  
void Offset_Info_Send(int8_t error_code,int16_t gx_offset,int16_t gy_offset,int16_t gz_offset, int16_t hx_offset,int16_t hy_offset,
	int16_t hz_offset,int16_t yaw_encoder_offset,int16_t pitch_encoder_offset);

#endif  //__PROTOCAL_H__
