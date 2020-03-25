#ifndef _IO_TASK_H_
#define _IO_TASK_H_
#include "main.h"

#define VERSION_A								1u
#define VERSION_B								6u
#define VERSION_C								4u
#define VERSION_D								0u
#define VERSION									(VERSION_A<<24)|(VERSION_B<<16)|(VERSION_C<<8)|(VERSION_D)

#define PARAM_SAVED_START_ADDRESS 								ADDR_FLASH_SECTOR_11

//enum the cali result
typedef enum
{
    CALI_STATE_ERR,
    CALI_STATE_IN,
    CALI_STATE_DONE,
}CALI_STATE_e;

typedef struct Version
{
		uint8_t A;   //main version number
		uint8_t B;	//sub version number
		uint8_t C;	
		uint8_t D;	// test version number
}Version;

#define VERSION_DEFAULT	\
{\
	1,\
	6,\
	2,\
	0,\
}\

#define PARAM_SAVED_FLAG                            0x5A   //header of the structure
#define PARAM_CALI_DONE                             0x5A 		
#define PARAM_CALI_NONE                             0x00

#define CALI_START_FLAG_GYRO                  ((uint32_t)1<<1)
#define CALI_END_FLAG_GYRO                    ((uint32_t)1<<2)
#define CALI_START_FLAG_ACC                   ((uint32_t)1<<3)
#define CALI_START_FLAG_MAG                   ((uint32_t)1<<4)
#define CALI_END_FLAG_MAG                     ((uint32_t)1<<5)
#define CALI_START_FLAG_GIMBAL                ((uint32_t)1<<6)
#define CALI_END_FLAG_GIMBAL                  ((uint32_t)1<<7)
#define CALI_FLAG_PID         				  ((uint32_t)1<<8)
#define CALI_FLAG_PITCH_SPEED_PID             ((uint32_t)1<<9)
#define CALI_FLAG_YAW_POSITION_PID            ((uint32_t)1<<10)
#define CALI_FLAG_YAW_SPEED_PID               ((uint32_t)1<<11)

typedef __packed struct
{
    int16_t     GimbalYawOffset;
    int16_t     GimbalPitchOffset;
    uint8_t     GimbalCaliFlag;
}GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef __packed struct
{
    int16_t     AccXOffset;
    int16_t     AccYOffset;
    int16_t     AccZOffset; 
    float       AccXScale;
    float       AccYScale;
    float       AccZScale;
    uint8_t     AccCaliFlag;
}AccCaliStruct_t;

typedef __packed struct
{
    int16_t     MagXOffset;
    int16_t     MagYOffset;
    int16_t     MagZOffset;
    float       MagXScale;
    float       MagYScale;
    float       MagZScale;    
    uint8_t     MagCaliFlag;
}MagCaliStruct_t;

typedef __packed struct
{
	int8_t pid_type;		// position PID
	int8_t motor_type;   //motor type ie: pitch yaw 201 202 203 204	
	int16_t kp_offset;
	int16_t ki_offset;
	int16_t kd_offset;
}PIDParamStruct_t;

typedef __packed struct 
{
    uint8_t     ParamSavedFlag;    				//header 
    uint32_t    FirmwareVersion;    			//version
    GimbalCaliStruct_t GimbalCaliData;    //gimbal pitch yaw encoder offset
    GyroCaliStruct_t   GyroCaliData;      //gyro offset data
    AccCaliStruct_t    AccCaliData;    		//ACC offset data
    MagCaliStruct_t    MagCaliData;				//Mag offset data
	PIDParamStruct_t   PitchPositionPID;
	PIDParamStruct_t   PitchSpeedPID;
	PIDParamStruct_t   YawPositionPID;
	PIDParamStruct_t   YawSpeedPID;
}AppParam_t;
//上传数据的类型
typedef enum
{
	REIMU = 1,
	REMOV = 2,
	REHMC = 3,
	REOFFSET = 4,
	REVERSION = 5,
	REERROR =6,
	REPID =7,
}UploadParamType_e;

extern GimbalCaliStruct_t GimbalSavedCaliData;    //gimbal pitch yaw encoder offset
extern GyroCaliStruct_t   GyroSavedCaliData;      //gyro offset data
extern AccCaliStruct_t    AccSavedCaliData;    		//ACC offset data
extern MagCaliStruct_t    MagSavedCaliData;				//Mag offset data

extern PIDParamStruct_t PitchPostionCaliData;  //保存pitch轴position校准值
extern PIDParamStruct_t PitchSpeedCaliData;  //保存pitch轴position校准值
extern PIDParamStruct_t YawPositionCaliData;  //保存pitch轴position校准值
extern PIDParamStruct_t YawSpeedCaliData;  //保存pitch轴position校准值

extern PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
extern PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
extern PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
extern PIDParamStruct_t YawSpeedSavedPID;        	//PID offset data

extern AppParam_t gAppParamStruct;
void AppParamInit(void);

void GetGimbalCaliData(GimbalCaliStruct_t *cali_data);
void GetGyroCaliData(GyroCaliStruct_t *cali_data);
void GetAccCaliData(AccCaliStruct_t *cali_data);
void GetMagCaliData(MagCaliStruct_t *cali_data);

//读取calidata保存到gAppParamStruct中，并写入到Flash中
void SetGimbalCaliData(GimbalCaliStruct_t *cali_data);
void SetGyroCaliData(GyroCaliStruct_t *cali_data);
void SetAccCaliData(AccCaliStruct_t *cali_data);
void SetMagCaliData(MagCaliStruct_t *cali_data);
CALI_STATE_e PIDCaliProcess(PIDParamStruct_t *cali_data);
//set or reset the Cali Cmd flag

void Sensor_Offset_Param_Init(AppParam_t *appParam);
void SetCaliCmdFlag(uint32_t flag);
void ResetCaliCmdFlag(uint32_t flag);
uint32_t GetCaliCmdFlagGrp(void);
uint8_t IsCaliCmdFlagSet(uint32_t flag);
CALI_STATE_e GimbalCaliProcess(void);
CALI_STATE_e GyroCaliProcess(void);
CALI_STATE_e MagStartCaliProcess(void);
CALI_STATE_e MagEndCaliProcess(void);
void CalibrateLoop(void);
uint8_t IsGimbalCalied(void);
uint8_t IsGyroCalied(void);
uint8_t IsAccCalied(void);
uint8_t IsMagCalied(void);
uint8_t Is_AppParam_Calied(void);


#endif

