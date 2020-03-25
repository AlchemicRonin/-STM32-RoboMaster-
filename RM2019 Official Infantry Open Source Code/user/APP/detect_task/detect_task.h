/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief      设备离线判断任务，通过freeRTOS滴答时间作为系统时间，设备获取数据后
  *             调用DetectHook记录对应设备的时间，在该任务会通过判断记录时间与系统
  *             时间之差来判断掉线，同时将最高的优先级的任务通过LED的方式改变，包括
  *             八个流水灯显示SBUB遥控器，三个云台上的电机，4个底盘电机，另外也通过
  *             红灯闪烁次数来显示错误码。
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
#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"
//错误码以及对应设备顺序
enum errorList
{
    DBUSTOE = 0,
    YawGimbalMotorTOE,
    PitchGimbalMotorTOE,
    TriggerMotorTOE,
    ChassisMotor1TOE,
    ChassisMotor2TOE,
    ChassisMotor3TOE,
    ChassisMotor4TOE,

    errorListLength,
};

typedef __packed struct
{
    uint32_t newTime;
    uint32_t lastTime;
    uint32_t Losttime;
    uint32_t worktime;
    uint16_t setOfflineTime : 12;
    uint16_t setOnlineTime : 12;
    uint8_t enable : 1;
    uint8_t Priority : 4;
    uint8_t errorExist : 1;
    uint8_t isLost : 1;
    uint8_t dataIsError : 1;

    fp32 frequency;
    bool_t (*dataIsErrorFun)(void);
    void (*solveLostFun)(void);
    void (*solveDataErrorFun)(void);
} error_t;

extern bool_t toe_is_error(uint8_t err);
extern void DetectTask(void *pvParameters);
void DetectHook(uint8_t toe);
extern const error_t *getErrorListPoint(void);
extern void DetectHook(uint8_t toe);
#endif
