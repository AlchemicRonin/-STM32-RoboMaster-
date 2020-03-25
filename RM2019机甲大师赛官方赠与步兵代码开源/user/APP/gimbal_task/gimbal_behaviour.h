#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_INIT,           //云台初始化
  GIMBAL_CALI,           //云台校准
  GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
  GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
  GIMBAL_MOTIONLESS,     //云台在遥控器无输入一段时间后保持不动，避免陀螺仪漂移
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern bool_t gimbal_cmd_to_shoot_stop(void);
#endif
