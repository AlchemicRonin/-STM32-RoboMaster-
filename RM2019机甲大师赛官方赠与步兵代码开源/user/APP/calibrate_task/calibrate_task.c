/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      完成校准相关设备的数据，包括云台，陀螺仪，加速度计，磁力计
  *             云台校准主要是中值，最大最小相对角度，陀螺仪主要校准零漂
  *             加速度计和磁力计只是写好接口函数，加速度计目前没有必要校准，
  *             磁力计尚未使用在解算算法中。
  * @note       
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

//包含校准设备的名字，校准标识符，校准数据flash大小，校准命令，对应的校准数据地址
#include "calibrate_Task.h"

#include "adc.h"
#include "buzzer.h"
#include "flash.h"
#include "Remote_Control.h"
#include "INS_Task.h"
#include "string.h"
#include "gimbal_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

static void cali_data_read(void);                           //读取所有校准数据
static void cali_data_write(void);                          //写入当前校准变量的数据
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //表头校准数据函数
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //陀螺仪校准数据函数
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //云台校准数据函数

static const RC_ctrl_t *calibrate_RC; //遥控器结构体指针
static head_cali_t head_cali;         //表头校准数据
static gimbal_cali_t gimbal_cali;     //云台校准数据
static imu_cali_t gyro_cali;          //陀螺仪校准数据
static imu_cali_t accel_cali;         //加速度校准数据
static imu_cali_t mag_cali;           //磁力计校准数据

static cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; //校准设备数组，

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"}; //校准设备的名字

//校准设备对应放入结构体变量地址
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] =
    {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};

//校准设备对应放入数据大小
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

//校准设备对应的校准函数
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

//校准对应时间戳，利用freeRTOS的tick完成。
static uint32_t calibrate_systemTick;

//遥控器控制校准设备的校准
static void RC_cmd_to_calibrate(void);

void calibrate_task(void *pvParameters)
{
    static uint8_t i = 0;
    calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {

        //遥控器操作校准步骤
        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            //校准命令为1 表示需要校准
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {
                    //调用校准函数
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //校准完成
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];

                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //写入flash
                        cali_data_write();
                    }
                }
            }
        }
        vTaskDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
//返回mpu6500需要控制到的温度
int8_t get_control_temperate(void)
{
    return head_cali.temperate;
}

//遥控器操作校准云台，陀螺仪
static void RC_cmd_to_calibrate(void)
{
    static uint8_t i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time = 0;
    static uint16_t rc_cmd_time = 0;
    static uint8_t rc_action_falg = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_cmd)
        {
            buzzer_time = 0;
            rc_cmd_time = 0;
            rc_action_falg = 0;
            return;
        }
    }

    if (rc_action_falg == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //判断遥控器2s内八开始20s校准选择时间，rc_action_falg及rc_cmd_time在下方逻辑判断
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_falg = 1;
        rc_cmd_time = 0;
    }
    else if (rc_action_falg == 2 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //判断遥控器在20s校准选择时间，上外八使能云台校准，并且保持2s,rc_action_falg及rc_cmd_time在下方逻辑判断
        rc_action_falg = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
    }
    else if (rc_action_falg == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //判断遥控器在20s校准选择时间，下外八使能陀螺仪校准，并且保持2s，rc_action_falg及rc_cmd_time在下方逻辑判断
        rc_action_falg = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        //更新MPU6500需要控制的温度
        head_cali.temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
        {
            head_cali.temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
    }

    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg == 0)
    {
        //判断遥控器2s内八 计时的时间， 当rc_cmd_time > 2000 为保持2s
        rc_cmd_time++;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg != 0)
    {
        //判断遥控器2s上外八 计时的时间， 云台使能
        rc_cmd_time++;
        rc_action_falg = 2;
    }

    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg != 0)
    {
        //判断遥控器2s下外八 计时的时间， 陀螺仪使能
        rc_cmd_time++;
        rc_action_falg = 3;
    }
    else
    {
        rc_cmd_time = 0;
    }

    calibrate_systemTick = xTaskGetTickCount();

    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //判断遥控器20s校准选择时间，无操作
        rc_action_falg = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_falg != 0)
    {

        //判断遥控器10s后校准选择时间切换蜂鸣器高频声音
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_falg != 0)
    {

        //遥控器10s前 开始蜂鸣器低频声音
        rc_cali_buzzer_start_on();
    }

    if (rc_action_falg != 0)
    {
        buzzer_time++;
    }
    //蜂鸣器断续发声
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_falg != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_falg != 0)
    {
        cali_buzzer_off();
    }
}

//初始化校准结构体数组，读取flash值，如果未校准，使能校准命令,同时初始化对应校准数据
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //如果校准完毕，则将校准值传递到对应的校准参数
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //读取校准设备的前部数据
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        //将名字，校准标识符进行赋值
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        //flash位置偏移
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //将flash保存的数据，传递到设备对应的变量地址中
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //偏移校准数据字节大小
        offset += cali_sensor[i].flash_len * 4;
        //如果设备未校准，使能校准
        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}

static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;
    const uint16_t len = (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3) / 4 + 5;
    uint8_t buf[len * 4];
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //复制设备前部参数，例如名字， 数据大小
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //复制设备校准数据
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);

        offset += cali_sensor[i].flash_len * 4;
    }

    //写入flash
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)buf, len);
}

//头设备校准函数，主要保存纬度，MPU6500控制的温度，硬件版本号
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    if (cmd == 0)
        return 1;
    head_cali.self_id = SELF_ID;
    head_cali.temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (head_cali.temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        head_cali.temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    head_cali.firmware_version = FIRMWARE_VERSION;
    head_cali.latitude = Latitude_At_ShenZhen;
    return 1;
}

//校准陀螺仪设备，主要校准零漂
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //掉线遥控器以防误操作
            imu_start_buzzer();
            return 0;
        }
    }
    return 0;
}

static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
        {
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gimbal_start_buzzer();
            return 0;
        }
    }
    return 0;
}

//返回纬度信息
void getFlashLatitude(float *latitude)
{

    if (latitude == NULL)
    {
        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = Latitude_At_ShenZhen;
    }
}
