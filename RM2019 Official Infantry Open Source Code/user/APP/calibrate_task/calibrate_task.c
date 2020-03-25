/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      ���У׼����豸�����ݣ�������̨�������ǣ����ٶȼƣ�������
  *             ��̨У׼��Ҫ����ֵ�������С��ԽǶȣ���������ҪУ׼��Ư
  *             ���ٶȼƺʹ�����ֻ��д�ýӿں��������ٶȼ�Ŀǰû�б�ҪУ׼��
  *             ��������δʹ���ڽ����㷨�С�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

//����У׼�豸�����֣�У׼��ʶ����У׼����flash��С��У׼�����Ӧ��У׼���ݵ�ַ
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

static void cali_data_read(void);                           //��ȡ����У׼����
static void cali_data_write(void);                          //д�뵱ǰУ׼����������
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //��ͷУ׼���ݺ���
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //������У׼���ݺ���
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //��̨У׼���ݺ���

static const RC_ctrl_t *calibrate_RC; //ң�����ṹ��ָ��
static head_cali_t head_cali;         //��ͷУ׼����
static gimbal_cali_t gimbal_cali;     //��̨У׼����
static imu_cali_t gyro_cali;          //������У׼����
static imu_cali_t accel_cali;         //���ٶ�У׼����
static imu_cali_t mag_cali;           //������У׼����

static cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; //У׼�豸���飬

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"}; //У׼�豸������

//У׼�豸��Ӧ����ṹ�������ַ
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] =
    {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};

//У׼�豸��Ӧ�������ݴ�С
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

//У׼�豸��Ӧ��У׼����
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

//У׼��Ӧʱ���������freeRTOS��tick��ɡ�
static uint32_t calibrate_systemTick;

//ң��������У׼�豸��У׼
static void RC_cmd_to_calibrate(void);

void calibrate_task(void *pvParameters)
{
    static uint8_t i = 0;
    calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {

        //ң��������У׼����
        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            //У׼����Ϊ1 ��ʾ��ҪУ׼
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {
                    //����У׼����
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //У׼���
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];

                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //д��flash
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
//����mpu6500��Ҫ���Ƶ����¶�
int8_t get_control_temperate(void)
{
    return head_cali.temperate;
}

//ң��������У׼��̨��������
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
        //�ж�ң����2s�ڰ˿�ʼ20sУ׼ѡ��ʱ�䣬rc_action_falg��rc_cmd_time���·��߼��ж�
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_falg = 1;
        rc_cmd_time = 0;
    }
    else if (rc_action_falg == 2 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //�ж�ң������20sУ׼ѡ��ʱ�䣬�����ʹ����̨У׼�����ұ���2s,rc_action_falg��rc_cmd_time���·��߼��ж�
        rc_action_falg = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
    }
    else if (rc_action_falg == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //�ж�ң������20sУ׼ѡ��ʱ�䣬�����ʹ��������У׼�����ұ���2s��rc_action_falg��rc_cmd_time���·��߼��ж�
        rc_action_falg = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        //����MPU6500��Ҫ���Ƶ��¶�
        head_cali.temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
        {
            head_cali.temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
    }

    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg == 0)
    {
        //�ж�ң����2s�ڰ� ��ʱ��ʱ�䣬 ��rc_cmd_time > 2000 Ϊ����2s
        rc_cmd_time++;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg != 0)
    {
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ��̨ʹ��
        rc_cmd_time++;
        rc_action_falg = 2;
    }

    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg != 0)
    {
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ������ʹ��
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
        //�ж�ң����20sУ׼ѡ��ʱ�䣬�޲���
        rc_action_falg = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_falg != 0)
    {

        //�ж�ң����10s��У׼ѡ��ʱ���л���������Ƶ����
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_falg != 0)
    {

        //ң����10sǰ ��ʼ��������Ƶ����
        rc_cali_buzzer_start_on();
    }

    if (rc_action_falg != 0)
    {
        buzzer_time++;
    }
    //��������������
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_falg != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_falg != 0)
    {
        cali_buzzer_off();
    }
}

//��ʼ��У׼�ṹ�����飬��ȡflashֵ�����δУ׼��ʹ��У׼����,ͬʱ��ʼ����ӦУ׼����
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
                //���У׼��ϣ���У׼ֵ���ݵ���Ӧ��У׼����
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
        //��ȡУ׼�豸��ǰ������
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        //�����֣�У׼��ʶ�����и�ֵ
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        //flashλ��ƫ��
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //��flash��������ݣ����ݵ��豸��Ӧ�ı�����ַ��
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //ƫ��У׼�����ֽڴ�С
        offset += cali_sensor[i].flash_len * 4;
        //����豸δУ׼��ʹ��У׼
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
        //�����豸ǰ���������������֣� ���ݴ�С
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //�����豸У׼����
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);

        offset += cali_sensor[i].flash_len * 4;
    }

    //д��flash
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)buf, len);
}

//ͷ�豸У׼��������Ҫ����γ�ȣ�MPU6500���Ƶ��¶ȣ�Ӳ���汾��
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

//У׼�������豸����ҪУ׼��Ư
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
            gyro_cali_disable_control(); //����ң�����Է������
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

//����γ����Ϣ
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
