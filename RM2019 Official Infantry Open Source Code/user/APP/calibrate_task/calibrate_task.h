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
#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "main.h"

#define imu_start_buzzer() buzzer_on(95, 10000) //IMUԪ��У׼�ķ�������Ƶ���Լ�ǿ��

#define gimbal_start_buzzer() buzzer_on(31, 20000) //��̨У׼�ķ�������Ƶ���Լ�ǿ��

#define cali_buzzer_off() buzzer_off() //�رշ�����

#define cali_get_mcu_temperature() get_temprate() //��ȡstm32�ϵ��¶� ��ΪIMUУ׼�Ļ����¶�

#define cali_flash_read(address, buf, len) flash_read((address), (buf), (len))                  //flash ��ȡ����
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len)) //flash д�뺯��

#define get_remote_ctrl_point_cali() get_remote_control_point() //��ȡң�����Ľṹ��ָ��
#define gyro_cali_disable_control() RC_unable()                 //������У׼ʱ�����ң����

//������У׼����
#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//����������У׼ֵ
#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_23 //Ҫд��flash������Ŀǰ�����һ�飬

#define GYRO_CONST_MAX_TEMP 45.0f //�����ǿ��ƺ��� �������¶�

#define CALI_FUNC_CMD_ON 1   //У׼������ʹ��У׼
#define CALI_FUNC_CMD_INIT 0 //У׼����������flash�е�У׼����

#define CALIBRATE_CONTROL_TIME 1 //У׼���������е����� Ϊ1ms

#define CALI_SENSOR_HEAD_LEGHT 1 //У׼�ṹ��ı�ͷ Ϊcali_sensor_t��ǰ�� �꿴 cali_sensor_t������ ��С 1 ����һ��32λ����

#define SELF_ID 0              //��ͷ�е�ID
#define FIRMWARE_VERSION 12345 //��ͷ�е�Ӳ���汾�� Ŀǰ����д��
#define CALIED_FLAG 0x55       //�������Ѿ�У׼���

#define CALIBRATE_END_TIME 20000 //ң����У׼ʱ��20s ����20s��Ҫ���²���

#define RC_CALI_BUZZER_MIDDLE_TIME 10000 //У׼ʱ���ı������Ƶ�ʳ���̨�ĸ�Ƶ����������������20sУ׼ʱ���������
#define rc_cali_buzzer_middle_on() gimbal_start_buzzer()
#define RC_CALI_BUZZER_START_TIME 0 //У׼ʱ���ı������Ƶ�ʳ�IMU�ĵ�Ƶ����������У׼ʱ��20s��ʼ��ʱ
#define rc_cali_buzzer_start_on() imu_start_buzzer()
#define RCCALI_BUZZER_CYCLE_TIME 400  //У׼ѡ��ʱ��20s��������������������ʱ��
#define RC_CALI_BUZZER_PAUSE_TIME 200 //У׼ѡ��ʱ��20s����������������ͣ��ʱ��
#define RC_CALI_VALUE_HOLE 600        //ң������˻����ڰ� ��ֵ�ж��� ң����ҡ������� 660 ֻҪ����630 ����Ϊ�����

#define RC_CMD_LONG_TIME 2000 //ң����ʹ��У׼��ʱ�䣬�������ڰ˵�ʱ��

#define GYRO_CALIBRATE_TIME 20000 //������У׼��ʱ�� 20s

//У׼�豸��
typedef enum
{
    CALI_HEAD,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_LIST_LENGHT,
    //add more...
} cali_id_e;

//У׼�豸ǰ����ͨ��flash_buf���ӵ���Ӧ��У׼�豸������ַ
typedef __packed struct
{
    uint8_t name[3];
    uint8_t cali_done;
    uint8_t flash_len : 7;
    uint8_t cali_cmd : 1;
    uint32_t *flash_buf;
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);
} cali_sensor_t;

//ͷ�豸��У׼����
typedef __packed struct
{
    uint8_t self_id;
    int8_t temperate;
    uint16_t firmware_version;
    fp32 latitude;
} head_cali_t;
//��̨�豸��У׼����
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;
//�����ǣ����ٶȼƣ�������ͨ��У׼����
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

//��ʼ�����Լ���ȡflashУ׼ֵ
extern void cali_param_init(void);
//����mpu6500���Ƶ��¶�
extern int8_t get_control_temperate(void);
//У׼����
extern void calibrate_task(void *pvParameters);
#endif
