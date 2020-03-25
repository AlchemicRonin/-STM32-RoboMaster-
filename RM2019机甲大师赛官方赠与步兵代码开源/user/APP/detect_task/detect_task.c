/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief      �豸�����ж�����ͨ��freeRTOS�δ�ʱ����Ϊϵͳʱ�䣬�豸��ȡ���ݺ�
  *             ����DetectHook��¼��Ӧ�豸��ʱ�䣬�ڸ������ͨ���жϼ�¼ʱ����ϵͳ
  *             ʱ��֮�����жϵ��ߣ�ͬʱ����ߵ����ȼ�������ͨ��LED�ķ�ʽ�ı䣬����
  *             �˸���ˮ����ʾSBUBң������������̨�ϵĵ����4�����̵��������Ҳͨ��
  *             �����˸��������ʾ�����롣
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

#include "Detect_Task.h"

#include "led.h"

#include "CAN_Receive.h"
#include "Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//��������������л�����
#define DETECT_LED_R_TOGGLE() led_red_toggle()
#define DETECT_LED_R_ON() led_red_on()
#define DETECT_LED_R_OFF() led_red_off()
//��ˮ��������
#define DETECT_FLOW_LED_ON(i) flow_led_on(i)
#define DETECT_FLOW_LED_OFF(i) flow_led_off(i)

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10
//��ʼ�������б�
static void DetectInit(uint32_t time);
//��ʾ���ȼ���ߵĴ��󣬴���Ĳ���Ϊ ��ʾ�Ĵ���Ĵ�����
static void DetectDisplay(uint8_t num);

static error_t errorList[errorListLength + 1];

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t DetectTaskStack;
#endif

//�����ж�����
void DetectTask(void *pvParameters)
{
    static uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    //��ʼ��
    DetectInit(systemTime);
    //����һ��ʱ��
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        static uint8_t error_num_display = 0;
        systemTime = xTaskGetTickCount();

        error_num_display = errorListLength;
        errorList[errorListLength].isLost = 0;
        errorList[errorListLength].errorExist = 0;

        for (int i = 0; i < errorListLength; i++)
        {
            //δʹ�ܣ�����
            if (errorList[i].enable == 0)
            {
                continue;
            }

            //�жϵ���
            if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].errorExist == 0)
                {
                    //��¼�����Լ�����ʱ��
                    errorList[i].isLost = 1;
                    errorList[i].errorExist = 1;
                    errorList[i].Losttime = systemTime;
                }
                //�жϴ������ȼ��� �������ȼ���ߵĴ�����
                if (errorList[i].Priority > errorList[error_num_display].Priority)
                {
                    error_num_display = i;
                }
                //��¼�б�Ĵ��ڴ���
                errorList[errorListLength].isLost = 1;
                errorList[errorListLength].errorExist = 1;
                //����ṩ������������н������
                if (errorList[i].solveLostFun != NULL)
                {
                    errorList[i].solveLostFun();
                }
            }
            else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)
            {
                //�ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
                errorList[i].isLost = 0;
                errorList[i].errorExist = 1;
            }
            else
            {
                errorList[i].isLost = 0;
                //�ж��Ƿ�������ݴ���
                if (errorList[i].dataIsError)
                {
                    errorList[i].errorExist = 1;
                }
                else
                {
                    errorList[i].errorExist = 0;
                }
                //����Ƶ��
                if (errorList[i].newTime > errorList[i].lastTime)
                {
                    errorList[i].frequency = configTICK_RATE_HZ / (fp32)(errorList[i].newTime - errorList[i].lastTime);
                }
            }
        }

        DetectDisplay(error_num_display + 1);
        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        DetectTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

//���ض�Ӧ���豸�Ƿ���ڴ���
bool_t toe_is_error(uint8_t err)
{
    return (errorList[err].errorExist == 1);
}

//�豸�������ݹ��Ӻ���
void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime = xTaskGetTickCount();
    //���¶�ʧ���
    if (errorList[toe].isLost)
    {
        errorList[toe].isLost = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
    //�ж������Ƿ����
    if (errorList[toe].dataIsErrorFun != NULL)
    {
        if (errorList[toe].dataIsErrorFun())
        {
            errorList[toe].errorExist = 1;
            errorList[toe].dataIsError = 1;

            if (errorList[toe].solveDataErrorFun != NULL)
            {
                errorList[toe].solveDataErrorFun();
            }
        }
        else
        {
            errorList[toe].dataIsError = 0;
        }
    }
    else
    {
        errorList[toe].dataIsError = 0;
    }
}
const error_t *getErrorListPoint(void)
{
    return errorList;
}
static void DetectDisplay(uint8_t num)
{
    static uint8_t last_num = errorListLength + 1;
    uint8_t i = 0;

    //8����ˮ��ʾǰ�˵Ĵ���������������SBUSң������yaw.pitch,triger��̨��4�����̵��
    for (i = 0; i <= ChassisMotor4TOE; i++)
    {
        if (errorList[i].errorExist)
        {
            DETECT_FLOW_LED_OFF(i);
        }
        else
        {
            DETECT_FLOW_LED_ON(i);
        }
    }

    //������ ͨ�������˸�������ж�
    if (num == errorListLength + 1)
    {
        DETECT_LED_R_OFF();
        last_num = errorListLength + 1;
    }
    else
    {
        static uint8_t i = 0, led_flag = 0, cnt_num = 0, time = 0;
        //��¼���µ�������ȼ��Ĵ����룬����һ����˸
        if (last_num != num)
        {
            last_num = num;
        }

        if (cnt_num == 0)
        {
            //cnt_num ��¼���м�����˸����0����һ��ʱ��ſ�ʼ��һ��
            time++;
            if (time > 50)
            {
                time = 0;
                cnt_num = last_num;
            }
            return;
        }

        if (i == 0)
        {

            DETECT_LED_R_TOGGLE();
            if (led_flag)
            {
                //��������һ�Σ���Ҫʣ�������һ
                led_flag = 0;
                cnt_num--;
            }
            else
            {
                led_flag = 1;
            }
        }

        //iΪ��ʱ������20Ϊ������ڣ��л�һ�κ������
        i++;

        if (i > 20)
        {
            i = 0;
        }
    }
}

static void DetectInit(uint32_t time)
{
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime priority
    uint16_t setItem[errorListLength][3] =
        {
            {30, 40, 15}, //SBUS
            {2, 3, 14},   //yaw
            {2, 3, 13},   //pitch
            {10, 10, 12}, //trigger
            {10, 10, 11}, //motor1
            {10, 10, 10}, //motor2
            {10, 10, 9},  //motor3
            {10, 10, 8},  //motor4

        };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime = setItem[i][1];
        errorList[i].Priority = setItem[i][2];
        errorList[i].dataIsErrorFun = NULL;
        errorList[i].solveLostFun = NULL;
        errorList[i].solveDataErrorFun = NULL;

        errorList[i].enable = 1;
        errorList[i].errorExist = 1;
        errorList[i].isLost = 1;
        errorList[i].dataIsError = 1;
        errorList[i].frequency = 0.0f;
        errorList[i].newTime = time;
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

    errorList[DBUSTOE].dataIsErrorFun = RC_data_is_error;
    errorList[DBUSTOE].solveLostFun = slove_RC_lost;
    errorList[DBUSTOE].solveDataErrorFun = slove_data_error;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    errorList[YawGimbalMotorTOE].solveLostFun = GIMBAL_lose_slove;
    errorList[PitchGimbalMotorTOE].solveLostFun = GIMBAL_lose_slove;
#endif
}
