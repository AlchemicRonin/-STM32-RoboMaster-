/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܣ���������ĳ�ʼ�����Լ�ѭ����������̨�����е��ã��ʶ����ļ�
  *             ����freeRTOS��������ֹر�״̬��׼��״̬�����״̬���Լ����״̬
  *             �ر�״̬�ǹر�Ħ�����Լ����⣬׼��״̬�ǽ��ӵ�����΢�Ϳ��ش������״
  *             ̬�ǽ��ӵ�������ж�΢�Ϳ���ֵ���������״̬�����״̬ͨ���ж�һ��ʱ��
  *             ΢�Ϳ������ӵ���Ϊ�Ѿ����ӵ������
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

#include "Shoot.h"
#include "CAN_Receive.h"
#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "pid.h"
#include "laser.h"
#include "fric.h"
#include "arm_math.h"
#include "user_lib.h"


#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         //�ر�����Ħ����

#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��

static const RC_ctrl_t *shoot_rc; //ң����ָ��

static PidTypeDef trigger_motor_pid;         //���PID
static Shoot_Motor_t trigger_motor;          //�������
static shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬��
//΢������IO
#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

extern void getTriggerMotorMeasure(motor_measure_t *motor);



/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    //ң����ָ��
    shoot_rc = get_remote_control_point();
    //���ָ��
    trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
    //��ʼ��PID
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    //��������
    Shoot_Feedback_Update();
    ramp_init(&trigger_motor.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
    ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);

    trigger_motor.ecd_count = 0;
    trigger_motor.angle = trigger_motor.shoot_motor_measure->ecd * Motor_ECD_TO_ANGLE;
    trigger_motor.given_current = 0;
    trigger_motor.move_flag = 0;
    trigger_motor.set_angle = trigger_motor.angle;
    trigger_motor.speed = 0.0f;
    trigger_motor.speed_set = 0.0f;
    trigger_motor.BulletShootCnt = 0;
}
/**
  * @brief          ���ѭ��
  * @author         RM
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{
    int16_t shoot_CAN_Set_Current; //���ص�canֵ

    Shoot_Set_Mode();        //����״̬��
    Shoot_Feedback_Update(); //��������

    //����״̬����
    if (shoot_mode == SHOOT_BULLET)
    {
        trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    //�������״̬����
    else if (shoot_mode == SHOOT_DONE)
    {
        shoot_done_control();
    }
    //����׼��״̬����
    else if (shoot_mode == SHOOT_READY)
    {
        trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        shoot_ready_control();
    }

    if (shoot_mode == SHOOT_STOP)
    {
        trigger_motor.fric1_ramp.out = Fric_OFF;
        trigger_motor.fric2_ramp.out = Fric_OFF;
        shoot_fric_off();
        shoot_laser_off();
        shoot_CAN_Set_Current = 0;
    }
    else
    {
        //Ħ����pwm
        static uint16_t fric_pwm1 = Fric_OFF;
        static uint16_t fric_pwm2 = Fric_OFF;


        shoot_laser_on();       //���⿪��


        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&trigger_motor.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

        if(trigger_motor.fric1_ramp.out == trigger_motor.fric1_ramp.max_value)
        {
            ramp_calc(&trigger_motor.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }

        if( trigger_motor.fric2_ramp.out != trigger_motor.fric2_ramp.max_value)
        {
            trigger_motor.speed_set = 0.0f;
        }


//����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
        static uint16_t up_time = 0;
        if (trigger_motor.press_r)
        {
            up_time = UP_ADD_TIME;
        }

        if (up_time > 0)
        {
            trigger_motor.fric1_ramp.max_value = Fric_UP;
            trigger_motor.fric2_ramp.max_value = Fric_UP;
            up_time--;
        }
        else
        {
            trigger_motor.fric1_ramp.max_value = Fric_DOWN;
            trigger_motor.fric2_ramp.max_value = Fric_DOWN;
        }

        fric_pwm1 = (uint16_t)(trigger_motor.fric1_ramp.out);
        fric_pwm2 = (uint16_t)(trigger_motor.fric2_ramp.out);

        shoot_fric1_on(fric_pwm1);
        shoot_fric2_on(fric_pwm2);

        //���㲦���ֵ��PID
        PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);

        trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
        shoot_CAN_Set_Current = trigger_motor.given_current;
    }

    return shoot_CAN_Set_Current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || (shoot_rc->key.v & SHOOT_OFF_KEYBOARD))
    {
        shoot_mode = SHOOT_STOP;
    }

    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
    {
        shoot_mode = SHOOT_STOP;
    }

    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_mode = SHOOT_STOP;
    }

    if (shoot_mode == SHOOT_READY)
    {
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_down(last_s)) || (trigger_motor.press_l && trigger_motor.last_press_l == 0) || (trigger_motor.press_r && trigger_motor.last_press_r == 0))
        {
            shoot_mode = SHOOT_BULLET;
            trigger_motor.last_butter_count = trigger_motor.BulletShootCnt;
        }
        //��곤��һֱ�������״̬ ��������
        if ((trigger_motor.press_l_time == PRESS_LONG_TIME) || (trigger_motor.press_r_time == PRESS_LONG_TIME) || (trigger_motor.rc_s_time == RC_S_LONG_TIME))
        {
            if (shoot_mode != SHOOT_DONE && trigger_motor.key == SWITCH_TRIGGER_ON)
            {
                shoot_mode = SHOOT_BULLET;
            }
        }
    }

    last_s = shoot_rc->rc.s[Shoot_RC_Channel];
}
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
    {
        trigger_motor.ecd_count++;
    }

    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
    //΢������
    trigger_motor.key = Butten_Trig_Pin;
    //��갴��
    trigger_motor.last_press_l = trigger_motor.press_l;
    trigger_motor.last_press_r = trigger_motor.press_r;
    trigger_motor.press_l = shoot_rc->mouse.press_l;
    trigger_motor.press_r = shoot_rc->mouse.press_r;
    //������ʱ
    if (trigger_motor.press_l)
    {
        if (trigger_motor.press_l_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_l_time++;
        }
    }
    else
    {
        trigger_motor.press_l_time = 0;
    }

    if (trigger_motor.press_r)
    {
        if (trigger_motor.press_r_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_r_time++;
        }
    }
    else
    {
        trigger_motor.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
    {

        if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
        {
            trigger_motor.rc_s_time++;
        }
    }
    else
    {
        trigger_motor.rc_s_time = 0;
    }
}
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //�ӵ�����ж�
    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
    {
        trigger_motor.shoot_done = 1;
        trigger_motor.shoot_done_time = 0;

        shoot_mode = SHOOT_DONE;
        trigger_motor.set_angle = trigger_motor.angle;
    }

    //ÿ�β��� 1/4PI�ĽǶ�
    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
        trigger_motor.cmd_time = xTaskGetTickCount();
        trigger_motor.move_flag = 1;
    }

    //����Ƕ��ж�
    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        trigger_motor.speed_set = TRIGGER_SPEED;
        trigger_motor.run_time = xTaskGetTickCount();

        //��ת�ж�
        if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
        {
            trigger_motor.speed_set = -TRIGGER_SPEED;
        }
        else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
        {
            trigger_motor.cmd_time = xTaskGetTickCount();
        }
    }
    else
    {
        trigger_motor.move_flag = 0;
    }
}
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void)
{
    trigger_motor.speed_set = 0.0f;
    //�������жϣ��ж�΢������һ��ʱ�����ӵ�
    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
    {
        if (trigger_motor.shoot_done_time < SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor.shoot_done_time++;
        }
        else if (trigger_motor.shoot_done_time == SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor.BulletShootCnt++;
            shoot_mode = SHOOT_READY;
        }
    }
    else
    {
        shoot_mode = SHOOT_BULLET;
    }
}
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void)
{

    if (trigger_motor.shoot_done)
    {
        trigger_motor.shoot_done = 0;
    }

    if (trigger_motor.key == SWITCH_TRIGGER_ON)
    {
        //�ж��ӵ�����΢�����ش�
        trigger_motor.set_angle = trigger_motor.angle;
        trigger_motor_pid.out = 0.0f;
        trigger_motor_pid.Iout = 0.0f;

        trigger_motor.speed_set = 0.0f;
        trigger_motor.move_flag = 0;
        trigger_motor.key_time = 0;
    }
    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time < KEY_OFF_JUGUE_TIME)
    {
        //�ж����ӵ�һ��ʱ��
        trigger_motor.key_time++;
    }
    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time == KEY_OFF_JUGUE_TIME)
    {
        //΢������һ��ʱ��û���ӵ������벦����һ����ת 1/10PI�ĽǶ�
        if (trigger_motor.move_flag == 0)
        {
            trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
            trigger_motor.cmd_time = xTaskGetTickCount();
            trigger_motor.move_flag = 1;
        }

        if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
        {
            //�Ƕȴﵽ�ж�
            trigger_motor.speed_set = Ready_Trigger_Speed;
            trigger_motor.run_time = xTaskGetTickCount();
            //��ת�ж�
            if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
            {
                trigger_motor.speed_set = -Ready_Trigger_Speed;
            }
            else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
            {
                trigger_motor.cmd_time = xTaskGetTickCount();
            }
        }
        else
        {
            trigger_motor.move_flag = 0;
        }
    }
}
