/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "buzzer.h"
#include "Detect_Task.h"

#include "user_lib.h"

////��̨У׼����������
//#define GIMBALWarnBuzzerOn() buzzer_on(31, 20000)
//#define GIMBALWarnBuzzerOFF() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          ��̨У׼��ͨ���жϽ��ٶ����ж���̨�Ƿ񵽴Ｋ��λ��
  * @author         RM
  * @param[in]      ��Ӧ��Ľ��ٶȣ���λrad/s
  * @param[in]      ��ʱʱ�䣬����GIMBAL_CALI_STEP_TIME��ʱ������
  * @param[in]      ��¼�ĽǶ� rad
  * @param[in]      �����ĽǶ� rad
  * @param[in]      ��¼�ı���ֵ raw
  * @param[in]      �����ı���ֵ raw
  * @param[in]      У׼�Ĳ��� ���һ�� ��һ
  * @retval         ���ؿ�
  */
#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          ��̨��Ϊ״̬�����ã���Ϊ��cali��ģʽ��ʹ����return���ʶ�������һ������
  * @author         RM
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);

/**
  * @brief          ��̨�������ƣ������ģʽ�·��͵�yaw��pitch �ǵ������ԭʼֵ����̨�������can���������ʹ����̨����
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

//��̨��Ϊ״̬��
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          ��̨��Ϊ״̬���Լ����״̬������
  * @author         RM
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */

gimbal_motor_mode_e test = GIMBAL_MOTOR_ENCONDE;
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //��̨��Ϊ״̬������
    gimbal_behavour_set(gimbal_mode_set);

    //������̨��Ϊ״̬�����õ��״̬��
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @author         RM
  * @param[in]      ���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[in]      ���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //��ң���������ݴ������� int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    //��������������ֵ
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
  * @brief          ��̨��Ϊ״̬�����ã���Ϊ��cali��ģʽ��ʹ����return���ʶ�������һ������
  * @author         RM
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //У׼��Ϊ��return ��������������ģʽ
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }
    //����ⲿʹ��У׼�����0 ��� start�������У׼ģʽ
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //��ʼ��ģʽ�ж��Ƿ񵽴���ֵλ��
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        //������ֵ ��ʱ
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            //�����ʼ��λ��
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            //û�е����ʼ��λ�ã�ʱ���ʱ
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //������ʼ�����ʱ�䣬�����Ѿ��ȶ�����ֵһ��ʱ�䣬�˳���ʼ��״̬���ش��µ������ߵ���
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]) && !toe_is_error(DBUSTOE))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //���ؿ��� ��̨״̬
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }

    if( toe_is_error(DBUSTOE))
    {

        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    //�жϽ���init״̬��
    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }

    static uint16_t motionless_time = 0;
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        //ң���� ���̾������룬����motionless״̬
        if (int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[0]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[1]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[2]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[3]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.x) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.y) < GIMBAL_MOTIONLESS_RC_DEADLINE && gimbal_mode_set->gimbal_rc_ctrl->key.v == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r == 0)
        {
            if (motionless_time < GIMBAL_MOTIONLESS_TIME_MAX)
            {
                motionless_time++;
            }
        }
        else
        {
            motionless_time = 0;
        }

        if (motionless_time == GIMBAL_MOTIONLESS_TIME_MAX)
        {
            gimbal_behaviour = GIMBAL_MOTIONLESS;
        }
    }
    else
    {
        motionless_time = 0;
    }


}

/**
  * @brief          ��̨�������ƣ������ģʽ�·��͵�yaw��pitch �ǵ������ԭʼֵ����̨�������can���������ʹ����̨����
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //��ʼ��״̬����������
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //�ж����������ݣ� ����¼�����С�Ƕ�����
        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}
/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TurnKeyBoard) && !(last_turn_keyboard & TurnKeyBoard))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //�����ͷ��Ŀ��ֵ
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //���Ͽ��Ƶ���ͷ��Ŀ��ֵ����ת����װ�����
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TurnSpeed;
            }
            else
            {
                *yaw -= TurnSpeed;
            }
        }
        //����pi ��180�㣩��ֹͣ
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //����Ҫ����
}
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
