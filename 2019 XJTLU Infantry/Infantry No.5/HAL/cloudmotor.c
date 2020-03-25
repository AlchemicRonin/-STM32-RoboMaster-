#include "main.h"



extern u8 vision_flag;
Angle t_angle = {0};
int fly_flag = 0;
float differ_set = 0;
float last_set = 0;
float new_set = 0;
int fly_times = 0;
//云台电机pid参数初始化
void angle_init(struct Angle *angle)//根据MPU位置改
{
	angle->back_flag = 0;//标志位初始化
	angle->yaw_set = real_angle.yaw;
	angle->pitch_set = real_angle.pitch;//读取真实值		//限位//编码器值位置值转换成mpu角度值
	
	GMPitchEncoder.init_angle = GMPitchEncoder.ecd_angle ;
	GMYawEncoder.init_angle = GMYawEncoder.ecd_angle ;
	angle->pitch_first = real_angle.pitch ;
	angle->yaw_first = real_angle.yaw ;
	
	angle->pitch_target = PITCH_MECHANICAL_CENTRE + (angle->pitch_first - GMPitchEncoder.init_angle);
	angle->yaw_target = YAW_MECHANICAL_CENTRE + (angle->yaw_first - GMYawEncoder.init_angle);
	
	chassis_set.YAW_CENTRE = YAW_MECHANICAL_CENTRE;
	chassis_set.YAW_CENTRE_Init = YAW_MECHANICAL_CENTRE;
//	chassis_set.chassis_cmd_slow_set_follow.input = chassis_set.chassis_cmd_slow_set_follow.out = YAW_MECHANICAL_CENTRE;
	
}

uint8_t PITCH_SEE = 0;
//uint64_t presstime = 0;
//云台电机更新设定值
void angle_set_update(struct Angle *angle)
{
	
	if(angle->back_flag == 0)//缓慢启动回中(采用空间云台)
	{
		if(((int)angle->yaw_set != (int)angle->yaw_target) || ((int)angle->pitch_set != (int)angle->pitch_target))
		{			
			if(((int)angle->pitch_set != (int)angle->pitch_target))
			{
				angle->pitch_set = angle->pitch_set < angle->pitch_target ? angle->pitch_set + 0.1f : angle->pitch_set - 0.1f;
				angle->pitch_remote_last = angle->pitch_remote_now = angle->pitch_set;
			}
			if(((int)angle->yaw_set != (int)angle->yaw_target) && ((int)angle->pitch_set == (int)angle->pitch_target))
				angle->yaw_set = angle->yaw_set < angle->yaw_target ? angle->yaw_set + 0.1f : angle->yaw_set - 0.1f;

		}
		else
			angle->back_flag = 1;	
	}
	
	else		//回中结束后开始正常操作 Q自瞄
	{			
		if((rc.Ctrl == 1) && (rc.A == 1))
		{
			PITCH_SEE = 1;
		}
		else if((rc.Ctrl == 1) && (rc.D == 1))
		{
			PITCH_SEE = 0;
		}

		
		if(rc.mouse_r == 1)
		{
			if(PITCH_SEE == 1)
			{
				angle->yaw_set += rc.L_x / 3000.0f - (rc.mouse_x / 300.0f) + out[VISUAL_YAW_OFFSET];	
				//pitch轴加入限位
				angle->pitch_remote_last = angle->pitch_remote_now;
				angle->pitch_remote_now += (-rc.L_y / 4000.0f) + (rc.mouse_y / 400.0f) + out[VISUAL_PITCH_OFFSET];
				angle->pitch_differ = angle->pitch_remote_now - angle->pitch_remote_last;
			}
			else if(PITCH_SEE == 0)
			{
				angle->yaw_set += rc.L_x / 3000.0f - (rc.mouse_x / 300.0f) + out[VISUAL_YAW_OFFSET];	
				//pitch轴加入限位
				angle->pitch_remote_last = angle->pitch_remote_now;
				angle->pitch_remote_now += (-rc.L_y / 4000.0f) + (-rc.mouse_y / 400.0f) + out[VISUAL_PITCH_OFFSET];
				angle->pitch_differ = angle->pitch_remote_now - angle->pitch_remote_last;
			}

		}

		else
		{ 
			//yaw轴不限位
			angle->yaw_set += rc.L_x / 3000.0f - (rc.mouse_x / 300.0f) ;
			//pitch轴加入限位
			angle->pitch_remote_last = angle->pitch_remote_now;
			angle->pitch_remote_now += (rc.L_y / 4000.0f) + (rc.mouse_y / 400.0f) ;
			angle->pitch_differ = angle->pitch_remote_now - angle->pitch_remote_last;
		}
		
		if(GMPitchEncoder.ecd_angle <= PITCH_MECHANICAL_LOW && angle->pitch_differ > 0.0f)
			angle->pitch_set +=  (angle->pitch_differ);
		 
		if(GMPitchEncoder.ecd_angle >= PITCH_MECHANICAL_TOP && angle->pitch_differ < 0.0f)
			angle->pitch_set +=  (angle->pitch_differ);

		if(GMPitchEncoder.ecd_angle > PITCH_MECHANICAL_LOW && GMPitchEncoder.ecd_angle < PITCH_MECHANICAL_TOP)
			angle->pitch_set +=  angle->pitch_differ;	 
	}

}
//云台电机更新输出电流值
void angle_out_update(struct Angle *angle)
{
	//设定值更新
	angle_set_update(angle);
	
	//pid计算输出值
	out[YAW_ANGLE] = Calculate_Current_Value(&pid[YAW_ANGLE], angle->yaw_set, real_angle.yaw);		
	out[YAW_SPEED] = Calculate_Current_Value(&pid[YAW_SPEED], out[YAW_ANGLE], real_angle.gy);
	
	out[PITCH_ANGLE] = Calculate_Current_Value(&pid[PITCH_ANGLE], angle->pitch_set, real_angle.pitch);
	out[PITCH_SPEED] = Calculate_Current_Value(&pid[PITCH_SPEED], out[PITCH_ANGLE], real_angle.gx);
	

//    if(rc.off_line_flag == 0)
//        Set_CloudMotor_Current((int16_t)(out[YAW_SPEED]),(int16_t)(-out[PITCH_SPEED]));
//            Set_CloudMotor_Current((int16_t)(out[YAW_SPEED]),0);
//    Set_CloudMotor_Current(0,(int16_t)(-out[PITCH_SPEED]));
//    else
//        Set_CloudMotor_Current(0, 0);
}

void Set_CloudMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)//云台电机电流发送函数//CAN1发送函数
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
	tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CAN1,&tx_message);
}

