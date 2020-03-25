#include "main.h"

Feed_set feed_set = {0};

void Shoot()
{
	if((rc.E == 1) && (rc.Ctrl ==0))
	{
		feed_set.feed_speed_max = 800;
	}
	else
	{
		feed_set.feed_speed_max = 500;
	}
	
	
	if((rc.Q == 1) && (rc.Ctrl == 1) && (rc.Shift == 1))
	{
		Set_FeedMotor_Current(-(int16_t)2000);
	}
	else
	{
		if(rc.mouse_l == 1 || rc.sr==3 ) //射
		{
			feed_set.feed_speed_set = feed_set.feed_speed_max;
		}
		else if(rc.mouse_l == 0 || rc.sr == 2) //停
		{
			feed_set.feed_speed_set = 0.0f;
		}

		fireMotorSingleLoopPid_out_update();

	}

}

void fireMotorPidReal_update(void)
{
	feed_set.feed_speed_real = CMFeedEncoder.filter_rate;
}
void fireMotorSingleLoopPid_out_update(void)
{
	fireMotorPidReal_update();//真实值更新
	out[FEED_MOTOR_SINGLE] = Calculate_Current_Value(&pid[FEED_MOTOR_SINGLE], feed_set.feed_speed_set, feed_set.feed_speed_real);
	Set_FeedMotor_Current((int16_t)out[FEED_MOTOR_SINGLE]);
}

void Set_FeedMotor_Current(int16_t feed_motor_iq)//拨弹电机电流发送函数//CAN2发送函数
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
		tx_message.Data[4] = (unsigned char)(feed_motor_iq >> 8);
    tx_message.Data[5] = (unsigned char)feed_motor_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CAN2,&tx_message);
}




//#include "main.h"

//Feed_set feed_set = {0, 0, 0, 0};
//Feed_Motor_State feed_state = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//void Shoot()
//{
//	feed_state.now_mouse_l = rc.mouse_l + (rc.sl == 2);
//	if(feed_state.now_mouse_l == 1 )//射
//	{
//		feed_state.pressState = 2;
//		feed_set.feed_speed_set = Feed_Speed_MAX;
//	}
//	else if(feed_state.now_mouse_l == 0 )//停
//	{
//		feed_state.pressState = 0;
//	}
//	feed_state.now_Jam = IfJam2(feed_state.pressState);//检测卡球
//	
//	if(feed_state.last_Jam == 0 && feed_state.now_Jam == 1)
//	{
//		feed_state.pressState = 1;//防卡弹
//		feed_set.feed_angle_set = feed_set.feed_angle_real - SingleBall;
//		feed_state.jamTime_1ms = 0;//检测卡弹初始化
//	}
//	feed_state.last_Jam = feed_state.now_Jam;
//	
//	if(feed_state.press_flag == 1)
//	{
//		feed_state.presstime_1ms ++;
//		if(feed_state.presstime_1ms > 300)
//		{
//			feed_state.presstime_1ms = 0;
//			feed_state.press_flag = 0;
//		}
//	}
//	
//	switch(feed_state.pressState)//输出
//	{
//		case 1 : 
//			feed_state.press_flag = 1;//位置环后退500ms
//			feed_state.press_number = 1;
//			fireMotorDoubleLoopPid_out_update();
//			break;
//		
//		case 2 : 
//			if(feed_state.press_flag != 1)
//			{
//				feed_state.press_number = 2;
//				feed_set.feed_angle_set = 0;
//				feed_state.sum_angle = (int)feed_set.feed_angle_real;
//				fireMotorSingleLoopPid_out_update();
//					 
//				feed_state.dance_time ++;//按下200ms后扭腰
//				if(feed_state.dance_time > Feed_All_Time)
//				{
//					feed_state.dance_time = 0;
//					feed_state.dance_flag = 1;
//				}
//			}
//			else
//			{
//				fireMotorDoubleLoopPid_out_update();
//			}
//			break;
//						 

//		case 0 : 
//			if(feed_state.press_number == 1)
//				fireMotorDoubleLoopPid_out_update();
//			else
//			{
//				feed_set.feed_speed_set = 0;
//				feed_set.feed_angle_set = ((feed_state.sum_angle / SingleBall) + 1) * SingleBall;
//				fireMotorDoubleLoopPid_out_update();
//			}
//			feed_state.dance_time = 0;
//			feed_state.dance_flag = 0;
//			break;
//	}
//	
//	feed_state.if_dance = Judge_Dance();
//}
//int16_t Judge_Dance()
//{
//	if((feed_state.dance_flag == 1) && (feed_state.last_dance_flag == 0))
//	{
//		feed_state.last_dance_flag = feed_state.dance_flag;
//		return 1;
//	}
//	else if((feed_state.dance_flag == 0) && (feed_state.last_dance_flag == 1))
//	{
//		feed_state.last_dance_flag = feed_state.dance_flag;
//		return 2;
//	}
//	feed_state.last_dance_flag = feed_state.dance_flag;
//	return 0;
//}
//int16_t IfJam2(int16_t state)
//{
//	if(state == 1 || state == 0)
//	{
//		if(fabs(feed_set.feed_angle_real - feed_set.feed_angle_set) >= Feed_Angle_Err)//绝对值
//			feed_state.jamTime_1ms++;
//		else 
//			feed_state.jamTime_1ms = 0;
//		if(feed_state.jamTime_1ms > Feed_All_Time)//200ms后判断是卡弹
//			return 1;

//	}
//	else if(state == 2)
//	{
//		if(fabs(feed_set.feed_speed_real - feed_set.feed_speed_set) >= Feed_Speed_Err)
//			feed_state.jamTime_1ms++;
//		else 
//			feed_state.jamTime_1ms = 0;
//		if(feed_state.jamTime_1ms > Feed_All_Time)
//			return 1;

//	}
//	return 0;
//}

//void fireMotorPidReal_update(void)
//{
//	feed_set.feed_angle_real = CMFeedEncoder.ecd_angle;
//	feed_set.feed_speed_real = CMFeedEncoder.filter_rate;
//}
//void fireMotorSingleLoopPid_out_update(void)
//{
//	fireMotorPidReal_update();//真实值更新
//	out[FEED_MOTOR_SINGLE] = Calculate_Current_Value(&pid[FEED_MOTOR_SINGLE], feed_set.feed_speed_set, feed_set.feed_speed_real);
//	Set_FeedMotor_Current((int16_t)out[FEED_MOTOR_SINGLE]);
//}

//void fireMotorDoubleLoopPid_out_update(void)
//{
//	fireMotorPidReal_update();
//	out[FEED_MOTOR_DOUBLE_ANGLE] = Calculate_Current_Value(&pid[FEED_MOTOR_DOUBLE_ANGLE], feed_set.feed_angle_set, feed_set.feed_angle_real);
//	Set_FeedMotor_Current((int16_t)out[FEED_MOTOR_DOUBLE_ANGLE]);
//}

//void Set_FeedMotor_Current(int16_t feed_motor_iq)//拨弹电机电流发送函数//CAN2发送函数
//{
//    CanTxMsg tx_message;    
//    tx_message.StdId = 0x1FF;
//    tx_message.IDE = CAN_Id_Standard;
//    tx_message.RTR = CAN_RTR_Data;
//    tx_message.DLC = 0x08;
//    
//    tx_message.Data[0] = 0x00;
//    tx_message.Data[1] = 0x00;
//    tx_message.Data[2] = 0x00;
//    tx_message.Data[3] = 0x00;
//	tx_message.Data[4] = (unsigned char)(feed_motor_iq >> 8);
//    tx_message.Data[5] = (unsigned char)feed_motor_iq;
//    tx_message.Data[6] = 0x00;
//    tx_message.Data[7] = 0x00;
//    CAN_Transmit(CAN2,&tx_message);
//}























