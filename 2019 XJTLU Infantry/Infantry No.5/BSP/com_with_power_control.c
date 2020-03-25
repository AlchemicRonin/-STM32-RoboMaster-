//#include "main.h"
//extern uint8_t NIUYAO;
//u8 power_control_state = 0;
//typedef union 
//{
//	char x[8];
//	struct 
//	{
//		float a;
//		float b;
//	}F;
//}POWER_CONTROL;

//POWER_CONTROL power_control;

//int aauu = 0;
//void Set_Flag_Current(float x, float y)
//{
//	CanTxMsg tx_message;    
//  tx_message.StdId = 0x103;//发给定位板数据
//  tx_message.IDE = CAN_Id_Standard;
//  tx_message.RTR = CAN_RTR_Data;
//  tx_message.DLC = 0x08;
//	power_control.F.a = x;
//	power_control.F.b = y;
//	for(int i = 0; i < 8; i++)
//	tx_message.Data[i] = power_control.x[i];
//	aauu ++;
//	can_data.can_tx = x;
//	CAN_Transmit(CAN1,&tx_message);
//}

////u8 power_control_state = 0;
//u8 charge_flag = 0;
//int32_t cooler_times = 0;

//uint16_t power_buffer = 0;
//uint16_t last_power_buffer = 0; 
//int16_t power_diff = 0;
//uint32_t power_buffer_cnts = 0;
//uint64_t toggle_ms = 0;
//u8 com_flag = 0;


//	uint32_t state_3_times = 0;
//	uint32_t state_2_times = 0;
//	uint16_t real_power = 0;
//	int16_t limit_power = 0;
//	u8 now_power_state = 0;
//	u8 last_power_state = 0;
//	u8 power_state_flag = 0;
//int64_t wait_charge_flag = 0;
//void wait_to_charge(void)
//{
//	if(rc.W == 0 &&  rc.A == 0  &&  rc.S == 0&&  rc.D == 0&& fabs(rc.mouse_x) <= 5.0f)
//	{
//		wait_charge_flag++;
//	}
//	else
//	{
//		wait_charge_flag = 0;
//	}
//}
//void power_info_tx(void)
//{
//	
//	
//	last_power_buffer = power_buffer;
//	power_buffer = judge_rece_mesg.power_heat_data.chassis_power_buffer;
//	power_diff = (int16_t)(last_power_buffer - power_buffer);
//	if( power_diff >= 59)
//		power_buffer_cnts++;
//	if(power_buffer_cnts > 0)
//	{
//		toggle_ms++;
//		if(toggle_ms <= 500)
//		{
////			flow_led_on(2);
//			flow_led_on(3);
//			flow_led_on(4);
//		}
//		else if(toggle_ms > 500)
//		{
////			flow_led_off(2);
//			flow_led_off(3);
//			flow_led_off(4);			
//		}
//		if(toggle_ms >= 1000)
//			toggle_ms = 0;
//	}
//	else 
//	{
////			flow_led_off(2);
//			flow_led_off(3);
//			flow_led_off(4);			
//	}


//	if(judge_rece_mesg.power_heat_data.chassis_power_buffer == 60)
//		flow_led_on(2);
//	else if(judge_rece_mesg.power_heat_data.chassis_power_buffer < 60)
//		flow_led_off(2);
//	/////////////////////////////////////////------------------
//	if((int16_t)judge_rece_mesg.power_heat_data.chassis_power_buffer >= 30  && rc.Shift == 1 )
//	{
//		
//		if(power_state_flag == 0)
//		{
//				power_state_flag = 1;
//		}	
//		if(power_state_flag)
//		{		
//				power_control_state = 3;				//充电标志
//		}
//		else 
//			power_control_state = 1;
//	}
//	else if((rc.Shift == 0) || ((judge_rece_mesg.power_heat_data.chassis_power_buffer < 50) && (rc.W == 1 ||  rc.A == 1  ||  rc.S == 1 ||  rc.D == 1 || fabs(rc.mouse_x) >= 10.0f || NIUYAO == 1)))
//	{
//		state_2_times++;
//		if(state_2_times >= 10)
//		{
//			power_control_state = 2;				//放电标志
//			if(state_2_times <= 11)
//			{
//				real_power = judge_rece_mesg.power_heat_data.chassis_power_buffer;
//				limit_power = real_power - 20;
//				limit_power = limit_power < 10 ? 10 : limit_power;
//			}
//		}
//		
//		else 
//			power_control_state = 1;
//		state_3_times = 0;
//		power_state_flag = 0;
//	}
//	else 
//	{
//		power_state_flag = 0;
//		power_control_state = 1;       	//死区
//	}
//	

//	/////////////////////////////////--------------
////	if((int16_t)judge_rece_mesg.power_heat_data.chassis_power_buffer == 60  && rc.sl == 3)
////	{
////		if()
////	}	
//	
//	//-------------------------------------
////	Set_Flag_Current(power_control_state, 0);
//		Set_Flag_Current(1, 0);
////		Set_Flag_Current(2, 0);
//	if(power_control_state == 1)
//	{
//		flow_led_on(7);
//		flow_led_off(6);
//		flow_led_off(5);
//	}
//	else if(power_control_state == 2)
//	{
//		flow_led_on(7);
//		flow_led_on(6);
//		flow_led_off(5);
//	}
//	else if(power_control_state == 3)
//	{
//		flow_led_on(7);
//		flow_led_on(6);
//		flow_led_on(5);
//	}
////	if(rc.sl == 1)
////	{
////		flow_led_on(5);
////		flow_led_off(6);
////		flow_led_off(7);
////	}
////	else if(rc.sl == 2)
////	{
////		flow_led_on(5);
////		flow_led_on(6);
////		flow_led_off(7);
////	}
////	else if(rc.sl == 3)
////	{
////		flow_led_on(5);
////		flow_led_on(6);
////		flow_led_on(7);
////	}
////	if(judge_rece_mesg.power_heat_data.chassis_power_buffer <= 30.0f)
////	{
////		power_control_state = 2;				//  放电标志
////		charge_flag = 1;
////		cooler_times = 0;
////	}
//////	else if((int16_t)judge_rece_mesg.power_heat_data.chassis_power_buffer == 60 && chassis_set.chassis_set_total <= 50.0f)
//////	{
//////		cooler_times++;
//////		if(cooler_times >= 200)
//////			power_control_state = 3;				//充电标志
//////		else 
//////			power_control_state = 1;
////////		if(can_data.can_rx == 1)
////////			power_control_state = 3;
//////	}
////	else if((int16_t)judge_rece_mesg.power_heat_data.chassis_power_buffer == 60 && rc.sl == 3)
////	{
////		cooler_times++;
////		if(cooler_times >= 200)
////			power_control_state = 3;				//充电标志
////		else 
////			power_control_state = 1;
//////		if(can_data.can_rx == 1)
//////			power_control_state = 3;
////	}
////	
////	else if(charge_flag == 0)
////	{
////		power_control_state = 1;
////		cooler_times = 0;
////	}
////	
////	if(charge_flag == 1 && (int16_t)judge_rece_mesg.power_heat_data.chassis_power_buffer < 60)
////	{
////		power_control_state = 2;
////		cooler_times = 0;
////	}
////	else if(charge_flag == 1 && (int16_t)judge_rece_mesg.power_heat_data.chassis_power_buffer == 60)
////	{
////		power_control_state = 1;
////		charge_flag = 0;
////		cooler_times = 0;
////	}
////	if(power_control_state == 3)
////		led_red_on();
////	else 
////		led_red_off();
//////		flow_led_on(2);
//////	if(power_control_state == 2)
//////		flow_led_on(1);
//////	if(power_control_state == 1)
//////		flow_led_on(0);
////	
//////	if(judge_rece_mesg.power_heat_data.chassis_power_buffer < 60)
//////		led_red_on();
//////	else 
//////		led_red_off();
////	Set_Flag_Current(power_control_state, 0);

//		
//}
