#include "main.h"

void miniPC_Init()
{
	enemy_position.mode = 0;
	enemy_position.SEE_pitch_angle.d = 0;
	enemy_position.SEE_yaw_angle.d = 0;
	enemy_position.SEE_shoot_speed.f = 0;
	enemy_position.SEE_yaw_speed.d = 0;
	enemy_position.z.f = 0;
}

void miniPC_rx()
{
		mode = enemy_position.mode ;
		SEE_yaw_angle = enemy_position.SEE_yaw_angle.d;
		SEE_pitch_angle = enemy_position.SEE_pitch_angle.d;
		z = enemy_position.z.f;
}

//void miniPC_uart6_tx(u8 *USART_RX_BUF ,int len);//·¢ËÍÊý¾Ý
