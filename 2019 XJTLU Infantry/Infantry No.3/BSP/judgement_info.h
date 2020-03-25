#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f4xx.h"

#define JUDGE_FIFO_BUFLEN 500


typedef enum
{	

	  GAME_STATUS_DATA_ID   = 0x0001,  //比赛状态数据
		GAME_RESULT_ID   			= 0x0002,  //比赛结果数据
		ROBO_EXIST_DATA_ID 		= 0x0003,  //机器人存活数据
		COURT_EVENT_DATA_ID   = 0x0101,  //场地事件数据
		SUPPLY_STA_ACTION_DATA = 0x0102, //场地补给站动作标识数据
		SUPPLY_STA_REQUEST_DATA = 0x103, //请求补给站补弹数据
		ROBO_STATE_DATA = 0x0201,				 //机器人状态数据
		REAL_POWER_ID = 0x0202,				   //实时功率热量数据
		ROBO_POS_DATA_ID  = 0x0203,			 //机器人位置数据
		ROBO_BUFF_DATA = 0x0204,				 //机器人增益数据
		SPACE_ROBO_ENERGY = 0x0205,			 //空中机器人能量状态数据
		HURT_DATA = 0x0206,							 //伤害状态数据
		REAL_SHOOT_DATA = 0x0207,				 //实时射击数据
		ROBO_CONNECTION_DATA = 0x0301,	 //机器人间交互数据
		
} judge_data_id_e;



/** 
  * @brief  1.比赛状态数据：0x0001。发送频率：1Hz 
  */
typedef __packed struct {   
	uint8_t game_type : 4;      //1:RM 2:单项赛 3:ICRA
	uint8_t game_progress : 4;   //当前比赛阶段
	uint16_t stage_remain_time;  //当前剩余时间
} ext_game_state_t; 




/** 
  * @brief  2.比赛结果数据：0x0002。发送频率：比赛结束后发送 
  */
typedef __packed struct 
{    
	uint8_t winner;     //0:平局 1：红方胜利 2:蓝方胜利
} ext_game_result_t; 



/** 
  * @brief  3.机器人存活数据：0x0003。发送频率：1Hz 
							bit 0：红方英雄机器人； 
							bit 1：红方工程机器人； 
							bit 2：红方步兵机器人 1； 
							bit 3：红方步兵机器人 2； 
							bit 4：红方步兵机器人 3；
							bit 5：红方空中机器人； 
							bit 6：红方哨兵机器人； 
							bit 7：保留 
							bit 8：蓝方英雄机器人； 
							bit 9：蓝方工程机器人； 
							bit 10：蓝方步兵机器人 1； 
							bit 11：蓝方步兵机器人 2； 
							bit 12：蓝方步兵机器人 3； 
							bit 13：蓝方空中机器人； 
							bit 14：蓝方哨兵机器人； 
							bit 15：保留 
对应的 bit 数值置 1代表机器人存活，数值置 0代表机器人死亡或者未上场。
  */
typedef __packed struct 
{   
	uint16_t robot_legion;   //见上面
} ext_game_robot_survivors_t; 


/** 
  * @brief 4. 场地事件数据：0x0101。发送频率：事件改变后发送 
  */
typedef __packed struct 
{   
	uint32_t event_type; 
} ext_event_data_t; 


/** 
  * @brief  5.补给站动作标识：0x0102。发送频率：动作改变后发送 
  */
typedef __packed struct 
{   
	uint8_t supply_projectile_id; //1:1号补给口 2:2号补给口   
	uint8_t supply_robot_id;    //补弹机器人ID：0 为当前无机器人补弹，1为红方英雄机器人补弹，2为红方工程 机器人补弹，3/4/5 为红方步兵机器人补弹，
															//11 为蓝方英雄机器人补弹，12 为蓝方 工程机器人补弹，13/14/15为蓝方步兵机器人补弹
	uint8_t supply_projectile_step;//出弹口开闭状态：0为关闭，1为子弹准备中，2为子弹下落  
	uint8_t supply_projectile_num; //50：50 颗子弹； 100：100 颗子弹； 150：150 颗子弹； 200：200 颗子弹。 
} ext_supply_projectile_action_t; 


/** 
  * @brief  6.请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限 10Hz。RM 对抗赛尚未开放 
  */
typedef __packed struct 
{   
	uint8_t supply_projectile_id;    
	uint8_t supply_robot_id;  
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/** 
  * @brief  7.比赛机器人状态：0x0201。发送频率：10Hz 
  */
typedef __packed struct 
{   
	uint8_t robot_id;    //机器人id
	uint8_t robot_level;   //机器人等级 1:一级 2:二级 3:三级
	uint16_t remain_HP;   //机器人剩余血量
	uint16_t max_HP;     //机器人上限血量
	uint16_t shooter_heat0_cooling_rate;   //机器人17mm枪口冷却值
	uint16_t shooter_heat0_cooling_limit;   //机器人17mm热量上限
	uint16_t shooter_heat1_cooling_rate;   //机器人42mm枪口冷却值
	uint16_t shooter_heat1_cooling_limit;   //机器人42mm热量上限
	uint8_t mains_power_gimbal_output : 1;   //gimnbal云台输出情况 1:24v 0:0v
	uint8_t mains_power_chassis_output : 1;  //chassis底盘输情况 1:24v 0:0v
	uint8_t mains_power_shooter_output : 1;  //shooter发射结构输出情况 1:24v 0:0v
} ext_game_robot_state_t; 


/** 
  * @brief  8.实时功率热量数据：0x0202。发送频率：50Hz 
  */
typedef __packed struct 
{   
	uint16_t chassis_volt;    //底盘输出电压 单位：mV  ！！！（注意单位）
	uint16_t chassis_current;    //底盘输出电流 单位:mA 
	float chassis_power;        //底盘输出功率 单位:W 
	uint16_t chassis_power_buffer;//底盘功率缓冲 单位:J     
	uint16_t shooter_heat0;    //17mm 枪口热量 
	uint16_t shooter_heat1;  //42mm 枪口热量 
} ext_power_heat_data_t; 


/** 
  * @brief  9.机器人位置：0x0203。发送频率：10Hz  
  */
typedef __packed struct 
{  
  float x;   
	float y;   
	float z;   
	float yaw; //位置枪口
} ext_game_robot_pos_t; 


/** 
		* @brief  10.机器人增益：0x0204。发送频率：状态改变后发送 
			bit 0：机器人血量补血状态 
			bit 1：枪口热量冷却加速 
			bit 2：机器人防御加成 
			bit 3：机器人攻击加成 
  */
typedef __packed struct 
{   
	uint8_t power_rune_buff; 
}ext_buff_musk_t; 


/** 
  * @brief  11.空中机器人能量状态：0x0205。发送频率：10Hz  
  */
typedef __packed struct 
{   
	uint8_t energy_point;  //积累的能量点 
	uint8_t attack_time;  //可攻击时间 50-0s 递减
} aerial_robot_energy_t; 

/** 
  * @brief  12.伤害状态：0x0206。发送频率：伤害发生后发送 
				bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4号代表机器人 的五个装甲片，其他血量变化类型，该变量数值为 0。 
				bit 4-7：血量变化类型 
				0x0 装甲伤害扣血； 
				0x1 模块掉线扣血； 
				0x2 超枪口热量扣血； 
				0x3 超底盘功率扣血
  */
typedef __packed struct 
{   
	uint8_t armor_id : 4;   
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t;

/** 
  * @brief  13.实时射击信息：0x0207。发送频率：射击后发送  
  */
typedef __packed struct 
{   
	uint8_t bullet_type;  //子弹类型: 1：17mm 弹丸 2：42mm 弹丸   
	uint8_t bullet_freq;    //子弹射频 单位 Hz 
	float bullet_speed;  //子弹射速 单位 m/s 
} ext_shoot_data_t; 



/**机器人间相互通信 （未使用）*/
///** 
//  * @brief  14.交互数据接收信息：0x0301。发送频率：上限 10Hz   
//  */
//typedef __packed struct 
//{   
//	uint16_t data_cmd_id;// 数据段的内容 ID    
//	uint16_t send_ID;    //需要校验发送者的 ID 正确性
//	uint16_t receiver_ID; //需要校验接收者的 ID 正确性
//}ext_student_interactive_header_data_t; 

///** 
//  * @brief  15.客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
//  */

typedef __packed struct 
{ 
  uint16_t data_cmd_id;// 数据段的内容 ID    
  uint16_t send_ID;    //需要校验发送者的 ID 正确性
  uint16_t receiver_ID; //需要校验接收者的 ID 正确性
	float data1; 
	float data2; 
	float data3;
	uint8_t masks; 
} client_custom_data_t; 

//	/** 
//  * @brief  16.交互数据 机器人间通信：0x0301。发送频率：上限10Hz  
//  */

//typedef __packed struct { 
//	uint8_t data[]; 
//} robot_interactive_data_t;

///** 
//  * @brief  the data structure receive from judgement
//  */
typedef struct
{

	
	ext_game_state_t      game_rstatus_data;  //0x0001 比赛状态数据 1Hz 周期发送 
	ext_game_result_t      game_result_data;   //0x0002 比赛结果数据，比赛结束后发送 
	ext_game_robot_survivors_t robot_exis_data;    //0x0003 比赛机器人存活数据，1Hz 周期发送 

	ext_event_data_t			court_event_data;		//0x0101 场地事件数据，事件改变后发送 
	ext_supply_projectile_action_t			supply_act_data;		//0x0102 场地补给站动作标识数据，动作改变后发送 
	ext_supply_projectile_booking_t		supply_request_data;	//0x0103 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放） 

	ext_game_robot_state_t	game_information;   //0x0201 机器人状态数据，10Hz 周期发送 
	ext_power_heat_data_t		power_heat_data;    //0x0202 实时功率热量数据，50Hz 周期发送 
	ext_game_robot_pos_t		robot_pos_data;     //0x0203 机器人位置数据，10Hz 发送 
	ext_buff_musk_t			get_buff_data;      //0x0204 机器人增益数据，增益状态改变后发送 
	aerial_robot_energy_t space_father_data;	//0x0205 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送 
	ext_robot_hurt_t		blood_changed_data; //0x0206  伤害状态数据，伤害发生后发送 
	ext_shoot_data_t			real_shoot_data;    //0x0207 实时射击数据，子弹发射后发送 

  client_custom_data_t robot_connection_data; //0x0301 机器人间交互数据，发送方触发发送，上限 10Hz 
} receive_judge_t;

extern receive_judge_t judge_rece_mesg;		//裁判系统各项详细数据

void  judgement_data_handler(uint8_t *p_frame);	//获取裁判系统各项详细数据
uint8_t shooter_heat_control(void);
uint8_t chassis_heat_control(void);
#endif

