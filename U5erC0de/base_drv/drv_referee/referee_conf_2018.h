#ifndef _REFEREE_CONF_H
#define _REFEREE_CONF_H
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#define REFEREE_SYS_VERSION 2018
//uart配置:
//波特率115200,数据位8位,停止位1,校验位无,流控制无
//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define	LEN_HEADER    5U        //帧头长
#define	LEN_CMDID     2U        //命令码长度
#define	LEN_TAIL      2U	      //帧尾CRC16

#define	SOF_BYTE			0xA5U     //帧头

#define	CMDID_OFFSET	5U
#define	CMDID_H				6U
#define	CMDID_L				5U

/* RFID卡类型 */
#define    CARD_ATTACK        ((uint8_t)0x00)
#define    CARD_PROTECT       ((uint8_t)0x01)
#define    CARD_BLOOD_RED     ((uint8_t)0x02)
#define    CARD_BLOOD_BLUE    ((uint8_t)0x03)
#define    CARD_HEAL_RED      ((uint8_t)0x04)
#define    CARD_HEAL_BLUE     ((uint8_t)0x05)
#define    CARD_COLD_RED      ((uint8_t)0x06)
#define    CARD_COLD_BLUE     ((uint8_t)0x07)
#define    CARD_FORT          ((uint8_t)0x08)

/**
  * @enum CmdID
  * @brief 命令码ID,用来判断接收的是什么数据
  */
typedef enum
{ 
	ID_GameRobotState   = 0x0001,///< 比赛机器人状态						//发送频率：10Hz					
	ID_RobotHurt 	   		= 0x0002,///< 伤害数据									//发送频率：受伤时发送
	ID_shootData   			= 0x0003,///< 实时射击数据							//发送频率：发弹时发送
	ID_PowerHeatData		= 0x0004,///< 实时功率与热量数据				//发送频率：50Hz
	ID_event_data  			= 0x0005,///< 实时场地交互数据 					//发送频率：检测到有RFID卡时10Hz周期发送
	ID_GameResult  			= 0x0006,///< 比赛结果数据							//发送频率：比赛结束时发送一次
	ID_BuffMusk					= 0x0007,///< Buff状态									//发送频率：Buff改变时发送一次
	ID_GameRobotPos			= 0x0008,///< 机器人位置与枪口朝向信息	//发送频率：50Hz
	ID_ShowData					= 0x0100,///< 自定义数据,显示在操作界面	//发送频率：限频10Hz
} CmdID_t;

typedef struct
{
	uint8_t  sof;
	uint16_t data_len;
  uint8_t  seq;
  uint8_t  crc8;
}frame_header_t;

/* 比赛机器人状态：0x0001。发送频率：10Hz */
typedef struct
{
	uint16_t stageRemainTime;			//该阶段剩余时间
	uint8_t gameProgress;					//当前比赛阶段
	uint8_t robotLevel;						//机器人等级
	uint16_t remainHP;						//机器人血量
	uint16_t maxHP;								//机器人最大血量
}extGameRobotState_t;

/* 伤害数据：0x0002。发送频率：受伤时发送 */
typedef struct
{
	uint8_t armorType:4;					//受伤害装甲ID
	uint8_t hurtType:4;						//血量变化类型
} extRobotHurt_t;

/* 实时射击信息：0x0003。发送频率：发弹时发送 */
typedef struct
{
	uint8_t bulletType;						
	uint8_t bulletFreq;						//单位:发每秒
	float bulletSpeed;						//单位:米每秒
} extshootData_t;

/* 实时功率与热量数据：0x0004。发送频率：50Hz*/
typedef struct
{
	float chassisVolt;						//单位:伏特
	float chassisCurrent;					//单位:安培
	float chassisPower;						//单位:瓦特
	float chassisPowerBuffer;			//单位:瓦特
	uint16_t shooterHeat0;
	uint16_t shooterHeat1;
} extPowerHeatData_t;

/* 实时场地交互数据：0x0005。发送频率：检测到有RFID卡时10Hz周期发送 */
typedef struct
{
	uint8_t cardType;
	uint8_t cardIdx;
} exteventdata_t;

/* 比赛结果数据：0x0006。发送频率：比赛结束时发送一次 */
typedef struct
{
    uint8_t winner;
} extGameResult_t;

/* Buff状态：0x0007。发送频率：Buff改变时发送一次 */
typedef struct
{
	uint16_t buffMusk;
}extBuffMusk_t;

/* 机器人位置与枪口朝向信息:0x0008。发送频率：50Hz*/
typedef struct
{
	float x;
	float y;
	float z;
	float yaw;
} extGameRobotPos_t;

/* 自定义数据：0x0100。发送频率：限频10Hz */
typedef struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
} extShowData_t;

#define CMDID_TABLE_COL 8U
#define CMDID_TABLE_ROW 2U

static uint8_t cmdid_len_table[CMDID_TABLE_ROW][CMDID_TABLE_COL] = 
{
//0x00xx系列
	{
		sizeof(extGameRobotState_t),sizeof(extRobotHurt_t),sizeof(extshootData_t),sizeof(extPowerHeatData_t),sizeof(exteventdata_t),
		sizeof(extGameResult_t),sizeof(extBuffMusk_t),sizeof(extGameRobotPos_t)
	},
//0x01xx系列
	{
		sizeof(extShowData_t),
	},
};
#endif
