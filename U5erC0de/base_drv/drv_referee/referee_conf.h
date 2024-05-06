// #include "customer_controller.h"
#include "stdint.h"

#ifndef _REFEREE_CONF_H
#define _REFEREE_CONF_H

#define REFEREE_SYS_VERSION 2022.8.5

// 长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取,参考裁判系统串口协议
#define LEN_HEADER 5U
#define LEN_CMDID 2U
#define LEN_TAIL 2U

#define SOF_BYTE 0xA5U // 帧头

#define CMDID_OFFSET 5U
#define CMDID_H 6U
#define CMDID_L 5U

/* RFID卡类型 */
#define CARD_ATTACK ((uint8_t)0x00)
#define CARD_PROTECT ((uint8_t)0x01)
#define CARD_BLOOD_RED ((uint8_t)0x02)
#define CARD_BLOOD_BLUE ((uint8_t)0x03)
#define CARD_HEAL_RED ((uint8_t)0x04)
#define CARD_HEAL_BLUE ((uint8_t)0x05)
#define CARD_COLD_RED ((uint8_t)0x06)
#define CARD_COLD_BLUE ((uint8_t)0x07)
#define CARD_FORT ((uint8_t)0x08)

/**
 * @enum CmdID
 * @brief 命令码ID,用来判断接收的是什么数据
 */
typedef enum {
  ID_game_state = 0x0001,                ///< 比赛状态数据
  ID_game_result = 0x0002,               ///< 比赛结果数据
  ID_game_robot_HP = 0x0003,             ///< 比赛机器人存活数据
  ID_dart_status = 0x0004,               ///< 飞镖状态
  ID_event_data = 0x0101,                ///< 场地事件数据
  ID_supply_projectile_action = 0x0102,  ///< 场地补给站动作标识数据
  ID_supply_projectile_booking = 0x0103, ///< 场地补给站预约子弹数据
  ID_refee_alert = 0x0104,      ///< 裁判警告数据，警告发生后发送
  ID_dart_countdown = 0x0105,   ///< 飞镖发射口倒计时，1Hz周期发送
  ID_game_robot_state = 0x0201, ///< 机器人状态数据
  ID_power_heat_data = 0x0202,  ///< 实时功率热量数据
  ID_game_robot_pos = 0x0203,   ///< 机器人位置数据
  ID_buff_musk = 0x0204,        ///< 机器人增益数据
  ID_aerial_robot_energy = 0x0205,  ///< 空中机器人能量状态数据
  ID_robot_hurt = 0x0206,           ///< 伤害状态数据
  ID_shoot_data = 0x0207,           ///< 实时射击数据
  ID_bullet_remaining_num = 0x0208, ///< 弹丸剩余发射数
  ID_RFID_status = 0x0209,          ///< 机器人RFID状态
  ID_dart_client_data = 0x020A,     ///< 飞镖机器人客户端指令数据
  ID_custom_data = 0x0301           ///< 机器人交互数据
} CmdID;

typedef __packed struct {
  uint8_t sof;
  uint16_t data_len;
  uint8_t seq;
  uint8_t crc8;
} frame_header_t;

/* 比赛状态数据：0x0001。发送频率：1Hz */
typedef __packed struct {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
} ext_game_status_t;

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送 */
typedef __packed struct {
  uint8_t winner;
} ext_game_result_t;

/* 机器人血量数据：0x0003。发送频率：1Hz */
typedef __packed struct {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz周期发送，发送范围：所有机器人
 */
typedef __packed struct {
  uint8_t F1_zone_status : 1;
  uint8_t F1_zone_buff_debuff_status : 3;
  uint8_t F2_zone_status : 1;
  uint8_t F2_zone_buff_debuff_status : 3;
  uint8_t F3_zone_status : 1;
  uint8_t F3_zone_buff_debuff_status : 3;
  uint8_t F4_zone_status : 1;
  uint8_t F4_zone_buff_debuff_status : 3;
  uint8_t F5_zone_status : 1;
  uint8_t F5_zone_buff_debuff_status : 3;
  uint8_t F6_zone_status : 1;
  uint8_t F6_zone_buff_debuff_status : 3;
  uint16_t red1_bullet_left;
  uint16_t red2_bullet_left;
  uint16_t blue1_bullet_left;
  uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

/* 场地事件数据：0x0101。发送频率：3Hz */
typedef __packed struct {
  uint32_t event_type;
} ext_event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人 */
typedef __packed struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：己方警告发生后发送 */
typedef __packed struct {
  uint8_t level;
  uint8_t offending_robot_id;
  uint8_t count;
} ext_referee_warning_t;

/* 飞镖发射口倒计时：cmd_id
 * (0x0105)。发送频率：3Hz周期发送，发送范围：己方机器人 */
typedef __packed struct {
  uint8_t dart_remaining_time;
  uint16_t dart_info;
} ext_dart_remaining_time_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz */
typedef __packed struct {
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management_gimbal_output : 1;
  uint8_t power_management_chassis_output : 1;
  uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

/* 实时功率热量数据：0x0202。发送频率：50Hz */
typedef __packed struct {
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz */
typedef __packed struct {
  float x;
  float y;
  float angle;
} ext_game_robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz */
typedef __packed struct {
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} ext_buff_t;

/* 空中机器人能量状态：0x0205。发送频率：10Hz */
typedef __packed struct {
  uint8_t airforce_status;
  uint8_t time_remain;
} aerial_robot_energy_t;

/* 伤害状态：0x0206。发送频率：伤害发生后发送 */
typedef __packed struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* 实时射击信息：0x0207。发送频率：射击后发送 */
typedef __packed struct {
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} ext_shoot_data_t;

/* 子弹剩余发射数：0x0208。发送频率：10Hz周期发送，所有机器人发送 */
// typedef __packed struct
//{
//     uint16_t projectile_allowance_17mm;
//     uint16_t projectile_allowance_42mm;
//     uint16_t remaining_gold_num;
// } projectile_allowance_t;
typedef __packed struct {
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* 机器人RFID状态：0x0209。发送频率：1Hz，发送范围：单一机器人 */
typedef __packed struct {
  uint32_t rfid_status;
} ext_rfid_status_t;

/* 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人 */
typedef __packed struct {
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 己方机器人位置坐标：0x020B*/
typedef __packed struct {
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;
} ground_robot_position_t;

// 敌方机器人标记进度：0x020C
typedef __packed struct {
  uint8_t mark_hero_progress;
  uint8_t mark_engineer_progress;
  uint8_t mark_standard_3_progress;
  uint8_t mark_standard_4_progress;
  uint8_t mark_standard_5_progress;
  uint8_t mark_sentry_progress;
} radar_mark_data_t;

// 哨兵兑换控制数据：0x020D
typedef __packed struct {
  uint32_t sentry_info;
} sentry_info_t;

// 雷达双倍易伤：0x020E
typedef __packed struct {
  uint8_t radar_info;
} radar_info_t;

typedef __packed struct {
  uint8_t delete_type;
  uint8_t layer;
} interaction_layer_delete_t;

#endif