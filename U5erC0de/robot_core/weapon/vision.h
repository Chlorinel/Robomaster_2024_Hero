#ifndef _VISION_H
#define _VISION_H

#include "base_drv/drv_conf.h"
#include HAL_INCLUDE
#include gimbal_module_core_h

#define INIT_SHOOT_SPEED 15.6f // 默认氮素
#define MAX_SHOOT_SPEED 15.8f  // 最高氮素
#define MIN_SHOOT_SPEED 15.2f  // 最低氮素
#define VISION_COLOR 1         // bule 0,red 1

typedef enum { COLOR_BLUE = 0, COLOR_RED = 1 } e_color_t;

// 协议接口层
typedef __packed struct {
  uint8_t header; // 0x5A
  uint8_t local_color : 1;

  uint8_t buff_mode : 2;
  // uint8_t base_mode : 1;
  uint8_t rest_tracker : 1;
  // uint8_t exposure_time : 2;
  uint8_t is_play : 1;
  bool reset_lost : 1;
  uint8_t reserved : 2;

  float roll;
  float pitch;
  float yaw;

  float aim_x;
  float aim_y;
  float aim_z; // aim的三个参数主要可以提供给视觉调试时使用，可不发送数据
  uint16_t game_time;   // s
  uint32_t s_timestamp; // ms
  uint16_t checksum;
} vision_req_t;

typedef __packed struct {
  uint8_t header; // 0xA5

  uint8_t target_found : 2;
  uint8_t id_num : 3;
  uint8_t armor_num : 3;

  // 以下变量均是以世界坐标系
  float x; // 前正后负
  float y; // 左正右负
  float z; // 上正下负

  float yaw; // 当前瞄准的装甲板朝向（在imu坐标系中的角度）

  float vx;
  float vy;
  float vz;
  float v_yaw; // 小陀螺速度
  float r1;    // 当前瞄准的装甲板与车身中心的水平距离
  float r2;    // 另一侧装甲板与车身中心的水平距离
  float dz;    // 另一侧装甲板相较于此次装甲板的高度差
  uint32_t r_timestamp; //(ms)frame capture time
  uint16_t first_phase; //(ms) speed_t offset
  uint16_t checksum;
  uint8_t suggest_fire;
} vision_ctrl_t;

extern vision_ctrl_t vision_ctrl_data;
extern vision_req_t vision_request;

#define VISION_UART_REQ_SOF 0x5A
#define VISION_UART_CTRL_SOF 0xA5
#define VISION_CTRL_FRAME_LEN (sizeof(vision_ctrl_t) - 1)
#define VISION_REQ_FRAME_LEN sizeof(vision_req_t)
#define VISION_CTRL_MOVE_LEN sizeof(vision_ctrl_move_t)

#define VISION_DATA_NOERR 0x00
#define VISION_DATA_ERR 0xFF
#define VISION_NOTARGET 0xFE
#define VISION_OK 0x00
typedef enum {
  outpost_spin_armor = 0,
  outpost_top_armor = 1,
  base_mid_armor = 2,
  base_top_armor = 3,
  other = 4,
} attack_target_type_t;
#define VISION_RX_LOST_MAX 30
#define VISION_MOVE_LOST_MAX 20

float get_cur_v0(void);
void force_resfresh_cur_v0(float new_bullet_speed);
void update_cur_v0_filter(void);
bool resetTracker(bool is_enable_tracker);

void est_position_filter_init(void);
uint8_t parse_vision_data(uint8_t *buf, uint16_t len);
void set_vision_req_ang(float pitch, float yaw);
void send_vision_request(float current_roll, float current_pitch,
                         float current_yaw);
bool is_vision_offline(void);
void inc_vision_rx_lost(void);

void get_vision_aim_mode(void);
void get_vision_suggest_fire(gimbal_state_t *gimbal_expt_state,
                             gimbal_state_t *gimbal_real_state);
uint8_t get_vision_ctrl(float *pitch_ang, float *yaw_ang, float dt);
void refresh_base_mode(bool force_clean_base_mode_flag);
uint8_t get_vision_ctrl_base_mode(float *pitch_ang, float *yaw_ang, float dt);
#endif
