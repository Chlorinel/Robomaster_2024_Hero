#ifndef _shooter_h_
#define _shooter_h_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
typedef enum {
  fire_status_idle = 0,    // 未要求发弹,则需要底盘自动补弹
  fire_status_prefill = 1, // 要求发弹但弹药暂时还没加上
  fire_status_waiting = 2, // 供弹成功,等待开火信号
  fire_status_firing = 3,  // 正在推弹
  fire_status_fired = 4,   // 发弹完成
} fire_status_t;
extern fire_status_t fire_status;

typedef enum {
  no_error = 0,
  friction_wheel_not_started, // 摩擦轮未启动而尝试发弹
} fire_error_status_t;
extern fire_error_status_t fire_error_status;

typedef struct {
  bool is_load_over;         // 是否上弹成功标识符
  bool is_dial_stalling;     // 拨弹电机是否堵转标识符
  bool is_friction_wheel_on; // 确定摩擦轮是否已开启
  bool is_in_heatlimit;      // 是否有热量限制
  bool is_ready_to_fire;     // 是否准备开启摩擦轮
  bool is_signal_to_fire;    // 是否开火
  uint32_t t0;
  uint32_t t1;

  fire_status_t _fire_status;
  fire_error_status_t _fire_error_status;
} shooter_t;
extern shooter_t shooter_debug;

bool is_ready_to_fire_rc(void);
bool is_ready_to_fire_km(void);
bool is_fire_rc(void);
bool is_fire_km(void);
bool is_fire_rc_vision(void);
bool is_fire_km_vision(void);
void get_v_ball_error(void);
void get_shoot_delay(void);
void shooter_control_loop(float front_wheel_v);

void clear_ammo_num(void);
#endif
