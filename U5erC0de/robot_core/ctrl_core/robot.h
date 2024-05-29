#ifndef _ROBOT_H_
#define _ROBOT_H_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "./base_drv/drv_motor/motor.h"
#include "./robot_core/weapon/dial.h"

extern float hs_tim_freq;
extern float fs_tim_freq;
extern float chs_tim_freq;
extern float ls_tim_freq;

typedef struct {
  // chassis_disable (chassis_info.tx_msg = 0)

  bool chassis_reset_flag;

  bool chassis_power_on_flag;

  bool chassis_dial_feeder_flag;
  bool chassis_dial_eater_flag;
  //

  uint8_t chassis_super_cap_enable_flag;

  bool gimbal_fire_start_flag;

  bool gimbal_fuck_mode_flag;

  bool vision_attrack_spin_armor_flag;
  bool vision_data_ready_flag;

  bool chassis_change_dirc_flag;

  bool chassis_dial_stall_flag;
  bool referee_kill_flag; // 1则被杀死
  bool vt_ctrl_flag;
  bool vt_config_flag;
  bool self_power_ctrl;

} robot_flag_t;

typedef struct {
  robot_flag_t robot_flag;

  enum {                   // 当前状态
    gimbal_power_off = 0,  // 机体断电
    gimbal_resetting,      // 机体重启
    gimbal_mode_switching, // 模式切换
    gimbal_power_on,       // 机体上电
  } gimbal_curr_state;

  enum {                    // 当前状态
    chassis_power_off = 0,  // 机体断电
    chassis_resetting,      // 机体重启
    chassis_mode_switching, // 模式切换
    chassis_power_on,       // 机体上电
  } chassis_curr_state;

  enum {
    online = 0,
    offline = 1, // 置1则为掉线
  } IMU_status;
  bool ctrl_mode; // 控制模式，0为遥控器，1为图传；
  uint32_t timestep;
  uint8_t is_bule_or_red; // bule:1   red:0  裁判系统：3
  bool is_imu_ctrl_yaw;
  bool is_center_fire;
  float base_speed;
  float tank_speed;
  float spin_speed;
  bool spin_dir;
  float vx, vy, wz;
  float v_yaw, v_pitch;
  bool base_offset_in;
  float expt_delta_yaw;
  float real_yaw;
  // flt	//in rad
  // 为云台期望移动的角度差,右手系
  float yaw_imu_gx;
  // flt	// 为云台imu读取的yaw速度

  enum {            // 运动模式
    _tank_mode = 0, // 坦克模式,头动底盘不跟头
    _follow_mode,   // 跟随模式,头动底盘跟头
    _spin_mode,     // 小陀螺模式
    _lob_mode,      // 吊射模式
  } move_mode,
      last_move_mode;

  uint32_t chassis_motor_status; // 置1则为掉线
  uint32_t gimbal_motor_status;  // 置1则为掉线

  struct {
    bool is_load;
    uint8_t _is_vision_ok;

    float expt_front_v_shooter;
    float expt_back_v_shooter;
    float last_real_v_shooter;
    float real_v_shooter;
    struct {
      float pitch;
      float yaw;
    } outpost_spin_armor_offset, outpost_top_armor_offset,
        base_mid_armor_offset, other_offset;
    struct {
      float *pitch;
      float *yaw;
    } shooter_offset;
    enum {
      // 打弹控制模式
      _manual_ctrl = 0, // 人工控制
      _vision_ctrl,     // 视觉控制
    } ctrl_mode;
    bool (*f_is_ready_to_fire)(void);
    bool (*f_is_fire)(void); // 开火开关
  } weapon;

  bool is_turn_on_cap; // 超级电容开启

  bool (*chassis_mode)(float expt_delta_yaw, float expt_vx, float expt_vy);
  bool (*last_chassis_mode)(float expt_delta_yaw, float expt_vx, float expt_vy);

} robot_ctrl_t;

typedef struct {
  uint8_t robot_id;                      // u8
  uint8_t robot_level;                   // u8
  uint16_t remain_HP;                    // u16
  uint16_t max_HP;                       // u16
  uint8_t mains_power_gimbal_output;     // u8
  uint8_t mains_power_chassis_output;    // u8
                                         // 64bits/8bytes
                                         // //chassis to gimbal
  uint8_t mains_power_shooter_output;    // u8
  uint8_t bullet_freq;                   // u8
  uint16_t shooter_id1_42mm_speed_limit; // u16
  float bullet_speed;                    // flt
                                         // 64bits/8bytes
                                         // //chassis to gimbal
  uint16_t shooter_barrel_heat_limit;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_id1_42mm_cooling_heat;

} referee_info_t;
extern referee_info_t referee_info;

extern robot_ctrl_t robot;

// 判断是否应该进入缓启动
#define robot_gimbal_power_on                                                  \
  (robot.gimbal_curr_state == gimbal_power_on                                  \
       ? (robot.gimbal_curr_state = gimbal_power_on)                           \
       : (robot.gimbal_curr_state = gimbal_resetting))
#define robot_gimbal_power_off (robot.gimbal_curr_state = gimbal_power_off)

#define robot_chassis_power_on                                                 \
  (robot.chassis_curr_state == chassis_power_on                                \
       ? (robot.chassis_curr_state = chassis_power_on)                         \
       : (robot.chassis_curr_state = chassis_resetting))
#define robot_chassis_power_off (robot.chassis_curr_state = chassis_power_off)

void robot_init(void);
void chassis_get_ctrl_way(void);
void robot_gimbal_tim_loop(void);
void robot_chassis_tim_loop(void);

#endif
