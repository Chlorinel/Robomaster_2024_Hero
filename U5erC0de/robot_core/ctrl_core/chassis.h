#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "./base_drv/drv_conf.h"
#include <stdint.h>
#include HAL_INCLUDE

/**
 *  此部分代码作用是给robot.c承载外部数据并转换出机器人控制的直观量
 *	以 chassis_real_state与chassis_expt_state展示
 */

#define GIMBAL_MID_SCALE 2138
#define GIMBAL_MID_ANGLE 2.98

// 一些机械常数：
#define R_McEnham 0.07625f
#define AxleTrack 0.4210f
#define Wheelbase 0.3466f
#define REDUCTION_RATIO (3591.0f / 187.0f) ///< reduction ratio of M3508
typedef struct {
  float yaw;
  float motor_yaw;
  // 云台相对底盘的yaw,为左正右负,由yaw电机调节控制
  float c_yaw, c_pitch, c_roll;
  // 底盘的yaw,为左正右负,由底盘电机调节控制
  float x, vx; // 底盘的移动速度(来自云台期望且已经过转换)为正即为前进,反之后退
  float y, vy; // 底盘的移动速度(来自云台期望且已经过转换)为正即为左移,反之右移
  float wz;    // 给正值底盘逆时针旋转
} chassis_state_t;

chassis_state_t *get_p_chassis_real_state(void);
chassis_state_t *get_p_chassis_expt_state(void);

void chassis_init(void);
void update_chassis_real_state(void);

bool chassis_lob_mode(float expt_delta_yaw, float expt_vx, float expt_vy);
bool chassis_tank_mode(float expt_delta_yaw, float expt_vx, float expt_vy);
bool chassis_follow_mode(float expt_delta_yaw, float expt_vx, float expt_vy);
bool chassis_spin_mode(float expt_delta_yaw, float expt_vx, float expt_vy);
bool chassis_smooth_start(float expt_delta_yaw, float expt_vx, float expt_vy);
bool chassis_disable(float expt_delta_yaw, float expt_vx, float expt_vy);

void chassis_ctrl(float expt_delta_yaw, float expt_vx, float expt_vy,
                  float expt_wz);
float set_spin_speed(uint16_t power_lim);

bool is_chassis_expt_stop(void);
bool is_chassis_real_stop(void);
#endif
