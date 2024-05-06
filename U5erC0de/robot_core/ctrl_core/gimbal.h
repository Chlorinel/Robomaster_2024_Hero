#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "./base_drv/drv_motor/motor.h"

/**
 *  此部分代码作用是给robot.c承载外部数据并转换出机器人控制的直观量
 *	以 gimbal_real_state与gimbal_expt_state展示
 */
typedef struct {
  float pitch;
  float yaw;
  float roll;
} gimbal_state_t;

#define PITCH_SOFT_START_ANGLE (PI / PITCH_SOFT_START_MAX_TIME / fs_tim_freq)

#define pitch_raw_scale_max 300
#define pitch_raw_scale_mid 5800
#define pitch_raw_scale_min 4500

gimbal_state_t *get_p_gimbal_real_state(void);
gimbal_state_t *get_p_gimbal_expt_state(void);
void gimbal_init(void);
void update_gimbal_real_state(gimbal_state_t *sensor_state);

bool gimbal_smooth_start(void);

void gimbal_ctrl(float delta_yaw_ang, float delta_pitch_ang);
#endif
