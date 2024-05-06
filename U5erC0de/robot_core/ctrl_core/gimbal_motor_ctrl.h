#ifndef _GIMBAL_MOTOR_CTRL_H_
#define _GIMBAL_MOTOR_CTRL_H_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include gimbal_module_core_h
#include chassis_module_motor_ctrl_h
#include "./base_drv/drv_motor/motor.h"

/**
 *  此部分代码作用是将直观量变换为电机控制的抽象量
 *  通常含有从半角度坐标(-PI~PI)变换到全角度(角度量程从0~2*PI)的操作
 */

typedef union {
  struct {
    motor_t _pitch_motor;
    motor_t _left_friction_motor; // friction(left)
    motor_t _right_friction_motor;
    motor_t _firectrl_motor;
  };
  motor_t _all_gimbal_motors[4];
} gimbal_motors_t;

// 偷懒用
#define num_of_all_gimbal_motors (sizeof(all_gimbal_motors) / sizeof(motor_t))

#define pitch_motor gimbal_motors._pitch_motor
#define left_friction_motor gimbal_motors._left_friction_motor
#define right_friction_motor gimbal_motors._right_friction_motor

#define firectrl_motor gimbal_motors._firectrl_motor
#define all_gimbal_motors gimbal_motors._all_gimbal_motors

#define pitch_expt_scale (pitch_motor.expt.raw_scale) // pitch轴期望角度
#define pitch_real_scale (pitch_motor.real.raw_scale) // pitch轴实际角度
#define pitch_expt_rel_angle (pitch_motor.expt.rel_angle) // pitch轴线对角度
#define pitch_real_rel_angle (pitch_motor.real.rel_angle) // pitch轴线对角度

#define lfm_expt_speed (left_friction_motor.expt.speed_rpm) // 左摩擦轮期望转速
#define lfm_real_speed (left_friction_motor.real.speed_rpm) // 左摩擦轮实际转速
#define lfm_real_currs (left_friction_motor.real.current) // 左摩擦轮实际电流

#define rfm_expt_speed (right_friction_motor.expt.speed_rpm) // 右摩擦轮期望转速
#define rfm_real_speed (right_friction_motor.real.speed_rpm) // 右摩擦轮实际转速
#define rfm_real_currs (right_friction_motor.real.current) // 右摩擦轮实际电流

#define fc_expt_abs_angle (firectrl_motor.expt.abs_angle) // 控火电机期望角度
#define fc_real_abs_angle (firectrl_motor.real.abs_angle) // 控火电机实际角度
#define fc_real_speed (firectrl_motor.real.speed_rpm) // 控火电机实际角度

#define fc_real_currs (firectrl_motor.real.current) // 控火电机实际电流

// 功能电机初始化函数
void pitch_motor_init(void);
void shooter_motor_init(void);
void firectrl_motor_init(void);

// 模块控制函数
void pitch_controller(gimbal_state_t *p_gimbal_expt_state,
                      gimbal_state_t *p_gimbal_real_state);
void shooter_controller(void);
void firectrl_controller(void);

// 其他功能函数
motor_t *p_fcM(void);
motor_t *p_lfwM(void);
motor_t *p_rfwM(void);
gimbal_state_t *fetch_robot_coordinate_system(gimbal_state_t *offset_state);
void update_shooter_motor_vel(float *expt_v_shooter);
bool get_fw_speed_error(void);
void get_shoot_delay(void);
bool f_is_friction_wheel_on(void);
#define firectrl_lock (firectrl_controller(0))
void shutdown_shooter_motor(void);
void shutdown_all_gimbal_motor(void);
#endif