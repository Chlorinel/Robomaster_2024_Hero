#ifndef _CHASSIS_MOTOR_CTRL_H_
#define _CHASSIS_MOTOR_CTRL_H_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include chassis_module_core_h
#include "../base_drv/drv_motor/motor.h"
#include "../base_drv/power_ctrl.h"

/**
 *  此部分代码作用是将直观量变换为电机控制的抽象量
 *  通常含有从半角度坐标(-PI~PI)(标准化角度坐标系)变换到全角度(0~2*PI)(非标准化角度坐标系)的操作
 */

typedef union {
  struct {
    motor_t _left_front_motor;
    motor_t _right_front_motor;
    motor_t _left_back_motor;
    motor_t _right_back_motor;
    motor_t _yaw_motor;
    motor_t _dial_motor;
  };
  motor_t _all_chassis_motors[6];
} chassis_motors_t;
extern chassis_motors_t chassis_motors;
// typedef struct {
//   float max_total_out; ///< total maximum output of motor,set to >=
//   ///< 4*C620_OUTPUT_MAX to disable
//   float power_limit;          // 最大额定输入功率
//   float chassis_power_buffer; // 从裁判系统读取,亦可软件设定
//   float raw_power[4];         // 原PID闭环所设定的功率
//   float real_power[4];        // 根据电机数据拟合出的实际功率
//   float set_power[4];         // 最终均分后所得的功率
//   float expt_omega[4];
//   float set_Iq[4];
// } chassis_power_lim_t;

// 偷懒用
#define num_of_all_chassis_motors sizeof(all_chassis_motors) / sizeof(motor_t)

#define left_front_motor chassis_motors._all_chassis_motors[0]
#define left_back_motor chassis_motors._all_chassis_motors[2]
#define right_front_motor chassis_motors._all_chassis_motors[1]
#define right_back_motor chassis_motors._all_chassis_motors[3]
#define yaw_motor chassis_motors._all_chassis_motors[4]
#define dial_motor chassis_motors._dial_motor
#define all_chassis_motors chassis_motors._all_chassis_motors

#define left_front_expt_speed (left_front_motor.expt.speed_rpm)
#define left_back_expt_speed (left_back_motor.expt.speed_rpm)
#define right_front_expt_speed (right_front_motor.expt.speed_rpm)
#define right_back_expt_speed (right_back_motor.expt.speed_rpm)

#define left_front_real_speed (left_front_motor.real.speed_rpm)
#define left_back_real_speed (left_back_motor.real.speed_rpm)
#define right_front_real_speed (right_front_motor.real.speed_rpm)
#define right_back_real_speed (right_back_motor.real.speed_rpm)

#define left_front_expt_currs (left_front_motor.expt.current)
#define left_back_expt_currs (left_back_motor.expt.current)
#define right_front_expt_currs (right_front_motor.expt.current)
#define right_back_expt_currs (right_back_motor.expt.current)

#define lf_real_currs (left_front_motor.real.current)
#define lb_real_currs (left_back_motor.real.current)
#define rf_real_currs (right_front_motor.real.current)
#define rb_real_currs (right_back_motor.real.current)

#define yaw_expt_rel_angle (yaw_motor.expt.rel_angle)
#define yaw_real_rel_angle (yaw_motor.real.rel_angle)

#define dial_expt_abs_angle (dial_motor.expt.abs_angle)
#define dial_real_abs_angle (dial_motor.real.abs_angle)
#define dial_real_currs (dial_motor.real.current)

#define delta_scale                                                            \
  (p_motor->motor_type.measure_max - p_motor->motor_type.measure_min)
#define pm_is_offline (p_motor->is_offline)
#define pm_real_scale (p_motor->real.raw_scale)
#define pm_real_last_scale (p_motor->real.last_raw_scale)
#define pm_real_speed (p_motor->real.speed_rpm)
#define pm_real_currs (p_motor->real.current)
#define pm_real_temp (p_motor->real.tempture)
#define pm_real_rel_angle (p_motor->real.rel_angle)
#define pm_real_last_rel_angle (p_motor->real.last_rel_angle)
#define pm_real_abs_angle (p_motor->real.abs_angle)
#define pm_RD_ratio (p_motor->motor_type.reduction_ratio)

// 功能电机初始化函数
void yaw_motor_init(void);
void dial_motor_init(void);
void wheel_motor_init(void);

// 模块控制函数
void yaw_controller(chassis_state_t *chassis_expt_state,
                    chassis_state_t *chassis_real_state);
void dial_controller(float pos_loop_outputMAX);

void wheel_controller(chassis_state_t *p_chassis_expt_state,
                      chassis_state_t *p_chassis_real_state,
                      chassis_power_lim_t *pow_lim, uint8_t is_cap_enable);

// 其他功能函数

void set_motor_mid_position(motor_t *p_motor, uint16_t mid_scale);

motor_t *p_dialM(void);
void shutdown_all_chassis_motor(void);
/*功率限制参数结构体*/
typedef struct {
  float max_total_out;  ///< total maximum output of motor,set to
                        ///< >= 4*C620_OUTPUT_MAX to disable
  float vxy_ratio;      ///< power reduction ratio of vx and vy output
  float vxy_output_max; ///< maximum output of vx or vy
  float wz_ratio;       ///< power reduction ratio of wz output(unused in
                        ///< chassis_pid_ctrl)
  float wz_output_max;  ///< maximum output of wz(unused in chassis_pid_ctrl)
  float power_limit_ratio;
} chassis_power_lim_ta;

void self_power_ctrl(chassis_power_lim_ta *chassis_power_lim);
#endif