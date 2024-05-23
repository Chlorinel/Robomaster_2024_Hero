#include "./algorithm/filter.h"
#include "./algorithm/pid.h"
#include "./algorithm/pid_ladrc.h"
#include "./algorithm/pid_leso.h"
#include "./algorithm/util.h"
#include "./base_drv/drv_motor/motor.h"
#include "./base_drv/power_ctrl.h"
#include "./robot_core/ctrl_core/move_ctrl.h"
#include "math.h"
#include "robot.h"

// #include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/ctrl_core/robot.h"
#include <stdint.h>
#include chassis_module_core_h
#include chassis_module_motor_ctrl_h

chassis_motors_t chassis_motors = {0};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**。
 * @breif   yaw电机初始化参数
 */
float yaw_kpid_pos[3] = {480, 4, 200};
float yaw_kpid_vel[3] = {600, 0, 0};
kalman1_state i_yaw_filter;
/**
 * @brief   pitch电机的初始化函数
 */
void yaw_motor_init(void) {
  // ID		rxId		txId
  // 5		0x209		0x2ff
  yaw_motor.ctrl_mode = dual_pid_ctrl;
  motor_init(&yaw_motor, motor_DJI_GM6020, 1, yaw_kpid_pos, yaw_kpid_vel);
  yaw_motor.motor_type.reduction_ratio = 2;
  yaw_motor.pid[POS_LOOP].out_max = 1000;
  yaw_motor.pid[POS_LOOP].i_max = 2.5;
  yaw_motor.pid[RPM_LOOP].i_max = 0;
  yaw_motor.lpf_fltr.fc = 1000;
  leso_6020_init(&yaw_motor.pid_leso.leso, 0.2f, 5.0f, 5.f);
  yaw_motor.pid_leso.leso.af_z1 = 0.05f;
  kalman1_init(&i_yaw_filter, 0, 0.1, 60);
}
/**
 * @brief   yaw电机的输出控制函数,数据采用imu
 */
float error_yaw_scl = 0;
float yaw_pos_i_out_max = 300;
float yaw_pos_out_max = 300;
// bool is_imu_ctrl_yaw = 1;
float yaw_input_angle = 0;

void yaw_controller(chassis_state_t *p_chassis_expt_state,
                    chassis_state_t *p_chassis_real_state) {
  // 调试部分
  error_yaw_scl =
      rad2scl(p_chassis_expt_state->yaw - p_chassis_real_state->yaw);
  pid_update_index(yaw_motor.pid[POS_LOOP], yaw_kpid_pos);
  pid_update_index(yaw_motor.pid[RPM_LOOP], yaw_kpid_vel);
  // yaw_motor.pid[POS_LOOP].i_max = yaw_pos_i_out_max;
  // yaw_motor.pid[POS_LOOP].out_max = yaw_pos_out_max;

  // 采用pid_leso控制yaw
  float yaw_curr_rpm = robot.yaw_imu_gx * -25.f;
  kalman1_filter(&i_yaw_filter,
                 yaw_motor.real.current +
                     yaw_motor.real.speed_rpm / 13.33f * (30000.0f / 24.0f));
  float delta_yaw_angle = -get_delta_ang(
      p_chassis_expt_state->yaw, p_chassis_real_state->motor_yaw, 2 * PI);
  delta_yaw_angle -= p_chassis_real_state->wz / 1000 * 0.012f;

  // get_motor_output.leso.pos_and_rpm_loop(&yaw_motor, delta_yaw_angle,
  // yaw_curr_rpm, i_yaw_filter.x);
  float pid_out;
  if (robot.is_imu_ctrl_yaw) {
    pid_out =
        pid_leso_dualloop(yaw_motor.pid_leso.pid, &yaw_motor.pid_leso.leso,
                          delta_yaw_angle, yaw_curr_rpm, i_yaw_filter.x);
  } else {
    pid_out =
        pid_dual_loop(yaw_motor.pid, yaw_input_angle, yaw_motor.real.speed_rpm);
  }
  yaw_motor.output = LPF_update(&yaw_motor.lpf_fltr, pid_out);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @breif  dial电机初始化参数
 */
float dial_kpid_pos[3] = {800, 0, 0};
float dial_kpid_vel[3] = {10, 1, 30};
// float dial_kpid_pos[3] = {0, 0, 0};
// float dial_kpid_vel[3] = {0, 0, 0};
/**
 * @brief   拨盘电机的初始化函数
 */
void dial_motor_init(void) {
  // ID		rxId		txId
  // 6		0x206		0x1ff
  dial_motor.ctrl_mode = dual_pid_ctrl;
  motor_init(&dial_motor, motor_DJI_M3508, 6, dial_kpid_pos, dial_kpid_vel);
}
/**
 * @brief     shooter电机的输出控制函数
 * @param     expt_angle: 期望的电机输出角度
 * @param     pos_loop_outputMAX: 位置环最大输出值
 */
void dial_controller(float pos_loop_outputMAX) {
  // 调试部分
  //    pid_update_index(dial_motor.pid[0], dial_kpid_pos);
  //    pid_update_index(dial_motor.pid[1], dial_kpid_vel);

  dial_motor.pid[POS_LOOP].out_max = pos_loop_outputMAX;
  // 进行闭环控制
  // dial_expt_abs_angle = expt_abs_angle;
  float pid_out =
      pid_dual_loop(dial_motor.pid, dial_expt_abs_angle - dial_real_abs_angle,
                    dial_motor.real.speed_rpm);
  dial_motor.output = LPF_update(&dial_motor.lpf_fltr, pid_out);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////

chassis_power_lim_ta power_a = {.max_total_out = 4.0f * 16384,
                                .vxy_ratio = 1.0f,
                                .vxy_output_max = 16384,
                                .wz_ratio = 1.0f,
                                .wz_output_max = 16384};
pid_struct_t power_base_ratio_pid = {
    .kp = 0.008f, .ki = 0.001f, .kd = 0.f, .i_max = 1.f, .out_max = 1.f};
pid_struct_t power_limit_pid_Nban = {
    .kp = 0.06f, .ki = 1.001f, .kd = 0.f, .i_max = 0.1f, .out_max = 1.f};
uint8_t error[2] = {0};
void self_power_ctrl(
    chassis_power_lim_ta *chassis_power_lim) // 输出chassis_power_lim
{

  float result_ratio, power_base_ratio;
  chassis_power_lim->max_total_out =
      4.0f * C620_OUTPUT_MAX; // 后面需要改,估计会寄

  power_base_ratio = pid_calc(&power_base_ratio_pid,
                              game_robot_status.chassis_power_limit * 1.2f,
                              power_heat_data.chassis_power);
  LIMIT_MIN_MAX(power_base_ratio, 0.f, 1.f);

  result_ratio = pid_calc(&power_limit_pid_Nban,
                          power_heat_data.chassis_power_buffer, 54.f) *
                 power_base_ratio;
  // result_ratio = pid_calc(&power_unlimit_pid_Nban,
  // power_heat_data.chassis_power_buffer, POWER_BUFFER_PEAK * 0.2f) *
  // power_base_ratio;

  if (result_ratio < 0) {
    result_ratio = 0.0f;
    error[0] += 1;
  } else if (isnan(result_ratio) == 1) {
    result_ratio = 0.1f;
    error[1] += 1;
    power_base_ratio_pid.err[0] = 0;
    power_base_ratio_pid.err[1] = 0;
  }
  chassis_power_lim->power_limit_ratio = result_ratio;
}

/////

/**
 * @breif  wheel电机初始化参数
 */
float wheel_kpid_vel[3] = {8, 0.8, 0};

pid_struct_t vx_offset_pid = {
    .kp = 1.4, .ki = 0.1, .i_max = 0.4, .out_max = 1.5};
pid_struct_t vy_offset_pid = {
    .kp = 1.4, .ki = 0.1, .i_max = 0.4, .out_max = 1.5};
/**
 * @brief   拨盘电机的初始化函数,内含电机初始化、位置控制初始化、功率限制初始化
 */
void wheel_motor_init(void) {
  // ID		rxId		txId
  // 1		0x201		0x1ff
  left_front_motor.ctrl_mode = single_pid_ctrl;
  motor_init(&left_front_motor, motor_DJI_M3508, 2, NULL, wheel_kpid_vel);

  // 2		0x202		0x1ff
  right_front_motor.ctrl_mode = single_pid_ctrl;
  motor_init(&right_front_motor, motor_DJI_M3508, 1, NULL, wheel_kpid_vel);

  // 3		0x203		0x1ff
  left_back_motor.ctrl_mode = single_pid_ctrl;
  motor_init(&left_back_motor, motor_DJI_M3508, 4, NULL, wheel_kpid_vel);

  // 4		0x204		0x1ff
  right_back_motor.ctrl_mode = single_pid_ctrl;
  motor_init(&right_back_motor, motor_DJI_M3508, 3, NULL, wheel_kpid_vel);

  //    power_ctrl_init(&chassis_power_lim);
}
/**
 *	@brief	获取底盘四电机输出,内含位置控制、麦轮解算、功率控制
 */

float MAX_I_OUT = 100;
float error_vx, error_vy;
float set_vx = 0, set_vy = 0, set_wz = 0, set_vz = 0;
float _set_vz = 0;
float velocity_to_rpm;
float abs_max_v = 3;      // 实际最大速度
float abs_max_ctrl_v = 3; // 速度补偿后的控制速度
float k_velocity2rpm = 1;
// 需要对其进行换比例补偿,避免期望转速爆金币

float ratio_vx = 1, ratio_vy = 1;
float _set_vx;
float _set_vy;
float _set_wz;

void wheel_controller(chassis_state_t *p_chassis_expt_state,
                      chassis_state_t *p_chassis_real_state,
                      chassis_power_lim_t *pow_lim, uint8_t is_cap_enable) {
  // 调试用
  pid_update_index(left_front_motor.pid[RPM_LOOP], wheel_kpid_vel);
  pid_update_index(left_back_motor.pid[RPM_LOOP], wheel_kpid_vel);
  pid_update_index(right_front_motor.pid[RPM_LOOP], wheel_kpid_vel);
  pid_update_index(right_back_motor.pid[RPM_LOOP], wheel_kpid_vel);

  for (uint8_t i = 0; i < 4; i++) {
    all_chassis_motors[i].pid[RPM_LOOP].i_max = MAX_I_OUT;
  }

  // 速度控制

  error_vx =
      NL_Deadband(p_chassis_expt_state->vx - p_chassis_real_state->vx, 0.01, 2);
  error_vy =
      NL_Deadband(p_chassis_expt_state->vy - p_chassis_real_state->vy, 0.01, 2);
  if (p_chassis_expt_state->vx != 0) {
    vx_offset_pid.i_out = 0;
    vy_offset_pid.i_out = 0;
  }

  if (sign(vx_offset_pid.i_out) != sign(p_chassis_expt_state->vx) &&
      p_chassis_expt_state->vx != 0)
    vx_offset_pid.i_out = 0;
  if (sign(vy_offset_pid.i_out) != sign(p_chassis_expt_state->vy) &&
      p_chassis_expt_state->vy != 0)
    vy_offset_pid.i_out = 0;

  // // 乘积速度环稳定速度
  if (robot.robot_flag.chassis_super_cap_enable_flag &&
      robot.move_mode != _tank_mode) {
    _set_vx = ratio_vx * (p_chassis_expt_state->vx);
    _set_vy = ratio_vy * (p_chassis_expt_state->vy);
    _set_wz = p_chassis_expt_state->wz;
  } else {
    _set_vx = ratio_vx * (p_chassis_expt_state->vx +
                          pid_calc(&vx_offset_pid, error_vx, 0) *
                              (1 + fabs(p_chassis_expt_state->vx)));
    _set_vy = ratio_vy * (p_chassis_expt_state->vy +
                          pid_calc(&vy_offset_pid, error_vy, 0) *
                              (1 + fabs(p_chassis_expt_state->vy)));
    _set_wz = p_chassis_expt_state->wz;
  }
  float nonlinear_C =
      (1.f + 0.5060f * expf(-3.89638f * fabsf(p_chassis_real_state->wz))) /
      0.98588f;
  _set_vz =
      (_set_wz * ((AxleTrack + Wheelbase) / 2.f * nonlinear_C)); // 单位微弧

  // 矫正下转速
  velocity_to_rpm = (9.549296f / R_McEnham) * (3591.0f / 187.0f);
  abs_max_v = 8000.f / velocity_to_rpm;
  extern float ratio_x, ratio_y;
  abs_max_ctrl_v =
      max(ratio_x, ratio_y) + max(vx_offset_pid.out_max * (1 + ratio_x),
                                  vy_offset_pid.out_max * (1 + ratio_y));
  k_velocity2rpm = abs_max_v / abs_max_ctrl_v;
  set_vx = _set_vx * k_velocity2rpm;
  set_vy = _set_vy * k_velocity2rpm;
  set_vz = _set_vz * k_velocity2rpm;

  left_front_expt_speed = velocity_to_rpm * (set_vx - set_vy - set_vz);
  left_back_expt_speed = velocity_to_rpm * (set_vx + set_vy - set_vz);
  right_back_expt_speed = velocity_to_rpm * (-set_vx + set_vy - set_vz);
  right_front_expt_speed = velocity_to_rpm * (-set_vx - set_vy - set_vz);

  // 速度环

  float total_out;

  for (uint8_t i = 0; i < 4; i++) {
    float pid_out = pid_calc(&all_chassis_motors[i].pid[RPM_LOOP],
                             all_chassis_motors[i].expt.speed_rpm,
                             all_chassis_motors[i].real.speed_rpm);
    all_chassis_motors[i].output =
        LPF_update(&all_chassis_motors[i].lpf_fltr, pid_out);

    total_out += all_chassis_motors[i].output;
  }
  /////////////////////////////////////////////////////////////////////
  if (robot.robot_flag.self_power_ctrl) {
    float limit_ratio = 0;
    self_power_ctrl(&power_a);
    if (total_out > power_a.max_total_out) {
      limit_ratio = power_a.max_total_out / total_out;
    } else {
      limit_ratio = 1.0f;
    }
    for (uint8_t i = 0; i < 4; i++) {
      all_chassis_motors[i].output = limit_ratio *
                                     all_chassis_motors[i].output *
                                     power_a.power_limit_ratio;
    }
  } else {
    /////////////////////////////////////////////////////////////////////

    for (uint8_t i = 0; i < 4; i++) {
      pow_lim->raw_expt_Iq[i] = ecd2iq(all_chassis_motors[i].output);
      pow_lim->expt_omega[i] =
          (float)((all_chassis_motors[i].expt.speed_rpm) * 2 * PI / 60);
      pow_lim->real_Iq[i] = ecd2iq(all_chassis_motors[i].real.current);
      pow_lim->real_omega[i] =
          (float)((all_chassis_motors[i].real.speed_rpm) * 2 * PI / 60);
    }

    // //
    chassis_power_distributor(pow_lim, is_cap_enable);

    for (uint8_t i = 0; i < 4; i++) {
      all_chassis_motors[i].output = iq2ecd(pow_lim->set_Iq[i]);
    }
  }
  /////////////////////////////////////////////////////////////////////
}
//
//
///
/////
///////
//////
////////
//////////
//////////
////////////
/////////////

/**根据电机功率模型求解原始设定值所需要的功率**/
// float total_power = 0;

// for (uint8_t i = 0; i < 4; i++) {
//   float expt_omega =
//   rpm2radps((float)all_chassis_motors[i].expt.speed_rpm); float expt_Iq =
//   ecd2iq(all_chassis_motors[i].output); pow_lim->raw_power[i] =
//   get_M3508_power(expt_omega, expt_Iq);

//   total_power += pow_lim->raw_power[i];
// }
// /**按功率占比分配**/
// float power_limit_ratio;
// if (total_power > pow_lim->power_limit) {
//   power_limit_ratio = pow_lim->power_limit / total_power;
// } else {
//   power_limit_ratio = 1.0f;
// }
// /**按分配功率逆向解算应当设定的角速度**/
// for (uint8_t i = 0; i < 4; i++) {
//   pow_lim->set_power[i] = pow_lim->raw_power[i] * power_limit_ratio;
//   int dirc = sign(all_chassis_motors[i].output);
//   float expt_omega =
//   rpm2radps((float)all_chassis_motors[i].expt.speed_rpm); float real_omega
//   = rpm2radps((float)all_chassis_motors[i].real.speed_rpm); float real_Iq =
//   ecd2iq(all_chassis_motors[i].real.current); pow_lim->real_power[i] =
//   get_M3508_power(real_omega, real_Iq);

//   float set_power_offset = pid_calc(&set_power_offset_pid[i], //
//                                     pow_lim->set_power[i],    //
//                                     pow_lim->real_power[i]);
//   float set_Iq = get_M3508_Iq(pow_lim->set_power[i] + set_power_offset, //
//                               real_omega,                               //
//                               dirc);
//   set_Iq = CLAMP0(set_Iq, MAX_CURRENT_LIMIT);
//   all_chassis_motors[i].output = iq2ecd(set_Iq);
//   if (dirc >= 0) {
//     LIMIT_MIN_MAX(all_chassis_motors[i].output, 0,
//                   all_chassis_motors[i].output);
//   } else {
//     LIMIT_MIN_MAX(all_chassis_motors[i].output,
//     all_chassis_motors[i].output,
//                   0);
//   }
//   pow_lim->expt_omega[i] = expt_omega;
//   pow_lim->set_Iq[i] = set_Iq;

//   // set_power_offset_pid[i].out_max = K_out * pow_lim->set_power[i];
//   // if (dirc != sign(set_power_offset_pid[i].i_out)) {
//   //   set_power_offset_pid[i].i_out = 0;
//   // }
//   set_power_offset_pid[i] =
//       (pid_struct_t){.kp = kp,
//                      .ki = ki,
//                      .p_out = set_power_offset_pid[i].p_out,
//                      .i_out = set_power_offset_pid[i].i_out,
//                      .d_out = set_power_offset_pid[i].d_out,
//                      .i_max = K_iout * pow_lim->set_power[i],
//                      .out_max = K_out * pow_lim->set_power[i]};

//   mp_debug._[i] = (struct single_mp_debug_t){
//       .real_power = pow_lim->real_power[i],
//       .real_omega = real_omega,
//       .real_Iq = real_Iq,
//       .set_Iq = set_Iq,
//       .raw_output = all_chassis_motors[i].output,
//       .final_output = tx_msg.D[i],
//   };

// float k_currspspd = get_divide(total_currs, total_speed);
// CLAMP_MAX(left_front_motor.output, left_front_expt_speed * k_currspspd);
// CLAMP_MAX(left_back_motor.output, left_back_expt_speed * k_currspspd);
// CLAMP_MAX(right_back_motor.output, right_back_expt_speed * k_currspspd);
// CLAMP_MAX(right_front_motor.output, right_front_expt_speed * k_currspspd);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

motor_t *p_yawM(void) { return &yaw_motor; }
motor_t *p_dialM(void) { return &dial_motor; }

void shutdown_all_chassis_motor(void) {
  for (uint8_t i = 0; i < num_of_all_chassis_motors; i++) {
    clone(all_chassis_motors[i].expt, all_chassis_motors[i].real);
    all_chassis_motors[i].output = 0; // 防止各电机傻转
  }
}
