#include "./algorithm/util.h"
#include "./base_drv/drv_can.h"
#include "./base_drv/drv_conf.h"
#include "./base_drv/drv_motor/motor.h"
#include "./robot_core/weapon/dial.h"
#include "math.h"
#include "string.h"
#include "tim.h"

#include chassis_module_core_h
#include chassis_module_motor_ctrl_h
#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/power_ctrl.h"
#include "./robot_core/ctrl_core/move_ctrl.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 控制参数
chassis_state_t chassis_expt_state = {0};
chassis_state_t chassis_real_state = {0};
chassis_power_lim_t chassis_power_lim = {.referee_power_buffer = 60};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief	其他函数
 */
chassis_state_t *get_p_chassis_expt_state(void) { return &chassis_expt_state; }
chassis_state_t *get_p_chassis_real_state(void) { return &chassis_real_state; }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 开机头的角度要求向着车前方

/**
 * @brief   底盘运动控制初始化
 * 			内部涉及到各电机的初始化、各模式pid初始化
 * */

void chassis_init(void) {
  extern chassis_motors_t chassis_motors;
  // 底盘四电机初始化
  wheel_motor_init();
  uint32_t wheels_fliter_list[4] = {
      rx_stdid_of(left_front_motor),
      rx_stdid_of(left_back_motor),
      rx_stdid_of(right_front_motor),
      rx_stdid_of(right_back_motor),
  };
  can_user_init(&WHEELS_CAN, wheels_fliter_list, WHEELS_FIFO, WHEELS_FLTRNUM);

  HAL_Delay(20);

  // 云台yaw轴电机、拨弹盘电机初始化
  yaw_motor_init();
  set_motor_mid_position(&yaw_motor, GIMBAL_MID_SCALE);
  dial_motor_init();
  uint32_t yaw_and_dial_fliter_list[4] = {
      rx_stdid_of(yaw_motor),
      rx_stdid_of(dial_motor),
      CAP_RESPONSE_ID,
  };
  can_user_init(&YAW_AND_DIAL_CAN, yaw_and_dial_fliter_list, YAW_AND_DIAL_FIFO,
                YAW_AND_DIAL_FLTRNUM);
  HAL_Delay(20);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 	更新chasiisi_real_state,获得底盘实际姿态,在imu的中断中调用
 * @note
 */
#include "./algorithm/filter.h"
#include "imu.h"

chassis_state_t sensor_state = {0};

void update_chassis_real_state(void) {
  extern chassis_motors_t chassis_motors;
  // 获得底盘编码器数据
  chassis_state_t wheel_state = *mecanum_forward();
  sensor_state.vx =
      wheel_state
          .vx; // kalman12_filter(&chassis_axis_x_ks, &velocity_measure[0]);
  sensor_state.vy =
      wheel_state
          .vy; // kalman12_filter(&chassis_axis_y_ks, &velocity_measure[1]);
  sensor_state.wz = wheel_state.wz;
  sensor_state.x = wheel_state.x;
  sensor_state.y = wheel_state.y;
  sensor_state.c_yaw = wheel_state.c_yaw;
  // sensor_state.c_pitch=_imu_eular.pitch;
  sensor_state.c_roll = 0;

  sensor_state.motor_yaw = -range_map(yaw_real_rel_angle, -PI, PI);
  sensor_state.yaw = robot.real_yaw;
  clone(chassis_real_state, sensor_state);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 	吊射模式
 * @param   expt_delta_yaw:
 * 期望控制的yaw角度变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vx:
 * 期望控制的x方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vy:
 * 期望控制的y方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @return  返回true保证 robot.curr_state处于power_on
 */
bool chassis_lob_mode(float expt_delta_yaw, float expt_vx, float expt_vy) {
  chassis_ctrl(expt_delta_yaw, expt_vx, expt_vy, 0);
  return true;
}

/**
 * @brief 	坦克模式
 * @param   expt_delta_yaw:
 * 期望控制的yaw角度变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vx:
 * 期望控制的x方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vy:
 * 期望控制的y方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @return  返回true保证 robot.curr_state处于power_on
 */
pid_struct_t chassis_tank_pid = {
    .kp = 5, .ki = 0.5, .kd = 3, .i_max = 0.05, .out_max = 6};
float expt_angle = 0;
float expt_angle1 = 0;
float tank_mode_trace_dirc;
float tank_angle1, tank_angle2;
float tank_mode_yaw_error = 0;
float tank_end_error = 0;
float xx, yy;
float chassis_expt_wz_tank = 0;
bool chassis_tank_mode(float expt_delta_yaw, float expt_vx, float expt_vy) {
  xx = expt_vx;
  yy = expt_vy;
  expt_angle1 = atan2(yy, xx);
  if (expt_vx != 0 || expt_vy != 0) {
    expt_angle = -expt_angle1;
    tank_angle1 =
        fabs(get_delta_ang(chassis_real_state.motor_yaw, expt_angle, 2 * PI));
    tank_angle2 = fabs(
        get_delta_ang(chassis_real_state.motor_yaw, (PI + expt_angle), 2 * PI));
    tank_mode_trace_dirc =
        (min(tank_angle1, tank_angle2) == tank_angle1 ? expt_angle
                                                      : (PI + expt_angle));
    tank_mode_yaw_error = (get_delta_ang(tank_mode_trace_dirc,
                                         chassis_real_state.motor_yaw, 2 * PI));
    chassis_expt_wz_tank = pid_calc(
        &chassis_tank_pid, 0, NL_Deadband(tank_mode_yaw_error, deg2rad(3), 3));

  } else {
    chassis_expt_wz_tank = 0;
  }
  expt_angle = 0;
  chassis_ctrl(expt_delta_yaw, expt_vx, expt_vy, chassis_expt_wz_tank);
  return true;
}

/**
 * @brief 	底盘跟随云台模式
 * @param   expt_delta_yaw:
 * 期望控制的yaw角度变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vx:
 * 期望控制的x方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vy:
 * 期望控制的y方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @return  返回true保证 robot.curr_state处于power_on
 */
pid_struct_t chassis_follow_pid = {
    .kp = 4, .ki = 0.5, .kd = 3, .i_max = 0.05, .out_max = 6};
float follow_mode_trace_dirc;
float delta_ang1, delta_ang2;
float follow_mode_yaw_error = 0;
float end_error = 0;
float kkkll = 3;

bool chassis_follow_mode(float expt_delta_yaw, float expt_vx, float expt_vy) {

  delta_ang1 = fabs(get_delta_ang(chassis_real_state.motor_yaw, 0, 2 * PI));
  delta_ang2 = fabs(get_delta_ang(chassis_real_state.motor_yaw, PI, 2 * PI));
  follow_mode_trace_dirc = (min(delta_ang1, delta_ang2) == delta_ang1 ? 0 : PI);

  follow_mode_yaw_error = (get_delta_ang(follow_mode_trace_dirc,
                                         chassis_real_state.motor_yaw, 2 * PI));

  end_error =
      follow_mode_yaw_error -
      range_map(get_delta_ang(chassis_real_state.motor_yaw,
                              -range_map(GIMBAL_MID_ANGLE, -PI, PI), 2 * PI),
                -PI, PI);

  float chassis_expt_wz =
      pid_calc(&chassis_follow_pid, 0,
               NL_Deadband(follow_mode_yaw_error, deg2rad(kkkll), 3));

  // wz的速度与差角量方向相反
  chassis_ctrl(expt_delta_yaw, expt_vx, expt_vy, chassis_expt_wz);

  return true;
}

/**
 * @brief 	底盘小陀螺模式
 * @param   expt_delta_yaw:
 * 期望控制的yaw角度变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vx:
 * 期望控制的x方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @param   expt_vy:
 * 期望控制的y方向的变换量,将在chassis_ctrl里累加到chassis_expt_state上
 * @return  返回true保证 robot.curr_state处于power_on
 */

float k_spin = 0.1;
float spin_speed = 10;
// 10->50w
// 12->60w
// 15->75w
// 15.5->100w
// 16 -> 105w
// 17->120w

// 18->150w
float set_spin_speed(uint16_t power_lim) {
  CLAMP(power_lim, 50, 140);
  float spin_speed_ =
      (0.00000021f * pow4of(power_lim) - 0.00006097f * pow3of(power_lim) +
       0.00453663f * pow2of(power_lim) + 0.12920262f * power_lim - 1.68644940f);
  // CLAMP(spin_speed_, 0, 18);
  CLAMP(spin_speed_, 0, 18);
  return spin_speed_;
}
bool chassis_spin_mode(float expt_delta_yaw, float expt_vx, float expt_vy) {
  extern chassis_power_lim_t chassis_power_lim;
  float chassis_expt_wz;

  // 依据移动来选择
  //  if (chassis_power_lim.weight_of_wz < k_spin) // 0.03;  // 0.01;
  if (0)
    chassis_expt_wz = 0;
  // 小陀螺时要x/y需求过大时,则自动切换到坦克模式

  else { // 小陀螺时要x/y需求较小时,则吃满功率小陀螺
    spin_speed = set_spin_speed(game_robot_status.chassis_power_limit);
    // chassis_expt_wz =
    // sign(chassis_real_state.wz) * fabsf(spin_speed) * robot.spin_speed;
    if (robot.spin_dir) {
      chassis_expt_wz =

          -1 * fabsf(spin_speed) * robot.spin_speed;
    } else {
      chassis_expt_wz =

          1 * fabsf(spin_speed) * robot.spin_speed;
    }
  }
  chassis_ctrl(expt_delta_yaw, expt_vx, expt_vy, chassis_expt_wz);

  return true;
}
/**
 * @brief 	缓启动
 * @return	true:缓启动成功
 * @return 	返回true保证 robot.curr_state处于power_on
 */
float YAW_SOFT_START_MAX_TIME = 0.5;
float ERROR_YAW_SOFT_START = 10;
#define YAW_SOFT_START_OMEGA (PI / YAW_SOFT_START_MAX_TIME)
bool chassis_smooth_start(float expt_delta_yaw, float expt_vx, float expt_vy) {
  float delta_ang1 =
      fabs(get_delta_ang(chassis_real_state.motor_yaw, 0, 2 * PI));
  float delta_ang2 =
      fabs(get_delta_ang(chassis_real_state.motor_yaw, PI, 2 * PI));
  follow_mode_trace_dirc = (min(delta_ang1, delta_ang2) == delta_ang1 ? 0 : PI);

  float error_yaw = 0;
  extern robot_ctrl_t robot; //.last_chassis_mode

  error_yaw = get_delta_ang(
      0, NL_Deadband(chassis_expt_state.motor_yaw, deg2rad(10), 3), 2 * PI);
  if (robot.last_chassis_mode == chassis_spin_mode) {
    if (sign(chassis_real_state.wz) * sign(chassis_real_state.motor_yaw) < 0) {
      error_yaw = 2 * PI + sign(chassis_real_state.wz) * error_yaw;
    }
  }

  float chassis_expt_wz = pid_calc(&chassis_follow_pid, 0, error_yaw);

  CLAMP_MAX(chassis_expt_wz, YAW_SOFT_START_OMEGA);

  chassis_ctrl(expt_delta_yaw, 0, 0,
               chassis_expt_wz); // 判断缓启动是否完成

  if (fabs(get_delta_ang(follow_mode_trace_dirc, chassis_real_state.motor_yaw,
                         2 * PI) < deg2rad(ERROR_YAW_SOFT_START))) {
    return true;
  }

  return false;
}

/**
 * @brief 	断控
 * @param   :3个float只是为了统一函数形式(
 * @return 	返回false保证 robot.curr_state处于power_off
 */
bool chassis_disable(float expt_delta_yaw, float expt_vx, float expt_vy) {
  chassis_expt_state.x = chassis_real_state.x;
  chassis_expt_state.y = chassis_real_state.y;
  chassis_expt_state.c_yaw = chassis_real_state.c_yaw;
  chassis_expt_state.vx = 0;
  chassis_expt_state.vy = 0;
  chassis_expt_state.wz = 0;
  chassis_ctrl(0, 0, 0, 0);
  shutdown_all_chassis_motor();
  return false;
}

/**
 * @brief	底盘控制
 * @param   expt_delta_yaw:
 * 期望控制的yaw角度变换量,方向逆时针正,将累加到chassis_expt_state上
 * @param   expt_vx:
 * 期望控制的x方向的变换量,方向前正后负,将累加到chassis_expt_state上
 * @param   expt_vy:
 * 期望控制的y方向的变换量,方向左正右负,将累加到chassis_expt_state上
 * @param   expt_vz:
 * 期望控制的y方向的变换量,方向逆时针正,将累加到chassis_expt_state上
 */
float expt_gimbal_yaw_scl, real_gimbal_yaw_scl;

float omega_yaw_test = 10;
float A_yaw_test = 10;
float min_yaw_test = -3;

float ratio_x = 6; // 2.2->50W //3.3->100W		-->1.1->0W
float ratio_y = 6;
float ratio_w = 6;

float chassis_expt_vx, chassis_expt_vy, chassis_expt_wz, chassis_expt_x,
    chassis_expt_y;

float ratio_vx1 = 0.1;
float ratio_vy1 = 0.1;
float chassis_expt_wz_thershold = 0.1;
float deg111 = 0.706206799;

float power_limit_test = 40;            // 测试上限功率
float expt_cap_energy_ratio_test = 0.8; // 测试期望电容组期望容值比
float expt_referee_power_buffer_test = 50; // 测试期望裁判系统缓冲能量值

void chassis_ctrl(float expt_delta_yaw, float expt_vx, float expt_vy,
                  float expt_wz) { // 调试部分
  expt_gimbal_yaw_scl = rad2scl(chassis_expt_state.yaw);
  real_gimbal_yaw_scl = rad2scl(chassis_real_state.yaw);

#if 1 // 接了裁判系统

  if (robot.robot_flag.chassis_super_cap_enable_flag == 1) {
    expt_cap_energy_ratio_test = 0.2;
  } else {
    expt_cap_energy_ratio_test = 0.8;
  }
  chassis_power_lim = (chassis_power_lim_t){
      .power_limit =
          /*power_limit_test,*/ game_robot_status.chassis_power_limit,

      //.referee_power_buffer = power_heat_data.chassis_power_buffer,
      .referee_power_buffer = chassis_power_lim.referee_power_buffer,
      .expt_cap_energy_ratio = expt_cap_energy_ratio_test,
      .expt_referee_power_buffer = expt_referee_power_buffer_test,
  };
  if (chassis_power_lim.power_limit == 0) {
    chassis_power_lim.power_limit = 50;
  }
  // update_soft_chassis_energy_buffer(&chassis_power_lim);
#else
  extern float power_limit_test;
  cap_data_t *cap_data = get_cap_data();
  update_soft_chassis_energy_buffer(&chassis_power_lim, power_limit_test,
                                    cap_data);
#endif
  // chassis_power_distributor(&chassis_power_lim);
  //  底盘控制
// #define theta (chassis_real_state.yaw)
#define theta (chassis_real_state.motor_yaw)
  extern chassis_power_lim_t chassis_power_lim;

  chassis_expt_vx = ratio_x * (expt_vx * cos(theta) - expt_vy * sin(theta));
  chassis_expt_vy = ratio_y * (expt_vy * cos(theta) + expt_vx * sin(theta));

  // yaw计算
  chassis_expt_state.yaw = chassis_real_state.motor_yaw + expt_delta_yaw;

  chassis_expt_state.yaw = range_map(chassis_expt_state.yaw, -PI, PI);

  chassis_expt_state.yaw = range_map(chassis_expt_state.yaw, -PI, PI);
  yaw_controller(&chassis_expt_state, &chassis_real_state);

  chassis_expt_state.vx = chassis_expt_vx;
  chassis_expt_state.vy = chassis_expt_vy;

  // extern float ratio_vx, ratio_vy;
  // ratio_vx = 1, ratio_vy = 1;

  if (robot.chassis_mode == chassis_follow_mode ||
      robot.chassis_mode == chassis_spin_mode) {

    chassis_expt_wz = ratio_w * expt_wz;
  } else if (robot.chassis_mode == chassis_tank_mode ||
             robot.chassis_mode == chassis_lob_mode) {
    chassis_expt_wz = ratio_w * expt_wz;
  }
  chassis_expt_state.wz = chassis_expt_wz;
  // if (robot.chassis_mode == chassis_tank_mode ||
  //     robot.chassis_mode == chassis_follow_mode) {
  //   // 四轮模式/底盘回正时,底盘功率优先分配给拿来回正
  //   if (fabs(chassis_expt_wz) > chassis_expt_wz_thershold) {
  //     if (fabs(chassis_real_state.vx) > fabs(chassis_expt_state.vx) &&
  //         sign(chassis_real_state.vx) == sign(chassis_expt_state.vx) &&
  //         chassis_expt_state.vx != 0) {
  //       ratio_vx = ratio_vx1;
  //     }
  //     if (fabs(chassis_real_state.vy) > fabs(chassis_expt_state.vy) &&
  //         sign(chassis_real_state.vy) == sign(chassis_expt_state.vy) &&
  //         chassis_expt_state.vy != 0) {
  //       ratio_vy = ratio_vy1;
  //     }
  //   }
  // }

  wheel_controller(&chassis_expt_state, &chassis_real_state, &chassis_power_lim,
                   (game_robot_status.power_management_chassis_output == 1));
}

///< other

// bool is_chassis_real_stop() {
//   return (fabs(chassis_real_state.vx) < error_vx_thershold &&
//           fabs(chassis_real_state.vy) < error_vy_thershold &&
//           fabs(chassis_real_state.wz) < error_wz_thershold);
// }