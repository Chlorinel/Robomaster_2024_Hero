#include "./algorithm/filter.h"
#include "./algorithm/pid.h"
#include "./algorithm/pid_ladrc.h"
#include "./algorithm/pid_leso.h"
#include "./base_drv/drv_motor/motor.h"

#include gimbal_module_motor_ctrl_h
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include chassis_module_motor_ctrl_h
gimbal_motors_t gimbal_motors = {0};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @breif   pitch电机初始化参数
 */
// float pitch_kpid_pos[3]={100,0.001,800};
// float pitch_kpid_vel[3]={1500,0.2,150};
// float pitch_kpid_pos[3] = {900, 1, 50};
// float pitch_kpid_vel[3] = {1700, 5, 0};
float pitch_kpid_pos[3] = {700, 2, 40};
float pitch_kpid_vel[3] = {600, 0, 0};
kalman1_state i_pitch_filter;
/**s
 * @brief   pitch电机的初始化函数
 */
void pitch_motor_init(void) {
  // 闭环控制初始化
  pitch_motor.ctrl_mode = dual_pid_leso_ctrl;
  motor_init(&pitch_motor, motor_DJI_GM6020, 6, (void *)pitch_kpid_pos,
             (void *)pitch_kpid_vel);
  pitch_motor.motor_type.reduction_ratio *= 3;
  leso_6020_init(&pitch_motor.pid_leso.leso, 0.2f, 5.0f, 1.7f);
  pitch_motor.pid_leso.leso.af_z1 = 0.05f;
  kalman1_init(&i_pitch_filter, 0, 0.1, 60);
  pitch_motor.lpf_fltr.fc = 2000;
  pitch_motor.pid[POS_LOOP].i_max = 8;
  pitch_motor.pid[POS_LOOP].i_out = 8;
  pitch_motor.pid[RPM_LOOP].i_max = 0;
  pitch_motor.pid[RPM_LOOP].out_max = 500;
  pitch_motor.pid[RPM_LOOP].out_max = 30000;
}
#include "imu.h"
/**
 * @brief   pitch电机的输出控制函数,数据采用imu
 */
// float offset_pitch_current=5000;
// float pitch_offset=0;
float pitch_pos_i_max = 8.f;
float pitch_vel_i_max = 3000.f;
float pitch_pos_o_max = 300.f;
float pitch_input_angle = 0;
void pitch_controller(gimbal_state_t *p_gimbal_expt_state,
                      gimbal_state_t *p_gimbal_real_state) {
  // 调试部分
  pid_update_index(pitch_motor.pid[0], pitch_kpid_pos);
  pid_update_index(pitch_motor.pid[1], pitch_kpid_vel);
  pitch_motor.pid[0].i_max = pitch_pos_i_max;
  pitch_motor.pid[1].i_max = pitch_vel_i_max;
  pitch_motor.pid[0].out_max = pitch_pos_o_max;

  // 采用leso配合pid的算法控制pitch
  float delta_pitch_angle = get_delta_ang(-p_gimbal_expt_state->pitch,
                                          -p_gimbal_real_state->pitch, 2 * PI);

  float pitch_curr_rpm = 0;
  if (robot.IMU_status != offline)
    pitch_curr_rpm = imu_real_info.gx * -20.f;
  else
    pitch_curr_rpm = pitch_motor.real.speed_rpm;

  kalman1_filter(&i_pitch_filter,
                 pitch_motor.real.current +
                     pitch_motor.real.speed_rpm / 13.33f * (30000.0f / 24.0f));
  // pitch_offset=offset_pitch_current*cosf(p_gimbal_real_state->pitch);
  float pid_out;
  // if (robot.is_imu_ctrl_yaw) {
  //   pid_out =
  //       0 + // pitch_offset+
  //       pid_leso_dualloop(pitch_motor.pid_leso.pid,
  //       &pitch_motor.pid_leso.leso,
  //                         delta_pitch_angle, pitch_curr_rpm,
  //                         i_pitch_filter.x);
  // } else {

  //   pid_out = 0 + pid_dual_loop(pitch_motor.pid, pitch_input_angle,
  //                               pitch_motor.real.speed_rpm);
  // }
  pid_out =
      0 + // pitch_offset+
      pid_leso_dualloop(pitch_motor.pid_leso.pid, &pitch_motor.pid_leso.leso,
                        delta_pitch_angle, pitch_curr_rpm, i_pitch_filter.x);

  pitch_motor.output = LPF_update(&pitch_motor.lpf_fltr, pid_out);
}
/**
 * @brief   获取机器人车身坐标系
 */
#include "./robot_core/interface/interface_BTB.h"

gimbal_state_t *fetch_robot_coordinate_system(gimbal_state_t *offset_state) {
  static gimbal_state_t robot_rpy_s;
  float motor_pitch, motor_yaw;

  motor_pitch =
      get_delta_ang(2 * PI, pitch_real_rel_angle, 2 * PI) + offset_state->pitch;
  motor_pitch = range_map(motor_pitch, -PI, PI);
  extern chassis_motors_t chassis_motors;
  motor_yaw =
      get_delta_ang(yaw_real_rel_angle, 2 * PI, 2 * PI) + offset_state->yaw;
  motor_yaw = range_map(motor_yaw, -PI, PI);

  gimbal_state_t robot_rpy = {
      .roll = 0, .pitch = motor_pitch, .yaw = motor_yaw};
  clone(robot_rpy_s, robot_rpy);
  return &robot_rpy_s;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float shooter_kpid_vel_16[3] = {100, 0, 0};
float shooter_kpid_cur_16[3] = {1.2, 0, 0};

/**
 * @brief   摩擦轮电机的初始化函数
 */
void shooter_motor_init(void) {
  left_friction_motor.ctrl_mode = dual_pid_ctrl;
  motor_init(&left_friction_motor, motor_DJI_M3508, 3,
             (void *)shooter_kpid_vel_16, (void *)shooter_kpid_cur_16);
  // 1		0x201		0x200
  left_friction_motor.motor_type.reduction_ratio = 1;
  left_friction_motor.lpf_fltr.fc = 1000;

  right_friction_motor.ctrl_mode = dual_pid_ctrl;
  motor_init(&right_friction_motor, motor_DJI_M3508, 1,
             (void *)shooter_kpid_vel_16, (void *)shooter_kpid_cur_16);
  // 2		0x202		0x200
  right_friction_motor.motor_type.reduction_ratio = 1;
  right_friction_motor.lpf_fltr.fc = 1000;
}
/**
 * @brief   shooter电机的输出控制函数
 * @param   expt_speed:期望旋转rpm
 */
void shooter_controller(void) {
  //    lfm_expt_speed = expt_rpm;
  //    rfm_expt_speed = -expt_rpm;
  // 进行闭环控制
  float pid_out;
  pid_out =
      pid_dual_loop(left_friction_motor.pid, lfm_expt_speed - lfm_real_speed,
                    left_friction_motor.real.current);
  left_friction_motor.output =
      LPF_update(&left_friction_motor.lpf_fltr, pid_out);

  pid_out =
      pid_dual_loop(right_friction_motor.pid, rfm_expt_speed - rfm_real_speed,
                    right_friction_motor.real.current);
  right_friction_motor.output =
      LPF_update(&right_friction_motor.lpf_fltr, pid_out);
}

/**
 * @brief   判断摩擦轮是否已经开启
 *					通过检测左右摩擦轮实际转速与期望转速,检测发送是否成功
 */
#include "stdlib.h"
#define p_ifwo 0.3f
bool f_is_friction_wheel_on(void) {
  uint32_t delta_lfm_speed = abs(lfm_expt_speed - lfm_real_speed);
  uint32_t delta_rfm_speed = abs(rfm_expt_speed - rfm_real_speed);
  uint32_t MAX_delta_lfm_speed = abs((int)(lfm_expt_speed * p_ifwo));
  uint32_t MAX_delta_rfm_speed = abs((int)(rfm_expt_speed * p_ifwo));

  if (delta_lfm_speed < MAX_delta_lfm_speed // 再次确定有无开启摩擦轮
      && delta_rfm_speed < MAX_delta_rfm_speed) {
    return true;
  }
  return false;
}

#include "./robot_core/weapon/vision.h"
void update_shooter_motor_vel(float *expt_v_shooter) {
  if (robot.weapon.real_v_shooter > 16) {
    *expt_v_shooter -= 0.15;
  }
  extern float real_vel_16;
  update_cur_v0_filter();
  float cur_v0 = get_cur_v0();

  // 发射机构转速限幅
  if (*expt_v_shooter > 21)
    *expt_v_shooter = 21;
  if (*expt_v_shooter < 17)
    *expt_v_shooter = 17;
}

/**
 *  @brief	检查一秒内摩擦轮的误差
 *  @return true:摩擦轮掉速严重
 *  @return false:摩擦轮转速并无大碍
 *
 */
#define sample_time 100
float fwSpeedError[2] = {0}; // 我的速度误差(
float motorTemp[2] = {0};
float fw_speed_error_max = 0.2;
bool get_fw_speed_error(void) {
  static int min_speed[2] = {0};
  static int max_speed[2] = {0};

  motorTemp[0] = left_friction_motor.real.tempture;
  motorTemp[1] = right_friction_motor.real.tempture;

  static uint32_t tickstart = 0;
  if (HAL_GetTick() - tickstart < sample_time) {
    if (lfm_real_speed < min_speed[0])
      min_speed[0] = lfm_expt_speed;
    else if (lfm_real_speed > max_speed[0])
      max_speed[0] = lfm_real_speed;

    if (rfm_real_speed < min_speed[1])
      min_speed[1] = rfm_real_speed;
    else if (rfm_real_speed > max_speed[1])
      max_speed[1] = rfm_real_speed;
  } else {
    if (lfm_expt_speed == 0)
      fwSpeedError[0] = -1;
    else
      fwSpeedError[0] = (float)(max_speed[0] - min_speed[0]) / lfm_expt_speed;
    if (rfm_expt_speed == 0)
      fwSpeedError[1] = -1;
    else
      fwSpeedError[1] = -(float)(max_speed[1] - min_speed[1]) / rfm_expt_speed;

    max_speed[0] = lfm_real_speed;
    max_speed[1] = rfm_real_speed;
    min_speed[0] = lfm_real_speed;
    min_speed[1] = rfm_real_speed;
    tickstart = HAL_GetTick();

    if (fabs(fwSpeedError[0]) > fw_speed_error_max &&
        fabs(fwSpeedError[1]) > fw_speed_error_max)
      return true;
  }
  return false;
}
/**
 * @brief   返回摩擦轮电机指针,便于直接检测变量
 */
motor_t *p_lfwM(void) { return &left_friction_motor; }
motor_t *p_rfwM(void) { return &right_friction_motor; }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @breif   firectrl电机初始化参数
 */

float firectrl_kpid_pos[3] = {8000, 7, 0};
float firectrl_kpid_vel[3] = {8, 0, 0};

/**
 * @brief   firectrl电机的初始化函数
 */
void firectrl_motor_init(void) {
  firectrl_motor.ctrl_mode = dual_pid_ctrl;
  motor_init(&firectrl_motor, motor_DJI_M2006, 5, (void *)firectrl_kpid_pos,
             (void *)firectrl_kpid_vel);
  // 5		0x205		0x1ff
}
/**
 * @brief   firectrl电机的输出控制函数
 */
void firectrl_controller(void) {
  // 调试部分
  // pid_update_index(firectrl_motor.pid[0], firectrl_kpid_pos);
  // pid_update_index(firectrl_motor.pid[1], firectrl_kpid_vel);
  // fc_expt_abs_angle += delta_angle;

  // 进行闭环控制
  float pid_out =
      pid_dual_loop(firectrl_motor.pid, fc_expt_abs_angle - fc_real_abs_angle,
                    firectrl_motor.real.speed_rpm);
  firectrl_motor.output = LPF_update(&firectrl_motor.lpf_fltr, pid_out);
}

/**
 * @brief   返回预置轮电机指针,便于直接检测变量
 */
motor_t *p_fcM(void) { return &firectrl_motor; }

/**
 * @brief   关断与发射相关的所有电机
 */
void shutdown_shooter_motor(void) {
  left_friction_motor.output = 0;
  right_friction_motor.output = 0;
  firectrl_motor.output = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void shutdown_all_gimbal_motor(void) {
  for (uint8_t i = 0; i < num_of_all_gimbal_motors; i++) {
    all_gimbal_motors[i].output = 0; // 防止各电机傻转
  }
}