#include "./algorithm/util.h"
#include "./base_drv/drv_can.h"
#include "./base_drv/drv_conf.h"
#include "./base_drv/drv_motor/motor.h"
#include "math.h"
#include "tim.h"

#include gimbal_module_core_h
#include gimbal_module_motor_ctrl_h
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/tool/utils/trice.h"
#include "./robot_core/weapon/vision.h"

#define TRICE_FILE Id(47478)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 控制参数
gimbal_state_t gimbal_real_state = {
    0}; // 云台相对于底盘初始化状态的真实数据(imu)
        // yaw:     左->0+,右->0-
        // pitch:   上->0-,下->0+
gimbal_state_t gimbal_expt_state = {
    0}; // 云台相对于底盘初始化状态期望数据(初始imu+sigma遥控)
        // yaw:     左->0+,右->0-
        // pitch:   上->0-,下->0+

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief	其他函数
 */
gimbal_state_t *get_p_gimbal_real_state(void) { return &gimbal_real_state; }
gimbal_state_t *get_p_gimbal_expt_state(void) { return &gimbal_expt_state; }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 	云台运动控制初始化
 * 			内部涉及到各电机的初始化、遥控初始化、板间通讯初始化
 * */

void gimbal_init(void) {
  extern gimbal_motors_t gimbal_motors;
  // 电机函数初始化
  pitch_motor_init();
  shooter_motor_init();
  firectrl_motor_init();
  //  can配置
  uint32_t Fliter_List[4] = {

      rx_stdid_of(left_friction_motor),
      rx_stdid_of(right_friction_motor),
    

  };
  can_user_init(&GIMBAL_MOTOR_CAN, Fliter_List, MOTOR_FIFO, MOTOR_FLTRNUM);
  HAL_Delay(100);

  uint32_t Fliter_List1[4] = {rx_stdid_of(firectrl_motor),
                              rx_stdid_of(pitch_motor)};
  can_user_init(&GIMBAL_MOTOR_CAN, Fliter_List1, MOTOR_FIFO,
                FIRL_MOTOR_FLTRNUM);
  HAL_Delay(100);
  // yaw、pitch角度初始化
  gimbal_expt_state.yaw = gimbal_real_state.yaw;

  set_motor_mid_position(&pitch_motor, pitch_raw_scale_mid);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 	更新gimbal_real_state
 * @note	此部分在imu循环中自动刷新
 */
void update_gimbal_real_state(gimbal_state_t *sensor_state) {
  if (sensor_state != NULL)
    clone_datfptr(gimbal_real_state, sensor_state);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 	缓启动
 * @return	true:缓启动成功
 * @return 	false:缓启动尚未结束
 */
float PITCH_SOFT_START_MAX_TIME = 1;
float ERROR_PITCH_SOFT_START = 10;
#define PITCH_SOFT_START_ANGLE (PI / PITCH_SOFT_START_MAX_TIME / fs_tim_freq)
float delta_yaw_anga, delta_pitch_anga;

bool gimbal_smooth_start(void) {
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 设定云台控制输
  delta_yaw_anga = 0;

  if (fabs(gimbal_expt_state.pitch) > 0.01)
    delta_pitch_anga = -sign(gimbal_real_state.pitch) * PITCH_SOFT_START_ANGLE;
  gimbal_ctrl(delta_yaw_anga, delta_pitch_anga);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 判断缓启动是否完成
  if (fabs(get_delta_ang(gimbal_real_state.pitch, 0, 2 * PI)) <
      deg2rad(ERROR_PITCH_SOFT_START)) {
    gimbal_expt_state.yaw = gimbal_real_state.yaw;
    gimbal_expt_state.pitch = 0;
    return true;
  }
  return false;
}

/**
 * @brief 	断控
 */
void gimbal_disable(void) { shutdown_all_gimbal_motor(); }

/**
 * @brief 调试参数
 */
float yaw_deg = 0;
float pitch_deg = 0;
float roll_deg = 0;

float expt_yaw_deg = 0;
float expt_pitch_deg = 0;
float expt_roll_deg = 0;

float pitch_error = 0;
float yaw_error = 0;
float distance_xyz1 = 20;
float pitch_error_l = 0;
float yaw_error_l = 0;
float pitch_aim = 0;
float yaw_aim = 0;

/**
 *	@brief	云台控制
 *	@note	内部集成缓启动功能
 */
float pitch_real_rel_deg_max = 20.f;
float pitch_real_rel_deg_min = -46.f;
float pitch_real_rel_angle_max = 0.35f;  // 0.30f
float pitch_real_rel_angle_min = -0.60f; // -0.75f
void gimbal_ctrl(float delta_yaw_ang, float delta_pitch_ang) {
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 调试部分
  yaw_deg = rad2scl(gimbal_real_state.yaw);
  pitch_deg = rad2scl(gimbal_real_state.pitch);
  roll_deg = rad2scl(gimbal_real_state.roll);
  expt_yaw_deg = rad2scl(gimbal_expt_state.yaw);
  expt_pitch_deg = rad2scl(gimbal_expt_state.pitch);
  expt_roll_deg = rad2scl(gimbal_expt_state.roll);
  pitch_error = get_delta_ang(expt_pitch_deg, pitch_deg, 8192);
  yaw_error = get_delta_ang(expt_yaw_deg, yaw_deg, 8192);

  yaw_error_l =
      get_delta_ang(gimbal_expt_state.yaw, gimbal_real_state.yaw, 2 * PI) *
      distance_xyz1 * 1000;
  yaw_aim = gimbal_real_state.yaw * distance_xyz1 * 1000;
  pitch_error_l =
      get_delta_ang(gimbal_expt_state.pitch, gimbal_real_state.pitch, 2 * PI) *
      distance_xyz1 * 1000;
  pitch_aim = gimbal_real_state.pitch * distance_xyz1 * 1000;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // yaw计算

  gimbal_expt_state.yaw += delta_yaw_ang;
  gimbal_expt_state.yaw = range_map(gimbal_expt_state.yaw, -PI, PI);
  float _gimbal_expt_state = gimbal_expt_state.yaw;
  _gimbal_expt_state = range_map(_gimbal_expt_state, -PI, PI);
  robot.expt_delta_yaw =
      get_delta_ang(_gimbal_expt_state, gimbal_real_state.yaw, 2 * PI);
  robot.real_yaw = range_map(gimbal_real_state.yaw, -PI, PI);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // pitch计算
  gimbal_expt_state.pitch += delta_pitch_ang;
  gimbal_expt_state.pitch = range_map(gimbal_expt_state.pitch, -PI, PI);
  CLAMP(gimbal_expt_state.pitch, pitch_real_rel_angle_min,
        pitch_real_rel_angle_max);
  //
  pitch_controller(&gimbal_expt_state, &gimbal_real_state);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
