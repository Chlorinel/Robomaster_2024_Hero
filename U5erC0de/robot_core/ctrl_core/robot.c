#include "./algorithm/ballistic.h"
#include "./algorithm/filter.h"
#include "./algorithm/pid.h"
#include "./algorithm/util.h"
#include "imu.h"
#include "math.h"
#include "string.h"
#include "tim.h"

#include "./base_drv/drv_buzzer/buzzer.h"
#include "./base_drv/drv_can.h"
#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/drv_conf.h"
#include "./base_drv/drv_imu/imu.h"
#include "./base_drv/drv_motor/motor.h"
#include "./base_drv/drv_rc/rc.h"
#include "./base_drv/drv_referee/referee.h"
#include "./graphic/graphic_def.h"

// #include "./robot_core/interface/interface_BTB.h"

#include "./robot_core/ctrl_core/chassis_motor_ctrl.h"
#include "./robot_core/ctrl_core/gimbal_motor_ctrl.h"
#include "./robot_core/ctrl_core/move_ctrl.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/tool/bus_detect.h"
#include "./robot_core/tool/utils/trice.h"
#include "./robot_core/weapon/shooter.h"
#include "./robot_core/weapon/vision.h"

#include gimbal_module_core_h
#include chassis_module_core_h
#define TRICE_FILE Id(0)
/**
 * @brief   整车参数
 */
uint8_t contral_conut = 0;

robot_ctrl_t robot = {
    .ctrl_mode = 0,
    .weapon.expt_front_v_shooter = 19,
    .weapon.expt_back_v_shooter = 18.6, // gen2:18.6 //白：17.43
    .weapon.real_v_shooter = INIT_SHOOT_SPEED,
    .weapon.last_real_v_shooter = INIT_SHOOT_SPEED,
    .weapon.ctrl_mode = _manual_ctrl,
    .weapon.f_is_fire = is_fire_rc,
    .weapon.f_is_ready_to_fire = is_ready_to_fire_rc,
    .weapon._is_vision_ok = VISION_NOTARGET,

    .chassis_mode = chassis_disable,
};
robot_ctrl_t *get_p_robot(void) { return &robot; }

referee_info_t referee_info;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float fs_tim_freq = 0;  // 云台主定时器,挂在tim6上,1khz
float hs_tim_freq = 0;  // 高速定时器,挂在tim7上,2.5khz
float chs_tim_freq = 0; // 底盘主定时器,挂在tim5上,1khz
float ls_tim_freq = 0;  // 低速定时器,挂在tim14上,10hz

void robot_init() {
  TRICE(Id(54911), "MSG: Start robot init process\n");

  htim6.Instance->ARR = 1001 - 1;
  htim6.Instance->PSC = APB1_freq / 1000 / 1000 - 1;
  fs_tim_freq =
      APB1_freq / ((htim6.Instance->ARR + 1) * (htim6.Instance->PSC + 1));

  htim5.Instance->ARR = 1001 - 1;
  htim5.Instance->PSC = APB1_freq / 1000 / 1000 - 1;
  chs_tim_freq =
      APB1_freq / ((htim5.Instance->ARR + 1) * (htim5.Instance->PSC + 1));
  // 开启100ms中断,以实时检测接收端是否离线
  htim14.Instance->ARR = 1001 - 1;
  htim14.Instance->PSC = APB1_freq / 1000 / 1000 * 100 - 1;
  ls_tim_freq =
      APB1_freq / ((htim14.Instance->ARR + 1) * (htim14.Instance->PSC + 1));

  // 云台初始化
  gimbal_init();
  // 底盘电机初始化
  chassis_init();
  robot_gimbal_power_off;
  robot_chassis_power_off;
  force_resfresh_cur_v0(INIT_SHOOT_SPEED);
  // BTB_init();
  HAL_Delay(10);

  HAL_Delay(200);
  // 遥控器配置
  rc_recv_dma_init();
  // 图传链路
  vt_rc_recv_dma_init();
  // 陀螺仪初始化(含tim7中断开启)
  imu_init();

  est_position_filter_init();
  // init_UI();
  // 开启循环
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim5);
  robot.timestep = 0;
  robot.is_imu_ctrl_yaw = 1;
  robot.is_bule_or_red = 2;
  robot.base_speed = 1;
  robot.tank_speed = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HAL_StatusTypeDef BTBTXSTATE = HAL_ERROR;
#define Gimbal_yaw_level0 0.4 // yaw旋转速度
#define Gimbal_yaw_level1 0.1
#define Gimbal_yaw_level2 0.6
float Gimbal_lob_yaw_level0 = 2.5;   // in mm
float Gimbal_lob_yaw_level1 = 10;    // in mm
float Gimbal_lob_yaw_level2 = 47.75; // in mm

#define Gimbal_pitch_level0 0.2 // pitch控制速度
#define Gimbal_pitch_level1 0.08
#define Gimbal_pitch_level2 0.4
float Gimbal_lob_pitch_level0 = 5;    // in mm
float Gimbal_lob_pitch_level1 = 20;   // in mm
float Gimbal_lob_pitch_level2 = 95.5; // in mm
//		yaw->0.05
//		pitch->0.1

#include "./robot_core/weapon/vision.h"

float real_vel_16 = INIT_SHOOT_SPEED; // 15.0409298f;
                                      // 新弹14.8
                                      // 旧弹14.9
#define acc_time 0.1f // 键盘控制移动加速到最高速所用的时间
#define k_cnt 1.f / (acc_time * fs_tim_freq)

float k_pitch = Gimbal_pitch_level1;
float k_yaw = Gimbal_yaw_level1;
float k_pitch_rc = Gimbal_pitch_level0;
float k_yaw_rc = Gimbal_yaw_level0;
float k_lob_pitch = 0.002;
float k_lob_yaw = 0.002;
float k_friction = 0.1; // 摩擦轮调节速率
float sigma_z = 0;
float bot, top;

extern target_spec_t target;

// 当遥控器接到电脑后,把左拨杆拨到顶、右拨杆拨到底,即可切换到键鼠控制
#define switch_to_mouse_ctrl                                                   \
  (rc_left_switch == rc_sw_top && rc_right_switch == rc_sw_bottom)
// 当遥控器接到电脑后,把左拨杆拨到顶、右拨杆拨到中,即可切换到配置模式
#define switch_to_config_mode                                                  \
  (rc_left_switch == rc_sw_top && rc_right_switch == rc_sw_middle)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t freq_of_vision = 0;
/**
 * @brief 	云台运动控制主循环
 * 				内部主要控制pitch与底盘运动
 *	@note 	此函数于tim4的1khz中断中运行
 *	@note	控制模式说明:带开机缓启动,缓启动时先自动回正
 **/
float a234 = 0;
gimbal_state_t vision_ctrl_state = {0};
float delta_yaw_ang, delta_pitch_ang;
float mid_pitch_test = -7;
float A_pitch_test = 5;
float omega_pitch_test = 6.28;
extern gimbal_state_t gimbal_real_state;
extern gimbal_state_t gimbal_expt_state;
bool temp_flag = 0;
void gimbal_get_ctrl_way(void) {
  temp_flag = is_vt_rc_offline();
  if (robot.ctrl_mode == 0) {
    // 将图传相关标志位清空

    robot.robot_flag.vt_config_flag = 0;
    // 断控

    if (((rc_left_switch == 1) && (rc_right_switch == 1)) ||
        (robot.robot_flag.referee_kill_flag == true)) {
      robot.robot_flag.vt_ctrl_flag = 0;
      robot_gimbal_power_off;
      robot_chassis_power_off;
    }
    // 异常情况
    if ((rc_left_switch == 0 || rc_right_switch == 0) ||

        is_rc_offline() == true) {
      robot.ctrl_mode = 1;

      robot_gimbal_power_off;
      robot_chassis_power_off;
    }
  } else {

    if ((IS_KEY_PRESS(KEY_Q) == true && IS_KEY_LAST_PRESS(KEY_Q) != true) &&
        (IS_KEY_PRESS(KEY_E) == true && IS_KEY_LAST_PRESS(KEY_E) != true)) {
      robot.robot_flag.vt_ctrl_flag = !robot.robot_flag.vt_ctrl_flag;
    }
    if ((IS_KEY_PRESS(KEY_Q) == true && IS_KEY_LAST_PRESS(KEY_Q) != true) &&
        (IS_KEY_PRESS(KEY_R) == true && IS_KEY_LAST_PRESS(KEY_R) != true)) {
      robot.robot_flag.vt_config_flag = !robot.robot_flag.vt_config_flag;
    }
    // 断控
    if (robot.robot_flag.vt_ctrl_flag == 0 ||
        (robot.robot_flag.referee_kill_flag == true)) {

      robot_gimbal_power_off;
      robot_chassis_power_off;
    }
    // 异常情况
    if (!(rc_left_switch == 0 || rc_right_switch == 0)) {
      robot.ctrl_mode = 0;
    }
  }

  if (switch_to_mouse_ctrl == true ||
      (robot.ctrl_mode == 1 &&
       robot.robot_flag.vt_config_flag == 0)) { // 键鼠控制
    robot.tank_speed += Mouse_z / (fs_tim_freq * 3);
    if (robot.tank_speed <= 0.1) {
      robot.tank_speed = 0.1;
    }
    if (robot.tank_speed >= 4) {
      robot.tank_speed = 4;
    }
    if (IS_KEY_PRESS(KEY_G) == true && IS_KEY_LAST_PRESS(KEY_G) != true)
      robot.is_center_fire = !robot.is_center_fire;
    // 模式选择
    if (IS_KEY_PRESS(KEY_Q) == true && IS_KEY_LAST_PRESS(KEY_Q) != true)
      robot.move_mode = _tank_mode;
    else if (IS_KEY_PRESS(KEY_E) == true && IS_KEY_LAST_PRESS(KEY_E) != true)
      robot.move_mode = _follow_mode;
    else if (IS_KEY_PRESS(KEY_Z) == true && IS_KEY_LAST_PRESS(KEY_Z) != true)
      robot.move_mode = _spin_mode;
    else if (IS_KEY_PRESS(KEY_X) == true && IS_KEY_LAST_PRESS(KEY_X) != true) {
      robot.move_mode = _lob_mode;
    }
    robot.last_move_mode = robot.move_mode;

    // 运动控制
    if (robot.ctrl_mode == 1) {
      if (robot.robot_flag.vt_ctrl_flag == 1) {
        robot_gimbal_power_on;
      }
    } else {
      robot.robot_flag.vt_ctrl_flag = 1;
      robot_gimbal_power_on;
    }

    static float vcnt[4] = {0};
    if (IS_KEY_PRESS(KEY_W) == true)
      vcnt[0] += k_cnt;
    else
      vcnt[0] -= k_cnt;
    if (IS_KEY_PRESS(KEY_S) == true)
      vcnt[1] += k_cnt;
    else
      vcnt[1] -= k_cnt;
    if (IS_KEY_PRESS(KEY_D) == true)
      vcnt[2] += k_cnt;
    else
      vcnt[2] -= k_cnt;
    if (IS_KEY_PRESS(KEY_A) == true)
      vcnt[3] += k_cnt;
    else
      vcnt[3] -= k_cnt;
    CLAMP(vcnt[0], 0, 1);
    CLAMP(vcnt[1], 0, 1);
    CLAMP(vcnt[2], 0, 1);
    CLAMP(vcnt[3], 0, 1);
    // 发弹控制
    if (rc_ctrl_data.mouse.press_right == true) {
      robot.weapon.ctrl_mode = _vision_ctrl;
    } else {
      clear_ammo_num();
      robot.weapon.ctrl_mode = _manual_ctrl;
    }
    // 移动控制
    if (robot.move_mode != _lob_mode) { // 非吊射
      robot.vx = (vcnt[0] - vcnt[1]) * robot.base_speed;
      robot.vy = (vcnt[3] - vcnt[2]) * robot.base_speed;
      robot.v_yaw = -2 * PI * k_yaw * Mouse_x;
      robot.v_pitch = -2 * PI * k_pitch * Mouse_y;
    } else if (robot.move_mode == _lob_mode &&
               robot.weapon.ctrl_mode == _vision_ctrl &&
               robot.weapon._is_vision_ok == VISION_OK) { // 视觉辅助吊射模式
      robot.vy = 0;
      robot.vx = 0;
      float distance_xyz = 1;

      extern attack_target_type_t attack_target_type;

      switch (attack_target_type) {
      case outpost_spin_armor:
        robot.weapon.shooter_offset.pitch =
            &robot.weapon.outpost_spin_armor_offset.pitch;
        robot.weapon.shooter_offset.yaw =
            &robot.weapon.outpost_spin_armor_offset.yaw;
        k_lob_yaw = Gimbal_lob_yaw_level2;
        k_lob_pitch = Gimbal_lob_pitch_level2;
        break;
      case outpost_top_armor:
        robot.weapon.shooter_offset.pitch =
            &robot.weapon.outpost_top_armor_offset.pitch;
        robot.weapon.shooter_offset.yaw =
            &robot.weapon.outpost_top_armor_offset.yaw;
        k_lob_yaw = Gimbal_lob_yaw_level2;
        k_lob_pitch = Gimbal_lob_pitch_level2;
        break;
      case base_mid_armor:
        robot.weapon.shooter_offset.pitch =
            &robot.weapon.base_mid_armor_offset.pitch;
        robot.weapon.shooter_offset.yaw =
            &robot.weapon.base_mid_armor_offset.yaw;
        k_lob_yaw = Gimbal_lob_yaw_level2;
        k_lob_pitch = Gimbal_lob_pitch_level2;
        break;
      default:
        robot.weapon.shooter_offset.pitch = &robot.weapon.other_offset.pitch;
        robot.weapon.shooter_offset.yaw = &robot.weapon.other_offset.yaw;
        k_lob_yaw = Gimbal_lob_yaw_level2;
        k_lob_pitch = Gimbal_lob_pitch_level2;
        break;
      }
      extern vision_ctrl_t vision_ctrl_data;
      float xc2 = vision_ctrl_data.x * vision_ctrl_data.x;
      float yc2 = vision_ctrl_data.y * vision_ctrl_data.y;
      float zc2 = vision_ctrl_data.z * vision_ctrl_data.z;
      distance_xyz = sqrt(xc2 + yc2 + zc2);
      if (distance_xyz <= 0.5)
        distance_xyz = 0.5;
      *robot.weapon.shooter_offset.pitch -= k_lob_pitch / 1000.f /
                                            distance_xyz * (vcnt[0] - vcnt[1]) /
                                            fs_tim_freq;
      *robot.weapon.shooter_offset.yaw -=
          k_lob_yaw / 1000.f / distance_xyz * (vcnt[2] - vcnt[3]) / fs_tim_freq;

      CLAMP_MAX(*robot.weapon.shooter_offset.pitch, deg2rad(15));
      CLAMP_MAX(*robot.weapon.shooter_offset.yaw, deg2rad(15));
    } else { // 手瞄吊射模式
      robot.vy = 0;
      robot.vx = 0;
      k_lob_yaw = Gimbal_lob_yaw_level1;
      k_lob_pitch = Gimbal_lob_pitch_level1;

      float v_pitch = k_lob_pitch / 1000.f / 6 * (vcnt[0] - vcnt[1]);
      float v_yaw = k_lob_yaw / 1000.f / 6 * (vcnt[2] - vcnt[3]);
      a234 = v_pitch;
      robot.v_pitch = -2 * PI * (v_pitch);
      robot.v_yaw = -2 * PI * (v_yaw);
    }
    extern bool base_mode_enable;
    extern bool had_find_base;
    static bool base_mode_enable_flag = false;
    if (IS_KEY_PRESS(KEY_B) == true && IS_KEY_LAST_PRESS(KEY_B) == false) {
      base_mode_enable_flag = !base_mode_enable_flag;
    }
    if (base_mode_enable_flag == true) {
      base_mode_enable = true;
    } else {
      base_mode_enable = false;
      had_find_base = false;
    }
    // 接入视觉控制自动开火
    robot.weapon.f_is_ready_to_fire = is_ready_to_fire_km;
    if (IS_KEY_PRESS(KEY_CTRL) == true)
      robot.weapon.f_is_fire = is_fire_km_vision;
    else
      robot.weapon.f_is_fire = is_fire_km;
    // 发弹是否取消热量限制
    if (IS_KEY_PRESS(KEY_F) == true && rc_ctrl_data.mouse.press_left == 1) {

      robot.robot_flag.gimbal_fuck_mode_flag = 1;
    } else {
      robot.robot_flag.gimbal_fuck_mode_flag = 0;
    }
    // 超级电容
    if (IS_KEY_PRESS(KEY_SHIFT) == true)
      robot.robot_flag.chassis_super_cap_enable_flag = 1;
    else
      robot.robot_flag.chassis_super_cap_enable_flag = 0;
  } else if (switch_to_config_mode ||
             (robot.ctrl_mode == 1 &&
              robot.robot_flag.vt_config_flag == 1)) { // 配置模式
    robot_gimbal_power_on;
    robot.vy = 0;
    robot.vx = 0;
    robot.v_yaw = 0;
    robot.v_pitch = 0;
    robot.move_mode = _tank_mode;
    robot.weapon.ctrl_mode = _vision_ctrl;

    // 手动reset
    void System_Reset(void);
    extern uint8_t rst_cnt;
    if (IS_KEY_PRESS(KEY_R) == true && IS_KEY_PRESS(KEY_G) == true) {
      rst_cnt++;
      if (rst_cnt > 100) {
        rst_cnt = 0;
        System_Reset();
      }
    } else {
      rst_cnt = 0;
    }

    // 发弹控制
    if (rc_ctrl_data.mouse.press_right == true) {
      robot.weapon.ctrl_mode = _vision_ctrl;
    } else {
      clear_ammo_num();
      robot.weapon.ctrl_mode = _manual_ctrl;
    }
    if (IS_KEY_PRESS(KEY_A) == true && IS_KEY_LAST_PRESS(KEY_A) == false) {
      robot.is_bule_or_red += 1;
    }
    if (robot.is_bule_or_red == 3) {
      robot.is_bule_or_red = 0;
    }

  } else if (rc_left_switch != rc_sw_top &&
             robot.ctrl_mode == 0) { // 遥控器控制
    // 模式选择

    if (rc_left_switch == rc_sw_middle) {
      // 混用打蛋时左边拨到中间一次就重置氮素滤波
      if (rc_left_last_switch != rc_sw_middle) {
        // force_resfresh_cur_v0(INIT_SHOOT_SPEED);
      }
      if (rc_right_switch == rc_sw_top) {
        robot.move_mode = _tank_mode;
        robot.is_imu_ctrl_yaw = 1;
        robot.weapon.ctrl_mode = _vision_ctrl;
        robot.robot_flag.gimbal_fuck_mode_flag = 0;
        robot.robot_flag.chassis_super_cap_enable_flag = 0;

      } else if (rc_right_switch == rc_sw_middle) {
        robot.move_mode = _follow_mode;
        robot.is_imu_ctrl_yaw = 1;
        robot.weapon.ctrl_mode = _manual_ctrl;
        robot.robot_flag.gimbal_fuck_mode_flag = 1;
        robot.robot_flag.chassis_super_cap_enable_flag = 1;

      } else if (rc_right_switch == rc_sw_bottom) {

        robot.move_mode = _spin_mode;
        robot.weapon.ctrl_mode = _vision_ctrl;
      }

      if (robot.weapon.ctrl_mode == _vision_ctrl)
        robot.weapon.f_is_fire = is_fire_rc_vision;
      else if (robot.weapon.ctrl_mode == _manual_ctrl)
        robot.weapon.f_is_fire = is_fire_rc;
    }
    // 运动控制
    robot_gimbal_power_on;
    robot.vx = s_curve(1, rc_left_x) * robot.base_speed;
    robot.vy = -s_curve(1, rc_left_y) * robot.base_speed;
    robot.v_yaw = -2 * PI * k_yaw_rc * rc_right_x;
    robot.v_pitch = -2 * PI * k_pitch_rc * rc_right_y;

    // 发弹控制
    robot.weapon.f_is_ready_to_fire = is_ready_to_fire_rc;
  } else {
    robot_gimbal_power_off;
    robot_chassis_power_off;
  }

  //  if (referee_info.shooter_id1_42mm_speed_limit >= 16) {
  //    CLAMP(robot.weapon.expt_v_shooter, Gimbal_shooter_level4,
  //          Gimbal_shooter_level5);
  //    bot = (Gimbal_shooter_level4 - Gimbal_shooter_level4) / k_friction;
  //    top = (Gimbal_shooter_level5 - Gimbal_shooter_level4) / k_friction;
  //  } else {
  //    CLAMP(robot.weapon.expt_v_shooter, Gimbal_shooter_level1,
  //          Gimbal_shooter_level3);
  //    bot = (Gimbal_shooter_level1 - Gimbal_shooter_level4) / k_friction;
  //    top = (Gimbal_shooter_level3 - Gimbal_shooter_level4) / k_friction;
  //  }
  // CLAMP(sigma_z, bot, top);
  // chassis_info.shooter_ref_vel = (float)(sigma_z - bot) / (top - bot);
  //  robot.shooter_ref_vel = (float)(sigma_z - bot) / (top - bot);
}

void chassis_get_ctrl_way(void) {

  if (robot.chassis_curr_state == chassis_power_off) {
    robot.chassis_mode = chassis_disable;
  }

  if (robot.chassis_curr_state == chassis_resetting) {
    if (robot.chassis_mode == chassis_follow_mode) {
      robot.chassis_mode = chassis_smooth_start;
    } else {
      robot.chassis_curr_state = chassis_power_on;
    }
  }
}

void robot_gimbal_tim_loop(void) {
  gimbal_state_t gimbal_real_state = *get_p_gimbal_real_state();
  gimbal_state_t gimbal_expt_state = *get_p_gimbal_expt_state();

  // 运动控制部分
  gimbal_get_ctrl_way();
  // 底盘yaw的pid_leso控制
  robot.yaw_imu_gx = imu_real_info.gz;

  // 获取视觉控制
  send_vision_request(gimbal_real_state.roll, gimbal_real_state.pitch,
                      gimbal_real_state.yaw);

  get_vision_aim_mode();
  extern attack_target_type_t attack_target_type;
  if (attack_target_type != base_mid_armor) {
    // 仅当看到过装甲板且找到前哨站
    robot.weapon._is_vision_ok = get_vision_ctrl(
        &vision_ctrl_state.pitch, &vision_ctrl_state.yaw,
        &vision_ctrl_state.shooter_yaw, 1.f / fs_tim_freq, robot.is_center_fire);
  } else {
    refresh_base_mode(false);
    robot.weapon._is_vision_ok = get_vision_ctrl_base_mode(
        &vision_ctrl_state.pitch, &vision_ctrl_state.yaw,&vision_ctrl_state.shooter_yaw, 1.f / fs_tim_freq);
  }
  get_vision_suggest_fire(&gimbal_expt_state, &gimbal_real_state);
  if (robot.weapon._is_vision_ok == VISION_OK)
    robot.robot_flag.vision_data_ready_flag = true;
  else
    robot.robot_flag.vision_data_ready_flag = 0;

  get_data_refresh_freq(vision_ctrl_state.yaw, freq_of_vision);

  // 状态选择
  switch (robot.gimbal_curr_state) {
  case gimbal_power_on: {
    robot_chassis_power_on;
    bool is_switch_to_vision_ctrl = robot.weapon.ctrl_mode == _vision_ctrl;
    bool is_tracker_reset_over = resetTracker(is_switch_to_vision_ctrl);
    if (is_switch_to_vision_ctrl == true) {

      if (is_tracker_reset_over == true &&
          robot.weapon._is_vision_ok == VISION_OK &&
          is_vision_offline() == false) { // reset成功,开始接入视觉数据
        get_p_gimbal_expt_state()->yaw = vision_ctrl_state.yaw;
        get_p_gimbal_expt_state()->pitch = vision_ctrl_state.pitch;
        // get_p_gimbal_expt_state()->pitch=
        // vision_ctrl_state.pitch+deg2rad(mid_pitch_test)+deg2rad(A_pitch_test)*sin(omega_pitch_test*HAL_GetTick()/1000.f);
        delta_yaw_ang = 0;
        delta_pitch_ang = 0;
      } else // if (is_tracker_reset_over == false)
      {      // 未reset成功
             // 或没视觉数据
        delta_yaw_ang = robot.v_yaw / fs_tim_freq;
        delta_pitch_ang = robot.v_pitch / fs_tim_freq;
      }
    } else {
      robot.robot_flag.vision_data_ready_flag = 0;
      delta_yaw_ang = robot.v_yaw / fs_tim_freq;
      delta_pitch_ang = robot.v_pitch / fs_tim_freq;
    }
    gimbal_ctrl(delta_yaw_ang, delta_pitch_ang);

    switch (robot.move_mode) {
    case _tank_mode: {
      robot.chassis_mode = chassis_tank_mode;

      break;
    }
    case _follow_mode: {
      robot.chassis_mode = chassis_follow_mode;

      break;
    }
    case _spin_mode: {
      robot.chassis_mode = chassis_spin_mode;
      break;
    }
    case _lob_mode: {
      robot.chassis_mode = chassis_lob_mode;
    }
    default:
      break;
    }

    // 发弹控制
    shooter_control_loop(robot.weapon.expt_back_v_shooter);
    break;
  }
  case gimbal_resetting: {
    robot.chassis_mode = chassis_follow_mode;
    robot_chassis_power_on;

    // 云台缓启动
    if (gimbal_smooth_start() == true)
      robot.gimbal_curr_state = gimbal_power_on;

    // 底盘速度控制
    robot.vx = 0;
    robot.vy = 0;
    // 发弹控制
    shooter_control_loop(robot.weapon.expt_back_v_shooter);
    break;
  }
  case gimbal_power_off: {
    // 操作云台断控
    float delta_yaw_ang, delta_pitch_ang; // 使得expt跟real
    delta_yaw_ang =
        -get_delta_ang(gimbal_expt_state.yaw, gimbal_real_state.yaw, 2 * PI);
    delta_pitch_ang = -get_delta_ang(gimbal_expt_state.pitch,
                                     gimbal_real_state.pitch, 2 * PI);
    gimbal_ctrl(delta_yaw_ang, delta_pitch_ang);
    shooter_control_loop(robot.weapon.expt_back_v_shooter);
    // 操作底盘停止
    robot.expt_delta_yaw = 0;
    robot.vx = 0;
    robot.vy = 0;
    robot.robot_flag.gimbal_fuck_mode_flag = 0;
    shutdown_all_gimbal_motor();

    // 清除各模式的标识

    robot.move_mode = _tank_mode;
    shooter_debug.is_ready_to_fire = 0;
    robot.robot_flag.gimbal_fuck_mode_flag = 0;
    robot.robot_flag.chassis_super_cap_enable_flag = 0;
    robot_chassis_power_off;
    clear_ammo_num();

    break;
  }
  }

  if (contral_conut % 3 == 2 || contral_conut % 3 == 1) {
    set_all_gimbal_motor_output();
  }

  contral_conut++;
  update_rc_last_key();
  inc_rc_rx_lost();
  // 其他代码
  get_shoot_delay();
  robot.timestep += 1; // 1000hz
}
// aabb

pid_struct_t power_pid = {
    .kp = 4, .ki = 0.5, .kd = 3, .i_max = 0.05, .out_max = 6};
float temp_error;
float k_power = 1.2;
void robot_chassis_tim_loop(void) {
  // 运动控制部分

  // update_chassis_real_state();
  chassis_get_ctrl_way();
  if (robot.robot_flag.chassis_super_cap_enable_flag == 1) {
    robot.base_speed = 4;
  } else {
    robot.base_speed = robot.tank_speed;
  }
  if (robot.chassis_mode(robot.expt_delta_yaw, robot.vx, robot.vy) == true)
    robot.chassis_curr_state = chassis_power_on;
  else
    robot.chassis_curr_state = chassis_power_off;

  dial_control_loop();
  if (contral_conut % 3 == 2 || contral_conut % 3 == 1) {
    set_all_chassis_motor_output();
  } else {
    if (contral_conut >= 240) {
      contral_conut = 0;
    }
  }

#if enable_watch_dog
  HAL_IWDG_Refresh(&hiwdg);
#endif
}
