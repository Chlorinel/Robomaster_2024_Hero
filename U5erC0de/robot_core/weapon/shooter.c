#include "./robot_core/weapon/shooter.h"
#include "./base_drv/drv_motor/motor.h"
#include "./base_drv/drv_rc/rc.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/weapon/vision.h"
#include "stdlib.h"

#include gimbal_module_core_h
#include gimbal_module_motor_ctrl_h
extern gimbal_motors_t gimbal_motors;

struct {
  uint8_t ready_to;
  uint8_t fire_on;
  float front_shotspeed;
  float back_shotspeed;
  uint8_t shoot_num;
  bool fuck_mode;
} testing_flag = {.ready_to = 0,
                  .fire_on = 0,
                  .front_shotspeed = 19.5,
                  .back_shotspeed = 19.4,
                  .shoot_num = 0,
                  .fuck_mode = 0};

shooter_t shooter_debug = {
    ._fire_status = fire_status_idle,
    .t0 = 100,
    .t1 = 100,
};

#define fire_status shooter_debug._fire_status
#define fire_error_status shooter_debug._fire_error_status

/**
 * @brief   上弹成功判断
 *          通过检测控弹电机逆向旋转时的转矩电流来确定是否成功上到弹
 * @note		电机反转,输出电流全为负
 */

int16_t stall_enery_MAX =
    -200; // 电机堵转电流积分的最大值,为此值时就可以判定说上弹成功了
int16_t idle_current = -1500; // 电机空转电流 //简单测测就好了
int16_t stall_vel = 1200;     // 堵转时的电机速度
float stall_enegry = 0;       // 电机堵转电流的积分
float last_stall_energy = 0; // 调试用,记录上一次确认上弹的电流积分值
float stall_current = 0; // 电机堵转电流

float delta_t = 6;
float delta_ang = 1;
bool is_load_over(void) {
  // 控制电机反转
  static uint32_t tickstart = 0;
  if (HAL_GetTick() - tickstart > delta_t) {
    fc_expt_abs_angle -= deg2rad(delta_ang);
    tickstart = HAL_GetTick();
  }

  // 计算是否堵转
  stall_current = fc_real_currs - idle_current;
  stall_enegry += stall_current / fs_tim_freq;
  if (stall_enegry > 0)
    stall_enegry = 0;
  // 判断是否填装成功
  if (stall_enegry < stall_enery_MAX &&
      abs(firectrl_motor.real.speed_rpm) < stall_vel) {
    last_stall_energy = stall_enegry;
    stall_enegry = 0;
    return true;
  } else
    return false;
}

float to_change_kp = 85;
float to_change_ki = 0;
float to_change_kd = 0;
uint8_t front_or_back = 0;

void chang_friction_wheel_on_pid(void) {
  if (front_or_back == 0) {
    left_friction_motor.pid[0].kp = to_change_kp;
    left_friction_motor.pid[0].ki = to_change_ki;
    left_friction_motor.pid[0].kd = to_change_kd;

    right_friction_motor.pid[0].kp = to_change_kp;
    right_friction_motor.pid[0].ki = to_change_ki;
    right_friction_motor.pid[0].kd = to_change_kd;
  } else {
  }
}
/**
 * @brief   判断摩擦轮是否已经开启
 *					通过检测左右摩擦轮实际转速与期望转速,检测发送是否成功
 */
#define p_ifwo 0.05f
bool is_friction_wheel_on(void) {
  uint32_t delta_lfm_speed = abs(lfm_expt_speed - lfm_real_speed);
  uint32_t delta_rfm_speed = abs(rfm_expt_speed - rfm_real_speed);
  uint32_t MAX_delta_lfm_speed = abs((int)(lfm_expt_speed * p_ifwo));
  uint32_t MAX_delta_rfm_speed = abs((int)(rfm_expt_speed * p_ifwo));

  if (delta_lfm_speed < MAX_delta_lfm_speed // 再次确定有无开启摩擦轮
      && delta_rfm_speed < MAX_delta_rfm_speed) {
    //      if (delta_lfm_back_speed < MAX_delta_lfm_back_speed &&
    //          delta_rfm_back_speed < MAX_delta_rfm_back_speed) {

    return true;
  }
  return false;
}

/**
 * @brief   推弹函数
 *					在确认发弹后调用此函数即可把弹丸推到摩擦轮处
 *					内嵌发弹延时控制功能
 * @return	成功推弹完后将返回true,反之为false
 */
enum {
  PUSH_STATE0 = 0,
  PUSH_STATE1 = 1,
  PUSH_STATE2 = 2,
} first_push_state = PUSH_STATE0;
uint32_t push_bullet_tick = 0;
uint32_t shoot_delay1 = 0;
uint32_t shoot_delay2 = 0;
uint32_t const_shoot_delay1 = 300;
uint32_t const_shoot_delay2 = 80;
float add_angle1 = 120;
float add_angle2 = 100;
bool delay_bit[2] = {0};
bool push_bullet(bool enable_const_shoot_delay) {
  if (enable_const_shoot_delay) {
    shoot_delay1 = const_shoot_delay1;
    shoot_delay2 = const_shoot_delay2;
  } else {
    shoot_delay1 = 0;
    shoot_delay2 = 0;
  }

  delay_bit[0] =
      (HAL_GetTick() - push_bullet_tick > shoot_delay1 - shoot_delay2);
  delay_bit[1] = (HAL_GetTick() - push_bullet_tick > shoot_delay1);

  if (first_push_state == PUSH_STATE0) { // 推弹,加第一次大角度
    first_push_state = PUSH_STATE1;
    push_bullet_tick = HAL_GetTick();
    fc_expt_abs_angle += deg2rad(add_angle1 - add_angle2);
  }
  if (HAL_GetTick() - push_bullet_tick > shoot_delay1 - shoot_delay2 &&
      first_push_state == PUSH_STATE1) { // 推弹,加第二次小角度
    first_push_state = PUSH_STATE2;
    fc_expt_abs_angle += deg2rad(add_angle2);
  }
  if (HAL_GetTick() - push_bullet_tick > shoot_delay1 &&
      first_push_state == PUSH_STATE2) { // 延时结束,清除标识符
    first_push_state = PUSH_STATE0;
    return true;
  }

  return false;
}
/**
 *	@brief  遥控器控制的开火准备函数
 * 				设定当左拨杆为下边时
 *					且右拨杆为非顶部时触发true
 **/
bool is_ready_to_fire_rc(void) {
  shooter_debug.is_ready_to_fire =
      (rc_left_switch == rc_sw_bottom && rc_right_switch != rc_sw_top);
  return shooter_debug.is_ready_to_fire;
}
bool is_ready_to_fire_ts(void) {
  shooter_debug.is_ready_to_fire = testing_flag.ready_to;
  return shooter_debug.is_ready_to_fire;
}

/**
 *	@brief  键鼠控制的开火准备函数
 * 				设定按下c时触发true
 *					设定按下v时触发false
 **/
bool _is_key_c_press = false;
bool _is_key_c_last_press = false;
bool _is_key_v_press = false;
bool _is_key_v_last_press = false;
uint8_t fire_mode = 1;

bool is_ready_to_fire_km(void) {
  if (shooter_debug.is_ready_to_fire == false) {
    fire_mode = 1;
    _is_key_c_press = IS_KEY_PRESS(KEY_C);
    _is_key_c_last_press = IS_KEY_LAST_PRESS(KEY_C);

    if ((IS_KEY_PRESS(KEY_C) && !IS_KEY_LAST_PRESS(KEY_C)))
      shooter_debug.is_ready_to_fire = true;
  } else {
    fire_mode = 2;
    _is_key_v_press = IS_KEY_PRESS(KEY_V);
    _is_key_v_last_press = IS_KEY_LAST_PRESS(KEY_V);

    if ((IS_KEY_PRESS(KEY_V) && !IS_KEY_LAST_PRESS(KEY_V)))
      shooter_debug.is_ready_to_fire = false;
  }
  return shooter_debug.is_ready_to_fire;
}
bool ret0 = false, ret1 = false;
/**
 *	@brief  遥控器控制的开火开关函数
 * 				设定当左拨杆为下边时
 *					将右上拨杆从中间切到下边将触发true
 **/
bool is_fire_rc(void) {
  if (rc_left_switch == rc_sw_bottom) {
    // static rc_switch_t last_rc_status=rc_sw_top;
    if (rc_right_last_switch == rc_sw_middle && rc_right_switch == rc_sw_bottom)
      ret0 = true;
    else
      ret0 = false;
  } else {
    ret0 = false;
  }
  if (ret0 == true)
    ret1 = true;
  return ret0;
}

bool is_fire_ts(void) {
  if (testing_flag.fire_on) {
    return true;

  } else {
    return false;
  }
}
uint8_t shoot_cnt = 0; // 允许发弹量
bool is_fire_rc_vision(void) {
  if (is_fire_rc())
    shoot_cnt++;
  if (shoot_cnt && vision_ctrl_data.suggest_fire == true)
    return true;
  else
    return false;
}
/**
 *	@brief  键鼠控制的开火开关函数
 * 				设定当按下左键时触发true
 **/
bool is_fire_km(void) {
  return ((rc_ctrl_data.mouse.press_left == 1) &&
          (rc_ctrl_data.mouse.last_press_left != 1));
}

bool is_fire_km_vision(void) {
  if (is_fire_km())
    shoot_cnt++;
  return (shoot_cnt && vision_ctrl_data.suggest_fire == true);
}
float soft_shooter_heat = 0;
uint8_t last_shoot_cnt = 0;

bool is_shooter_overheat(void) { return 0; }
/**
 * @brief   发射装置使能循环
 * @param 	speed_of_friction_motor:摩擦轮转速
 */
bool is_shooter_cool_down;
uint32_t shoot_delay = 0;
bool test_ignore_heat_lim = true;
void shooter_control_loop(float front_wheel_v) {
  static uint32_t tickstart = 0; // 用于限制发弹频率
  chang_friction_wheel_on_pid();

  if (robot.weapon.real_v_shooter != referee_info.bullet_speed &&
      referee_info.bullet_speed != 0) {
    robot.weapon.last_real_v_shooter = robot.weapon.real_v_shooter;
    robot.weapon.real_v_shooter = referee_info.bullet_speed;
    update_shooter_motor_vel(&robot.weapon.expt_back_v_shooter);
  }

  // update_soft_shooter_heat();

  // 刷新fire_status
  if (fire_status != fire_status_firing && fire_status != fire_status_fired) {
    //    if (testing_flag.ready_to == false) {
    //      fire_status = fire_status_idle;
    //      testing_flag.fire_on = 0;
    //      testing_flag.shoot_num = 0;
    if (robot.weapon.f_is_ready_to_fire() == false)
      fire_status = fire_status_idle;
    else if (shooter_debug.is_load_over == false)
      fire_status = fire_status_prefill;
    else if (shooter_debug.is_load_over == true)
      fire_status = fire_status_waiting;

    //		 else
    //		 {
    //		 	fire_status = fire_status_waiting;
    //		 }
  }

  switch (fire_status) {
  case fire_status_idle:
    lfm_expt_speed = 0;
    rfm_expt_speed = 0;

    fc_expt_abs_angle = fc_real_abs_angle;
    robot.robot_flag.chassis_dial_eater_flag = 0;
    robot.robot_flag.chassis_dial_feeder_flag = 0;
    stall_enegry = 0;
    break;

  case fire_status_prefill:
    lfm_expt_speed = mps2rpm_front(front_wheel_v);
    rfm_expt_speed = -mps2rpm_front(front_wheel_v);

    // 确定是否成功上弹
    shooter_debug.is_load_over = is_load_over();
    if (robot.robot_flag.gimbal_fuck_mode_flag == true)
      shooter_debug.is_load_over = true;

    if (HAL_GetTick() - tickstart > shooter_debug.t0 + shoot_delay) {
      shooter_debug.is_dial_stalling = robot.robot_flag.chassis_dial_stall_flag;
      // 判断是否卡弹
      if (shooter_debug.is_load_over == false) {
        if (shooter_debug.is_dial_stalling == true) {
          robot.robot_flag.chassis_dial_eater_flag = 1;
          robot.robot_flag.chassis_dial_feeder_flag = 0;
        } else if (shooter_debug.is_dial_stalling == false) {
          robot.robot_flag.chassis_dial_feeder_flag = 1;
          robot.robot_flag.chassis_dial_eater_flag = 0;
        }
      } else {
        firectrl_motor.expt.abs_angle =
            firectrl_motor.real.abs_angle; // 防止卡死
        fire_status = fire_status_waiting;
      }
    } else {
      robot.robot_flag.chassis_dial_feeder_flag = 1;
    }
    break;

  case fire_status_waiting: {
    lfm_expt_speed = mps2rpm_front(front_wheel_v);
    rfm_expt_speed = -mps2rpm_front(front_wheel_v);

    if (robot.robot_flag.gimbal_fuck_mode_flag == false) {
      robot.robot_flag.chassis_dial_feeder_flag = 1;
      robot.robot_flag.chassis_dial_eater_flag = 1;
    } else { // 象征性的防止一下卡弹
      shooter_debug.is_dial_stalling = robot.robot_flag.chassis_dial_stall_flag;
      if (shooter_debug.is_dial_stalling == true) {
        robot.robot_flag.chassis_dial_feeder_flag = 0;
        robot.robot_flag.chassis_dial_eater_flag = 1;
      } else if (shooter_debug.is_dial_stalling == false) {
        robot.robot_flag.chassis_dial_feeder_flag = 1;
        robot.robot_flag.chassis_dial_eater_flag = 0;
      }
    }

    // 更新状态

    shooter_debug.is_in_heatlimit =
        ((referee_info.shooter_barrel_heat_limit -
          referee_info.shooter_id1_42mm_cooling_heat) < 99);
    shooter_debug.is_friction_wheel_on = is_friction_wheel_on();
    shooter_debug.is_signal_to_fire =
        (robot.weapon.f_is_fire() ||
         (robot.robot_flag.gimbal_fuck_mode_flag == true));

    // is_shooter_cool_down = !shooter_debug.is_in_heatlimit;

    is_shooter_cool_down = (robot.robot_flag.gimbal_fuck_mode_flag == false &&
                            (shooter_debug.is_in_heatlimit == false ||
                             test_ignore_heat_lim == true)) ||
                           (robot.robot_flag.gimbal_fuck_mode_flag == true);

    bool shoot_within_heat_limit = (shooter_debug.is_signal_to_fire == true &&
                                    is_shooter_cool_down == true);
    if (shoot_within_heat_limit == true) {
      fire_status = fire_status_firing;
    }
    break;
  }
  case fire_status_firing: {
    lfm_expt_speed = mps2rpm_front(front_wheel_v);
    rfm_expt_speed = -mps2rpm_front(front_wheel_v);

    bool is_enable_shoot_delay =
        (robot.robot_flag.gimbal_fuck_mode_flag == false);
    if (push_bullet(is_enable_shoot_delay) == true) {
      fire_status = fire_status_fired;
      if (robot.robot_flag.gimbal_fuck_mode_flag == 1) {
        if (shoot_cnt) {
          shoot_cnt--;

        } else {
          shoot_cnt = 0;
          shooter_debug.is_signal_to_fire = 0;
        }
      } else {
        shooter_debug.is_signal_to_fire = 0;
      }
      tickstart = HAL_GetTick();
    }
    break;
  }
  case fire_status_fired:
    lfm_expt_speed = mps2rpm_front(front_wheel_v);
    rfm_expt_speed = -mps2rpm_front(front_wheel_v);

    if (robot.robot_flag.gimbal_fuck_mode_flag == false) {
      robot.robot_flag.chassis_dial_feeder_flag = 1;
      robot.robot_flag.chassis_dial_eater_flag = 1;
    }
    if (HAL_GetTick() - tickstart > shooter_debug.t1) {
      shooter_debug.is_load_over = false;
      if (shoot_cnt)
        shoot_cnt--;
      fire_status = fire_status_prefill;

      // robot.robot_flag.chassis_dial_feeder_flag = 1;
      //  transmit_direct_BTB_msg(&BTB_CAN,);
    }
    break;

  default:
    break;
  }
  get_shoot_delay();
  // 刷新pid
  shooter_controller();
  firectrl_controller();

  if (fire_status == fire_status_idle) {
    left_friction_motor.output = 0;
    right_friction_motor.output = 0;

    firectrl_motor.output = 0;
  }
  get_v_ball_error();
}

/**
 *	@brief	检查一秒内摩擦轮的误差
 */
uint16_t sample_time = 1000;
int min_speed[4] = {0};
int max_speed[4] = {0};
float error_of_friction[4] = {0};
float temp[4] = {0};
void get_v_ball_error(void) {
  temp[0] = left_friction_motor.real.tempture;
  temp[1] = right_friction_motor.real.tempture;

  static uint32_t tickstart = 0;
  if (HAL_GetTick() - tickstart < sample_time) {
    if (left_friction_motor.real.speed_rpm < min_speed[0])
      min_speed[0] = left_friction_motor.real.speed_rpm;
    else if (left_friction_motor.real.speed_rpm > max_speed[0])
      max_speed[0] = left_friction_motor.real.speed_rpm;

    if (right_friction_motor.real.speed_rpm < min_speed[1])
      min_speed[1] = right_friction_motor.real.speed_rpm;
    else if (right_friction_motor.real.speed_rpm > max_speed[1])
      max_speed[1] = right_friction_motor.real.speed_rpm;

  } else {
    if (left_friction_motor.expt.speed_rpm == 0)
      error_of_friction[0] = -1;
    else
      error_of_friction[0] = (float)(max_speed[0] - min_speed[0]) /
                             left_friction_motor.expt.speed_rpm;

    if (right_friction_motor.expt.speed_rpm == 0)
      error_of_friction[1] = -1;
    else
      error_of_friction[1] = -(float)(max_speed[1] - min_speed[1]) /
                             right_friction_motor.expt.speed_rpm;

    max_speed[0] = left_friction_motor.real.speed_rpm;
    max_speed[1] = right_friction_motor.real.speed_rpm;

    min_speed[0] = left_friction_motor.real.speed_rpm;
    min_speed[1] = right_friction_motor.real.speed_rpm;

    tickstart = HAL_GetTick();
  }
}

/**
 *	@brief	获得从发出开火指令到发射成功的用时
 */
bool tickstart_bit = false;
bool is_friction_wheel_slow_down = false; // 加速时间过短,有时检测不到
bool is_friction_wheel_accelerate = false;
float dv_l, dv_r, dv_bl, dv_br; // 已同号处理,减速为负号
float a_down = 500;             // 最高近900,但不易检测出
float a_up = 50;                // 最高近200

void get_shoot_delay(void) {
  static uint32_t delay_10ms_cnt = 0;
  delay_10ms_cnt++;
  if (delay_10ms_cnt >= 5) { // 改大点可以减小噪声
    delay_10ms_cnt = 0;
    static float last_lfm_speed, last_rfm_speed;
    dv_l = (lfm_real_speed - last_lfm_speed);
    dv_r = -(rfm_real_speed - last_rfm_speed);

    last_lfm_speed = lfm_real_speed;
    last_rfm_speed = rfm_real_speed;
    // last_lfm_back_speed = lfm_back_real_speed;
    // last_rfm_back_speed = rfm_back_real_speed;
  }

  static uint32_t tickstart;
  if (tickstart_bit == false && fire_status == fire_status_firing &&
      HAL_GetTick() - tickstart > shoot_delay + 500) {
    tickstart_bit = true;
    tickstart = HAL_GetTick();
  }

  if (tickstart_bit == true) { // 开始发弹
    // if(dv_l<-a_down&&dv_r>a_down)	is_friction_wheel_slow_down=true;
    shoot_delay = HAL_GetTick() - tickstart;
    // if ((dv_l > a_up && dv_r > a_up && dv_bl > a_up && dv_br > a_up) ||
    // shoot_delay > 1000)
    if ((dv_l > a_up && dv_r > a_up) || shoot_delay > 1000) {
      tickstart_bit = false;
    }
  }
}
void clear_ammo_num(void) { shoot_cnt = 0; }