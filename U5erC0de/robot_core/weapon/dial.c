#include "./algorithm/filter.h"
#include "./base_drv/drv_conf.h"
// #include classis_module_core_h
#include "./robot_core/ctrl_core/chassis.h"
// #include classis_module_motor_ctrl_h
#include "./robot_core/ctrl_core/chassis_motor_ctrl.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/weapon/dial.h"
extern chassis_motors_t chassis_motors;
uint32_t delay_t = 200;
#if 0
#define delay_feed_deg 4
#define delay_feed_t 5
#define delay_eat_deg 6
#define delay_eat_t 20
#else
int32_t dial_feed_deg = 2;
int32_t dial_feed_t = 1;

#endif

#if 1
#define dial_disable                                                           \
  (dial_motor.expt.abs_angle = dial_motor.real.abs_angle, dial_motor.output = 0)
#define dial_eater(_ang, _t) (dial_motor.expt.abs_angle -= (float)deg2rad(_ang))
#define dial_feeder(_ang, _t)                                                  \
  (dial_motor.expt.abs_angle += (float)deg2rad(_ang))
#define dial_lock (dial_motor.expt.abs_angle = dial_lock_angle)
#define get_dial_expt_mode                                                     \
  (robot.robot_flag.chassis_dial_feeder_flag << 1 |                            \
   robot.robot_flag.chassis_dial_eater_flag)
bool dial_free = false;
LPF_t dial_current_filter;
float dial_filter_current = 0;
struct {

  bool is_dial_feed;
  bool is_dial_eat;
  bool is_load_over;

  dial_mode_t dial_mode;

  bool is_stall;

  enum {
    nop = 0,
    forward,
    backward,
  } dial_dirc,
      stall_dirc;
  float first_stall_angle;
  uint16_t stall_t; // 堵转恢复用时
  uint16_t forward_current;
  uint16_t backward_current;
  bool trigger[10];

} dial_debug = {
    .dial_dirc = forward,
    .forward_current = 10000,
    .backward_current = 10000,
    .stall_t = 300,
};

uint32_t free_lock = 0;
// uint8_t MAX_free_lock=10;
/**
 * @brief   卡弹判断
 * @note		电机送弹,输出电流为负
 */
int dial_stall_current = 0;
float stall_energy = 0;
float MAX_stall_energy = 50; // 20;//<越大堵转检测越保守(不容易触发)
int dial_idle_current = 13000; // 9000//<同上
bool is_dial_stall(void) {
  // 计算是否堵转
  dial_stall_current =
      dial_filter_current - sign(dial_filter_current) * dial_idle_current;
  if (sign(dial_stall_current) != sign(dial_filter_current) // 减猛了
  )
    dial_stall_current = 0;

  if (stall_energy >= MAX_stall_energy) {
    return true;
  } else {
    stall_energy += abs(dial_stall_current) / chs_tim_freq;
    return false;
  }
}

/**
 * @brief	判断拨盘是否没弹了
 * @note
 */
bool is_bullet_depleted_lock_bit = false;
float MAX_dial_speed = 1000;
bool is_bullet_depleted(void) {
  is_bullet_depleted_lock_bit =
      abs(dial_motor.real.speed_rpm) > MAX_dial_speed &&
      (0 < dial_stall_current && dial_stall_current < 2000);
  return is_bullet_depleted_lock_bit;
}

float stall_eater_angle = 60;
float stall_feeder_angle = 60;

uint16_t is_dial_free = 0;
LPF_t dial_current_filter = {.fc = 1, .ts = 0.001};

uint32_t stall_st_tick = 0;
uint32_t max_fw_eat_tm = 50; // 向前旋转时堵转后,运行反转的耗时
uint16_t dial_stall_tm = 0;
/**
 * @brief   送弹循环
 * @details 防卡弹逻辑
 *    电流积分检测堵转(一旦成立则自锁)
 * -->shooter根据发弹状态判断是否应该退弹
 * -->执行退弹角度60°的退弹程序,最大退弹耗时max_fw_eat_tm(50ms),超了就清除堵转标识
 * @note [1]记得铲屎,get_dial_expt_mode改用单u8(enum)传递
 *       [2]dial_free可以关掉
 */
void dial_control_loop(void) {
  is_dial_free = free_lock * 10;
  dial_filter_current =
      LPF_update(&dial_current_filter, dial_motor.real.current);

  static float dial_lock_angle = 0;
  if (robot.robot_flag.chassis_dial_feeder_flag ||
      robot.robot_flag.chassis_dial_eater_flag) {
    dial_debug.dial_mode = (dial_mode_t)get_dial_expt_mode;

    switch (get_dial_expt_mode) {
    case disable:
      stall_energy = 0; // 清一下
      dial_disable;
      break;
    case eatting: {
      // 拨盘上一次是往前转然后卡弹,需要反转
      // 反转
      dial_motor.expt.abs_angle =
          dial_debug.first_stall_angle - deg2rad(stall_eater_angle);
      // 负号为退弹
      free_lock = 0;
      dial_free = false;

      if (HAL_GetTick() - stall_st_tick >= max_fw_eat_tm) {
        stall_energy = 0;
        stall_st_tick = HAL_GetTick();
      }
      dial_lock_angle = dial_real_abs_angle;
      break;
    }
    case feeding: {
      dial_debug.dial_dirc = forward;
      stall_st_tick = HAL_GetTick();
      dial_free = false;
      dial_feeder(dial_feed_deg, dial_feed_t);
      dial_debug.first_stall_angle = dial_motor.real.abs_angle;
      dial_lock_angle = dial_real_abs_angle;
      break;
    }
    case lock: {
      dial_free = false;
      dial_lock;
      dial_debug.is_load_over = true;
      break;
    }
    case no_bullet:
      break;
    }
    // 判断堵转与否
    if (is_dial_stall() == true) {
      dial_debug.is_stall = true;
      robot.robot_flag.chassis_dial_stall_flag = true;
    } else {
      dial_debug.is_stall = false;
      robot.robot_flag.chassis_dial_stall_flag = false;
    }

    float output_max = 0;
    if (free_lock == false) {
      if (is_bullet_depleted() == true) {
        output_max = MAX_dial_speed;
      } else {
        output_max = C620_OUTPUT_MAX / 10;
      }
      dial_controller(output_max);
    } else {
      dial_motor.output = 0;
      free_lock--;
    }
  } else {
    dial_disable;
    dial_motor.expt.abs_angle = dial_motor.real.abs_angle;
  }
} // aaa
#endif