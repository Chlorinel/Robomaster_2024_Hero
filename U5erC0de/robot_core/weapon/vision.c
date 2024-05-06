#include "./robot_core/weapon/vision.h"

#include <math.h>
#include <string.h>

#include "./algorithm/ballistic.h"
#include "./algorithm/zero_new_bee_filter.h"

#include "./algorithm/cordic.h"
#include "./algorithm/crc.h"
#include "./algorithm/filter.h"
#include "./algorithm/util.h"
#include "./base_drv/drv_imu/imu.h"
#include "usbd_cdc_if.h"
#include gimbal_module_core_h
#include "./drv_referee/referee.h"

#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/weapon/shooter.h"

uint16_t vision_rx_lost = VISION_RX_LOST_MAX;
uint16_t vision_move_rx_lost = VISION_MOVE_LOST_MAX;
uint8_t vision_req_flag = 0;

vision_req_t vision_request = {0};
vision_ctrl_t vision_ctrl_data = {0};

float current_pitch, current_yaw;
float vision_pitch, vision_yaw;

float MAX_PREDICT_T = 5.f;
#define INIT_SHOOT_DELAY 0.30f
float CONST_SHOOT_DELAY = 0.32; // 0.25f; // 固定发射延时
// 0807
//->0.32	先0.3滞后,调高到0.32后过几小时又超前
//  0806
//->0.25	到深圳后第一晚取消发弹延迟调出的参数
//  0727
//->0.10	左转小寄不算寄,
//->0.18	晚上好但又寄了
//->0.25	超前
//->0.18	正中
//  0726
//->0.12	超前
//->0.08	滞后
float EXTERNAL_DELAY = 0.2f; // 0.17f;
// 额外的发射延时,可能发弹延时不准,可能视觉拍的不是实时画面,可能通讯时间也不对,最终就凑出来离谱的近200ms额外延时
float EXTERNAL_DELAY1 = 0.15f; // 0.587f;
// 去除发弹延时的修正量
#define MAX_TARGET_LOST 60

float predict_time = 0.6f;
float raw_predict_time = 0.f;

extern float cur_v0;

extern eular_t _imu_eular;

float est_x, est_y, est_z, est_yaw, distance_xy;
// 从弹丸飞出,到预测击打目标装甲板的位置差
float best_attack_x, best_attack_y, best_attack_z, best_attack_yaw;
// 从发出开火指令,到预测击打目标装甲板的位置差

// anti-spinning variance
uint8_t shoot_flag = 0;
uint8_t spin_flag = 0;
vision_ctrl_t spinning_target;

/**
 * @brief      return the self color for vision
 * @return     0 for red, others for blue
 */
uint8_t vision_color;
uint8_t is_red_or_blue(void) { // 蓝给0,红给1

  vision_color = robot.is_bule_or_red;
  if (vision_color = 3) {
    return (referee_info.robot_id < 100);
  } else {
    return robot.is_bule_or_red;
  }
}
LPF_t shoot_delay_filter = {
    .ts = 0.001, .fc = 0.1, .fltr_val = INIT_SHOOT_DELAY};
void update_shoot_delay(void) {
  static uint32_t last_shoot_delay = 0;
  extern uint32_t shoot_delay;
  if (last_shoot_delay != shoot_delay && shoot_delay != 0) {
    // CONST_SHOOT_DELAY=LPF_update(&shoot_delay_filter,shoot_delay);
    last_shoot_delay = shoot_delay;
  }
}
#include "./algorithm/util.h"
vision_ctrl_t raw_vision_ctrl_data = {0};
uint8_t parse_vision_data(uint8_t *buf, uint16_t len) {
  memcpy(&raw_vision_ctrl_data, buf, len);
  if (buf[0] == VISION_UART_CTRL_SOF && len == VISION_CTRL_FRAME_LEN &&
      Verify_CRC16_Check_Sum(&buf[0], VISION_CTRL_FRAME_LEN)) {
    // 计算目标位置信息
    memcpy((uint8_t *)&vision_ctrl_data, buf, VISION_CTRL_FRAME_LEN);

    vision_ctrl_data.checksum =
        (uint16_t)((buf[VISION_CTRL_FRAME_LEN - 1] << 8) |
                   buf[VISION_CTRL_FRAME_LEN - 2]);
    // 刷新预测时间与弹速卡尔曼预测
    if (vision_ctrl_data.target_found) {
      if (cur_v0 != 0.0f) {
        extern target_spec_t target;
        raw_predict_time =
            target.x0 /
                (cur_v0 * cosf(vision_request.pitch)) // 斜抛运动发射耗时
            + CONST_SHOOT_DELAY;
        //+ CONST_SHOOT_DELAY
        //// 弹丸加速用时
        //+ EXTERNAL_DELAY;
        //// 视觉解算及通讯时间延时
        //+ (HAL_GetTick()-last_frame_tick);
        ////USB通讯用时
      } else
        raw_predict_time = CONST_SHOOT_DELAY;
      CLAMP(raw_predict_time, 0.0f, MAX_PREDICT_T);
      predict_time = raw_predict_time;
    } else {
      predict_time = 0.0f;
    }
    // last_frame_tick=HAL_GetTick();
    vision_rx_lost = 0;
    return VISION_DATA_NOERR;
  } else
    return VISION_DATA_ERR;
}
/**
 * @brief	刷新tracker
 * @return	是否reset完毕
 */
#define reset_status_time 1
bool resetTracker(bool is_enable_tracker) {
  extern bool reset_tracker_flag;
  static bool is_first_enable_tracker = false;
  static bool had_reset_tracker = false;
  static uint32_t first_enable_tracker_tick = 0;
  // 刷新标识符
  if (is_enable_tracker == false)
    had_reset_tracker = false;
  // 判断是否为第一次触发tracker
  if (is_enable_tracker == true && is_first_enable_tracker == false &&
      had_reset_tracker == false) { // 判断是否为第一次尝试触发tracker
    is_first_enable_tracker = true;
    first_enable_tracker_tick = HAL_GetTick();
  }
  if (is_first_enable_tracker == true &&
      HAL_GetTick() >=
          first_enable_tracker_tick +
              reset_status_time) { // 第一次尝试触发tracker一定时间后才清除判断标识符
    is_first_enable_tracker = false;
  }

  // 判断是否需要刷新tracker
  if (is_first_enable_tracker == true || HAL_GetTick() < 3000) {
    reset_tracker_flag = true;
    had_reset_tracker = true;
    return false;
  } else {
    reset_tracker_flag = false;
    return true;
  }
}
void increase_camera_exposure() {}
/////////////////////////////////////////////////////////////////////////////////
#define num111 5
uint32_t MAX_time = 1000;
uint32_t MAX_track_time = 2000;
float MAX_yaw_error_in_base_mode = 10;   // in deg
float MAX_pitch_error_in_base_mode = 10; // in deg
bool had_find_base = false; // 是否在此次开基地模式时已检测到过基地
struct {
  /* data */
  uint32_t tick;
  float pitch_ang;
  float yaw_ang;
} vision_base_mode_data[num111];

float base_mode_aim_yaw, base_mode_aim_pitch;
float base_sum_yaw, base_sum_pitch;
uint8_t identify_base_cnt = 0;
float base_mode_yaw_offset = -0.0107f;
float base_mode_pitch_offset = 0.0484;
float base_mode_target_z0_offset = 0;   // 观测值比实际低了约0.2m
float outpost_top_armor_offset = 0.312; // 观测值比实际低了约0.2m

uint8_t get_vision_ctrl_base_mode(float *pitch_ang, float *yaw_ang, float dt) {
  extern bool base_mode_enable; // 抽象吧
  static uint8_t i = 0;
  if (base_mode_enable == false) {
    i = 0;
    had_find_base = false;
    return VISION_NOTARGET;
  }
  // 开了基地模式
  if (i == num111)
    i = 0;
  if (vision_ctrl_data.target_found == true) { // 有收到零琐的数据帧
    vision_base_mode_data[i].tick = HAL_GetTick();
    get_vision_ctrl(&vision_base_mode_data[i].pitch_ang,
                    &vision_base_mode_data[i].yaw_ang, 1.f / hs_tim_freq);
    i++;
  }

  extern gimbal_state_t gimbal_real_state;
  if (i == num111 &&
      vision_base_mode_data[num111 - 1].tick - vision_base_mode_data[0].tick <
          MAX_time) { // 得连续收到多个包才判断一次
    for (uint8_t j = 0; j < num111; j++) {
      float select_yaw = vision_base_mode_data[j].yaw_ang;
      float select_pitch = vision_base_mode_data[j].pitch_ang;
      base_sum_yaw = 0;
      base_sum_pitch = 0;
      identify_base_cnt = 0;
      if (get_delta_ang(select_yaw, gimbal_real_state.yaw, 2 * PI) <
          deg2rad(MAX_yaw_error_in_base_mode)) {
        base_sum_yaw += select_yaw;
        base_sum_pitch += select_pitch;
        identify_base_cnt++;
      }
    }
    if (identify_base_cnt != 0) {
      extern float shooter_yaw_offset, shooter_pitch_offset;
      base_mode_aim_yaw = range_map(base_sum_yaw / identify_base_cnt, -PI, PI) -
                          shooter_yaw_offset + base_mode_yaw_offset;
      base_mode_aim_pitch =
          range_map(base_sum_pitch / identify_base_cnt, -PI, PI) -
          shooter_pitch_offset + base_mode_pitch_offset;
      *yaw_ang = base_mode_aim_yaw;
      *pitch_ang = base_mode_aim_pitch;
      had_find_base = true;
      return VISION_OK;
    } else if (had_find_base == false) // 未发现过基地
    {
      base_mode_aim_yaw = 0;
      base_mode_aim_pitch = 0;
      return VISION_NOTARGET;
    }
  } else if (had_find_base == true) {
    if (HAL_GetTick() - vision_base_mode_data[num111 - 1].tick <
        MAX_track_time) { // 2秒内连续发现有基地,就开轰
      *yaw_ang = base_mode_aim_yaw;
      *pitch_ang = base_mode_aim_pitch;
      return VISION_OK;
    } else { // 超时清零
      had_find_base = false;
      return VISION_NOTARGET;
    }
  } else {
    base_mode_aim_yaw = 0;
    base_mode_aim_pitch = 0;
    return VISION_NOTARGET;
  }
  return VISION_NOTARGET;
}

// 清除标识位
void refresh_base_mode(bool force_clean_base_mode_flag) {
  extern bool base_mode_enable;
  if (force_clean_base_mode_flag == true) {
    base_mode_enable = false;
    had_find_base = false;
    return;
  }

  if (had_find_base == true && base_mode_enable == false &&
      HAL_GetTick() - vision_base_mode_data[0].tick > 2000) {
    had_find_base = false;
  }
}

/**
 * @brief	解算上位机发送过来的数据,最后解算出期望pitch、yaw
 *	@param	pitch_angle,yaw_angle:期望yaw、pitch,顺时针0~2*PI计
 * @param	dt:调用此函数的间隔时间
 */

/**
 * @brief  send upper data
 * @return none
 */
bool base_mode_enable = false;
bool reset_tracker_flag = false;
extern attack_target_type_t attack_target_type;
uint8_t is_play;
void send_vision_request(float current_roll, float current_pitch,
                         float current_yaw) {

  vision_request.header = VISION_UART_REQ_SOF;
  // vision_request.base_mode = (attack_target_type == base_mid_armor ||
  //                             attack_target_type == base_top_armor);

  vision_request.s_timestamp = robot.timestep;
  vision_request.rest_tracker = reset_tracker_flag;
  if (game_status.game_progress == 4) {
    is_play = 1;
  } else {
    is_play = 0;
  }
  vision_request.is_play = is_play;

  vision_request.roll = current_roll;
  vision_request.pitch = current_pitch;
  vision_request.yaw = current_yaw;

  vision_request.aim_x = est_x;
  vision_request.aim_y = est_y;
  vision_request.aim_z = est_z;

  vision_request.local_color = is_red_or_blue(); // VISION_COLOR;

  vision_request.game_time = game_status.stage_remain_time;
  vision_request.checksum = Get_CRC16_Check_Sum(
      (uint8_t *)&vision_request, sizeof(vision_req_t) - 2, CRC16_INIT);

  CDC_Transmit_FS((uint8_t *)&vision_request, sizeof(vision_req_t));
  //  vision_request.exposure_time = false;
}
/**
 * @brief 识别另一侧装甲板的旋转状态
 */
float TARGETED_RATE = 0.2f;
uint16_t SPINNING_SHOOT_OFFSET = 0;
uint16_t spinning_period;
int16_t delay = 0;
uint16_t spinning_tick = 0;
// 函数名称：get_spinning_state
// 输入参数：无
// 输出参数：无
// 功能描述：获取旋转状态，对机器人或物体是否处于旋转状态进行判断和更新

void get_spinning_state(void) {
  // 每次调用该函数，自增一个spinning_tick，表示经过的时间
  spinning_tick++;

  // 定义一个静态变量last_vision_ctrl_data，存储上一次的vision_ctrl_data数据
  static vision_ctrl_t last_vision_ctrl_data;

  // 定义一个静态变量lost_threshold，表示丢失目标阈值，初始化为1000
  static uint16_t lost_threshold = 1000;

  // 如果目标被找到
  if (vision_ctrl_data.target_found) {
    // 如果目标的位置变动超过了阈值0.3，或者机器人正在跟踪目标且时间达到了预设的旋转周期
    if ((fabs(vision_ctrl_data.x - last_vision_ctrl_data.x) +
         fabs(vision_ctrl_data.y - last_vision_ctrl_data.y) +
         fabs(vision_ctrl_data.z - last_vision_ctrl_data.z)) > 0.3f

        || ((spinning_tick <= TARGETED_RATE * spinning_period) &&
            spinning_tick > 0)) {
      // 更新旋转周期为当前的tick数，并将tick数归零
      spinning_period = spinning_tick;
      spinning_tick = 0;

      // 如果spin_flag为假，则将其设为真
      if (!spin_flag) {
        spin_flag = 1;
      } else {
        // 否则，将当前的vision_ctrl_data设为旋转目标
        spinning_target = vision_ctrl_data;
      }
    }

    // 更新last_vision_ctrl_data为当前的vision_ctrl_data，以便下一次使用
    last_vision_ctrl_data = vision_ctrl_data;
  }

  // 如果tick数超过了阈值lost_threshold，则表示目标丢失，将spin_flag设为假，并重置tick数和shoot_flag
  if (spinning_tick > lost_threshold) {
    spin_flag = 0;
    spinning_tick = 0;
    shoot_flag = 0;
  } else if ((spinning_period - spinning_tick) <
                 delay + SPINNING_SHOOT_OFFSET &&
             (spinning_period - spinning_tick) > SPINNING_SHOOT_OFFSET &&
             spin_flag) {
    // 如果满足条件，则将shoot_flag设为真，表示可以进行射击操作
    shoot_flag = 1;
  } else {
    // 否则，将shoot_flag设为假，表示不可以进行射击操作
    shoot_flag = 0;
  }
}
/**
 * @brief  check if vision upper is offline
 * @return 0 for  vision upper online, others for offline
 */
bool is_vision_offline(void) {
  if (vision_rx_lost >= VISION_RX_LOST_MAX) {
    return true;
  } else {
    return false;
  }
} // return vision_rx_lost >= VISION_RX_LOST_MAX; }

/**
 * @brief  increment of vision_rx_lost counter, should be call after process of
 * all rc data
 * @return none
 */
void inc_vision_rx_lost(void) {
  if (vision_rx_lost < VISION_RX_LOST_MAX)
    vision_rx_lost++;
}

/**
 * @brief  check vision request flag
 * @return 1 for vision upper request, others for not receive
 * @note calling this function will set vision_rx_flag to zero
 */
uint8_t is_vision_req(void) {
  uint8_t rx = vision_req_flag;
  vision_req_flag = 0;
  return rx;
}
