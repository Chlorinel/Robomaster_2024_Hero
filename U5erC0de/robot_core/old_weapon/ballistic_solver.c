#include "./robot_core/weapon/ballistic_solver.h"
#include "./algorithm/ballistic.h"
#include "./algorithm/filter.h"
#include "./algorithm/util.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/weapon/fire_ctrl.h"
#include "./robot_core/weapon/vision.h"

float predict_time = 0.6f;
float raw_predict_time = 0.f;

float MAX_PREDICT_T = 5.f;
float CONST_SHOOT_DELAY = 0.12f; // 固定发射延时
float EXTERNAL_DELAY = 0.2f;     // 0.17f;
// 额外的发射延时,可能发弹延时不准,可能视觉拍的不是实时画面,可能通讯时间也不对,最终就凑出来离谱的近200ms额外延时
float EXTERNAL_DELAY1 = 0.15f; // 0.587f;
// 去除发弹延时的修正量

#define INIT_SHOOT_SPEED 15.2f   // 默认氮素
float cur_v0 = INIT_SHOOT_SPEED; // 氮素

kalman1_state ks = {
    .x = INIT_SHOOT_SPEED,
    .p = 0.0f, /* estimated error convariance */
    .A = 1,
    .H = 1,
    .q = 0.01f, // 10e-6;  /* predict noise convariance */
    .r = 0.4f   // 10e-5;  /* measure error convariance */
};

target_spec_t target;
ballistic_sol_t solution;

float *get_p_cur_v0(void) { return &cur_v0; }
float *get_p_cur_v0_kalman_x(void) { return &ks.x; }
float get_predict_time(void) { return predict_time; }

float last_bullet_speed = INIT_SHOOT_SPEED;
void update_cur_v0_kalman(void) {
  if (last_bullet_speed != referee_info.bullet_speed) {
    last_bullet_speed = referee_info.bullet_speed;
    // cur_v0 = kalman1_filter(&ks, referee_info.bullet_speed);
    cur_v0 = mean_filter_2(referee_info.bullet_speed);
  }
}

// void update_predict_time(void) {
//   if (vision_ctrl_data.target_found) {
//     if (cur_v0 != 0.0f) {
//       raw_predict_time =
//           target.x0 / (cur_v0 * cosf(vision_request.pitch)) //
//           斜抛运动发射耗时
//           + CONST_SHOOT_DELAY;
//       //+ CONST_SHOOT_DELAY
//       //// 弹丸加速用时
//       //+ EXTERNAL_DELAY;
//       //// 视觉解算及通讯时间延时
//       //+ (HAL_GetTick()-last_frame_tick);
//       ////USB通讯用时
//     } else
//       raw_predict_time = CONST_SHOOT_DELAY;
//     CLAMP(raw_predict_time, 0.0f, MAX_PREDICT_T);
//     predict_time = raw_predict_time;
//   } else {
//     predict_time = 0.0f;
//   }
// }

float shooter_yaw_offset = 0.0126f - 0.0015625f;
float shooter_pitch_offset = 0;

void get_ballistic_calc(est_postion_t est_position, float *pitch_ang,
                        float *yaw_ang) {
  if (pitch_ang != NULL && yaw_ang != NULL) {
    float est_x = est_position.x;
    float est_y = est_position.y;
    float est_z = est_position.z;

    float distance_xy = sqrtf(est_x * est_x + est_y * est_y);

    *yaw_ang = atan2f(est_y, est_x) + shooter_yaw_offset;

    target.x0 = distance_xy;
    target.z0 = est_z;
    //			target.x0 = 5.04;
    //			target.z0 = -0.4;
    projectile_solve(cur_v0, &target, &solution);

    if (solution.solution_num > 0) {
      if (solution.ang_solution1 < solution.ang_solution2) {
        *pitch_ang = solution.ang_solution1 + shooter_pitch_offset;
      } else {
        *pitch_ang = solution.ang_solution2 + shooter_pitch_offset;
      }
    } else {
      *pitch_ang = atan2f(est_z, distance_xy);
    }
  }
}