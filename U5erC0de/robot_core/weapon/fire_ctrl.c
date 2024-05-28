#include "./algorithm/ballistic.h"
#include "./algorithm/filter.h"
#include "./algorithm/util.h"
#include "./algorithm/zero_new_bee_filter.h"
#include "./robot_core/ctrl_core/robot.h"

#include gimbal_module_core_h
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/tool/bus_detect.h"
#include "./robot_core/weapon/fire_ctrl_param.h"
#include "./robot_core/weapon/vision.h"
#define MIDDLE 0
float cur_v0 = INIT_SHOOT_SPEED; // 氮素
float last_bullet_speed = INIT_SHOOT_SPEED;

// 旋转矩阵（Roll, Pitch, Yaw）的3x3矩阵表示
typedef struct {
  float m[3][3];
} Matrix3x3;

Matrix3x3 matrix_multiply(Matrix3x3 a, Matrix3x3 b) {
  Matrix3x3 result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.m[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        result.m[i][j] += a.m[i][k] * b.m[k][j];
      }
    }
  }
  return result;
}
// 计算旋转矩阵（Roll, Pitch, Yaw）
Matrix3x3 rotation_matrix(float roll, float pitch, float yaw) {
  Matrix3x3 R_x, R_y, R_z, R;

  float cr = cos(roll);
  float sr = sin(roll);
  float cp = cos(pitch);
  float sp = sin(pitch);
  float cy = cos(yaw);
  float sy = sin(yaw);

  R_x.m[0][0] = 1;
  R_x.m[0][1] = 0;
  R_x.m[0][2] = 0;
  R_x.m[1][0] = 0;
  R_x.m[1][1] = cr;
  R_x.m[1][2] = -sr;
  R_x.m[2][0] = 0;
  R_x.m[2][1] = sr;
  R_x.m[2][2] = cr;

  R_y.m[0][0] = cp;
  R_y.m[0][1] = 0;
  R_y.m[0][2] = sp;
  R_y.m[1][0] = 0;
  R_y.m[1][1] = 1;
  R_y.m[1][2] = 0;
  R_y.m[2][0] = -sp;
  R_y.m[2][1] = 0;
  R_y.m[2][2] = cp;

  R_z.m[0][0] = cy;
  R_z.m[0][1] = -sy;
  R_z.m[0][2] = 0;
  R_z.m[1][0] = sy;
  R_z.m[1][1] = cy;
  R_z.m[1][2] = 0;
  R_z.m[2][0] = 0;
  R_z.m[2][1] = 0;
  R_z.m[2][2] = 1;

  R = matrix_multiply(matrix_multiply(R_z, R_y), R_x);

  return R;
}
// 从单位方向向量近似欧拉角（Pitch, Yaw）
void approximate_euler_from_vector(float vx, float vy, float vz, float *pitch,
                                   float *yaw) {
  // 注意：这里我们忽略了roll，因为输入向量已经是在特定roll下的
  // 计算pitch（俯仰角，绕Y轴旋转）
  if (vz < -1)
    vz = -1; // 避免除零和arcsin定义域问题
  if (vz > 1)
    vz = 1;
  *pitch = -atan2(vz, sqrt(vx * vx + vy * vy));
  // 计算yaw（偏航角
  // 绕Z轴旋转，但这里我们需要根据X和Y分量）
  *yaw = atan2(vy, vx);
}

float get_cur_v0(void) { return cur_v0; }
void force_resfresh_cur_v0(float new_bullet_speed) {
  cur_v0 = new_bullet_speed;
  for (uint8_t i = 0; i < 3; i++)
    mean_filter_2(new_bullet_speed);
}
/**
 * @brief   刷新弹速滤波器
 */
void update_cur_v0_filter(void) {
  if (last_bullet_speed != referee_info.bullet_speed &&
      referee_info.bullet_speed != 0) {
    last_bullet_speed = referee_info.bullet_speed;
    // cur_v0 = kalman1_filter(&ks, referee_info.bullet_speed);
    float clamp_bullet_speed =
        CLAMP(last_bullet_speed, MIN_SHOOT_SPEED, MAX_SHOOT_SPEED);
    cur_v0 = mean_filter_2(clamp_bullet_speed);
  }
}

LPF_t xc_filter, yc_filter, zc_filter, yaw_filter;
LPF_t base_xc_filter, base_yc_filter, base_zc_filter;
LPF_t vx_filter, vy_filter, vz_filter, vyaw_filter;
LPF_t rc_filter, rd_filter;
LPF_t outpost_armor_v_filter;
sin_data_t outpost_armor_yaw = {
    .A = 0.226,           // Initial guess for A
    .omega = 2.5,         // Initial guess for omega
    .phi = 0,             // Initial guess for phi
    .learning_rate = 0.01 // Learning rate for gradient descent
};
outpost_armor_t outpost = {0};
attack_target_type_t attack_target_type;
float count_tset;
void est_position_filter_init(void) {
  LPF_init(&xc_filter, 1.f / fs_tim_freq, 1);
  LPF_init(&yc_filter, 1.f / fs_tim_freq, 1);
  LPF_init(&zc_filter, 1.f / fs_tim_freq, 1);

  LPF_init(&base_xc_filter, 1.f / fs_tim_freq, count_tset);
  LPF_init(&base_yc_filter, 1.f / fs_tim_freq, count_tset);
  LPF_init(&base_zc_filter, 1.f / fs_tim_freq, count_tset);

  LPF_init(&vx_filter, 1.f / fs_tim_freq, 1);
  LPF_init(&vy_filter, 1.f / fs_tim_freq, 1);
  LPF_init(&vz_filter, 1.f / fs_tim_freq, 1);

  LPF_init(&yaw_filter, 1.f / fs_tim_freq, 1);
  LPF_init(&rc_filter, 1.f / fs_tim_freq, 0.1);
  LPF_init(&rd_filter, 1.f / fs_tim_freq, 0.1);
  LPF_init(&vyaw_filter, 1.f / fs_tim_freq, 0.1);

  LPF_init(&outpost_armor_v_filter, 1.f / fs_tim_freq, 0.1);
} /**
   * @brief   刷新低通滤波器
   */
void refresh_LPF_filter(void) {
  xc_filter.fltr_val = vision_ctrl_data.x;
  yc_filter.fltr_val = vision_ctrl_data.y;
  zc_filter.fltr_val = vision_ctrl_data.z;
  base_xc_filter.fltr_val = vision_ctrl_data.x;
  base_yc_filter.fltr_val = vision_ctrl_data.y;
  base_zc_filter.fltr_val = vision_ctrl_data.z;
  vx_filter.fltr_val = vision_ctrl_data.vx;
  vy_filter.fltr_val = vision_ctrl_data.vy;
  vz_filter.fltr_val = vision_ctrl_data.vz;
  vyaw_filter.fltr_val = vision_ctrl_data.v_yaw;
  float r1 = vision_ctrl_data.r1, r2 = vision_ctrl_data.r1;
  rc_filter.fltr_val = max(r1, r2);
  rd_filter.fltr_val = min(r1, r2);

  float outpost_armor_v = vision_ctrl_data.v_yaw * vision_ctrl_data.r1;
  outpost_armor_v_filter.fltr_val = outpost_armor_v;
}
/**
 * @brief   选择击打模式:
 *              1、前哨站顶部装甲板
 *              2、前哨站旋转装甲板
 *              3、基地中间装甲板
 *              4、其他
 */
void get_vision_aim_mode(void) {
  extern bool base_mode_enable;
  if (base_mode_enable == true && (vision_ctrl_data.target_found == true &&
                                   vision_ctrl_data.armor_num == 3)) {
    attack_target_type = outpost_top_armor;
  } else if (base_mode_enable == false &&
             (vision_ctrl_data.target_found == true &&
              vision_ctrl_data.armor_num == 3)) {
    attack_target_type = outpost_spin_armor;
  } else if (base_mode_enable == true &&
             (vision_ctrl_data.target_found == false ||
              (vision_ctrl_data.target_found == true &&
               vision_ctrl_data.armor_num != 3))) {
    attack_target_type = base_mid_armor;
  } else if (base_mode_enable == false) {
    attack_target_type = other;
  }
  if (base_mode_enable == true) {

  } else {
  }
  // 抽象吧
}
/**
 * @brief   视觉数据自动开火控制
 */
float vyaw_threshold = 0.5f;
float delta_R, delta_angle;
enum {
  STOP = 0,
  SPIN_FOLLOW = 1, // 目标在转,且击打目标是要打的目标
  SPIN_WAIT = 2,   // 目标在转,但击打目标非要打的目标
} aim_spin_status;

float outpost_vyaw_level0 = 1.25;
float outpost_vyaw_level1 = 2.5;

float const_outpost_R = 0.270; // 0.29; // 0.27 // 0.226;

float offset_theta = 30; // 不击打刚出来那一段 //0
                         // //fire_config_debug[0];

float k_theta_omega = 10; // 云台跟随角度振幅与云台跟随角频率之积//5
float aim_mid = 0.001;
float gimbal_follow = 0.004;
float boundary_theta; // 可使选取比理想的边界小点	//59

float vyaw_bound = 5.f;
float choose_speed_vyaw = 0.5;
float armor_theta_diff = 0;
float distance_offset;
float zc_offest = 0.12;
float center_theta;
float last_xc;
float distance_xyz;
float gimbal_follow_omega;   // 云台yaw最大震荡频率
float gimbal_follow_boudary; // 云台yaw最大摆动幅度
float gimbal_follow_theta;

extern float shooter_yaw_offset, shooter_pitch_offset;
float debug[12] = {0}, debu[12];
target_spec_t target;
target_spec_t center_target;
ballistic_sol_t center_solution;
ballistic_sol_t solution;
float r_freq_vision = 0;
float shooter_yaw_offset_temp, shooter_pitch_offset_temp;
float temp_pitch_angle, temp_yaw_angle;
uint8_t get_vision_ctrl(float *pitch_ang, float *yaw_ang,
                        float *shooter_yaw_ang, float dt, bool is_center_fire) {
  if (vision_ctrl_data.target_found) {
    float x = vision_ctrl_data.x;
    get_data_refresh_freq(x, r_freq_vision);
    // 在重启tracker后刷新LPF
    static uint32_t first_reset_tracker_tick = 0;
    if (vision_request.rest_tracker == true) {
      first_reset_tracker_tick = HAL_GetTick();
    }
    if (HAL_GetTick() <
        first_reset_tracker_tick +
            25) { // 触发resetTracker 25ms后,此时视觉数据才稳定,更新LPF
      refresh_LPF_filter();
      // 不接入前25ms的数据
      return VISION_NOTARGET;
    }

    // 载入数据

    float yaw = vision_ctrl_data.yaw;
    float r1 = vision_ctrl_data.r1;
    float r2 = vision_ctrl_data.r2;
    float xc = vision_ctrl_data.x;
    float yc = vision_ctrl_data.y;
    float zc = vision_ctrl_data.z;
    float vx = vision_ctrl_data.vx;
    float vy = vision_ctrl_data.vy;
    float vz = vision_ctrl_data.vz;
    float vyaw = vision_ctrl_data.v_yaw;
    float dz = vision_ctrl_data.dz;
    float armor_num = vision_ctrl_data.armor_num;
    // uint8_t id_num = vision_ctrl_data.id_num;
    extern float predict_time;
    extern float est_x, est_y, est_z, est_yaw, distance_xy;
    extern float center_est_x, center_est_y, center_distance_xy;
    extern float best_attack_x, best_attack_y, best_attack_z, best_attack_yaw;
    // 前哨站特化
    if (attack_target_type == outpost_spin_armor ||
        attack_target_type == outpost_top_armor) {
      xc = LPF_update(&xc_filter, xc);
      yc = LPF_update(&yc_filter, yc);
      zc = LPF_update(&zc_filter, zc);
      // yaw=LPF_update(&yaw_filter,yaw);
      vx = 0;
      vy = 0;
      vz = 0;
      dz = 0;
      r1 = LPF_update(&rd_filter,
                      r1); // const_outpost_R; // LPF_update(&rd_filter, r1);
      rd_filter.fltr_val = const_outpost_R;
      r2 = r1;

      // float outpost_armor_v = vision_ctrl_data.v_yaw * vision_ctrl_data.r1;
      // LPF_update(&outpost_armor_v_filter, outpost_armor_v);
      // vyaw = outpost_armor_v / const_outpost_R;
      // CLAMP_MAX(vyaw, 3);
      vyaw = LPF_update(&vyaw_filter, vyaw);

      yaw_filter.fc = vyaw;
      //      if (vyaw > 2) {
      //        vyaw = -1 * vyaw;
      //        yaw = -1 * yaw;
      //      }
      // if(vyaw>2)
      // {shooter_yaw_offset=0.02;}
      // else if(vyaw<-2)
      // {shooter_yaw_offset=0;}
      // bool is_armor_jump = vision_ctrl_data.is_armor_jump;
      // calculate_outpost_yaw(&outpost, is_armor_jump, yaw);
      // write_sensor_data(&outpost_armor_yaw, outpost.track_yaw,
      // HAL_GetTick()); update_sin_data(&outpost_armor_yaw);
      if (attack_target_type == outpost_spin_armor) {
        k_theta_omega = gimbal_follow;
      } else if (attack_target_type == outpost_top_armor) {
        k_theta_omega = aim_mid;
      }
    }
    // 基地模式特化

    else if (attack_target_type == base_mid_armor ||
             attack_target_type == base_top_armor) {
      extern bool had_find_base;
      if (last_xc == 0 && xc != 0 && had_find_base != 1) {
        xc = LPF_update(&base_xc_filter, xc);
        xc = LPF_update(&base_xc_filter, xc);
        xc = LPF_update(&base_xc_filter, xc);
        yc = LPF_update(&base_yc_filter, yc);
        zc = LPF_update(&base_zc_filter, zc);
        yc = LPF_update(&base_yc_filter, yc);
        zc = LPF_update(&base_zc_filter, zc);
        yc = LPF_update(&base_yc_filter, yc);
        zc = LPF_update(&base_zc_filter, zc);
      }

      xc = LPF_update(&xc_filter, xc);
      yc = LPF_update(&yc_filter, yc);
      zc = LPF_update(&zc_filter, zc);
      vx = 0;
      vy = 0;
      vz = 0;
      dz = 0;
      vyaw = 0;
      r1 = LPF_update(&rc_filter, r1);
      r2 = r1;
      last_xc = xc;
      extern float base_mode_target_z0_offset;
      zc += base_mode_target_z0_offset;

      k_theta_omega = aim_mid;
    } else {
      LPF_update(&rc_filter, max(r1, r2));
      LPF_update(&rd_filter, min(r1, r2));
      k_theta_omega = gimbal_follow;
    }

    // 预测
    xc = xc + predict_time * vx;
    yc = yc + predict_time * vy;
    zc = zc + predict_time * vz;
    // float yaw_predict_time = predict_time + (fabs(vyaw) > vyaw_bound ?
    // _shoot_delay : 0.f);
    yaw = yaw + predict_time * vyaw;

    // 选板
    center_theta = atan2(yc, xc); // vision_request.yaw;
    bool use_1 = true;
    float diff_angle = 2 * PI / armor_num;
    float add_theta = deg2rad(offset_theta) * -sign(vyaw);

    armor_theta_diff = PI;
    if ((attack_target_type == outpost_spin_armor ||
         attack_target_type == other) &&
        fabs(vyaw) > choose_speed_vyaw) {
      // 目标转速较快,此时就得选板
      // 确定跟随幅度
      distance_xyz = sqrt(xc * xc + yc * yc + zc * zc);
      gimbal_follow_omega =
          fabs(vyaw) * r1 / distance_xyz; // 云台yaw最大震荡频率
      gimbal_follow_boudary =
          k_theta_omega / gimbal_follow_omega; // 云台yaw最大摆动幅度
      gimbal_follow_theta = gimbal_follow_boudary * distance_xyz / r1;
      CLAMP_MAX(gimbal_follow_theta, 2 * PI / armor_num);
      boundary_theta = rad2deg(PI / armor_num - gimbal_follow_theta / 2.f);
      // 击打点最大波动角度
      if (rad2deg(PI / armor_num) > 5) // 虽然这个判断比较抽象,但还是限一下好点
        CLAMP(boundary_theta, 0, rad2deg(PI / armor_num) - 5 / 2);
      else
        CLAMP(boundary_theta, rad2deg(PI / armor_num) - 5 / 2, 0);

      for (size_t i = 0; i < armor_num; i++) {
        float armor_yaw = yaw + i * diff_angle;
        float yaw_diff = get_delta_ang(armor_yaw, center_theta, 2 * PI);
        debug[7 + i] = yaw_diff;
        if (-PI / armor_num + deg2rad(boundary_theta) + add_theta < yaw_diff &&
            yaw_diff < PI / armor_num - deg2rad(boundary_theta) + add_theta) {
          armor_theta_diff = yaw_diff;
          debug[0] = -PI / armor_num + deg2rad(boundary_theta) + add_theta;
          debug[1] = yaw_diff;
          debug[2] = PI / armor_num - deg2rad(boundary_theta) + add_theta;

          debug[10] = deg2rad(boundary_theta);
          // 获得预瞄装甲板的位置
          float r = r1;
          float _est_z = zc;
          if (armor_num == 4) {
            r = use_1 == true ? r1 : r2; // 现在瞄上的是r1，zc
            _est_z = use_1 == true ? zc : (zc + dz);
          }

          bool is_selected_armor_radius_shorter = (r == min(r1, r2));
          if ( //(fabs(r - rd_filter.fltr_val) < fabs(r - rd_filter.fltr_val)
               //&&
               // armor_num == 4) ||
              (is_selected_armor_radius_shorter && armor_num == 4) ||
              // 4装甲板目标仅打半径小的
              armor_num == 3 || // 前哨站
              armor_num == 2)   // 平步
          {
            aim_spin_status = SPIN_FOLLOW;
            // Robot state to armor
            debu[0] = vx;
            debu[1] = vy;
            debu[2] = vz;
            debu[3] = vyaw;
            debu[4] = armor_yaw;
            debu[5] = center_theta;
            est_x = xc - rd_filter.fltr_val * cos(armor_yaw);

            center_est_x = xc;
            est_y = yc - rd_filter.fltr_val * sin(armor_yaw);
            center_est_y = yc;

            est_z = _est_z;
          } else { // 非半径较小的装甲板,就不打了
            aim_spin_status = SPIN_WAIT;
          }
          break;
        } else {
          aim_spin_status = SPIN_WAIT;
        }
        use_1 = !use_1;
      }
    } else if (attack_target_type == outpost_top_armor) {
      aim_spin_status = STOP;
      extern float outpost_top_armor_offset;
      est_x = xc;
      est_y = yc;
      est_z = zc + outpost_top_armor_offset;
      armor_theta_diff = get_delta_ang(yaw, center_theta, 2 * PI);
    } else if (fabs(vyaw) < choose_speed_vyaw ||
               attack_target_type == base_mid_armor) {
      // 目标转速过低接近停转,此时即可直接跟着去打
      aim_spin_status = STOP;
      est_x = xc - r1 * cos(yaw);
      est_y = yc - r1 * sin(yaw);
      est_z = zc + zc_offest;
      // est_z = real_z;

      armor_theta_diff = get_delta_ang(yaw, center_theta, 2 * PI);
    }

    // 弹道解算
    if (pitch_ang != NULL && yaw_ang != NULL) {
      extern gimbal_state_t gimbal_real_state;
      shooter_yaw_offset_temp =
          shooter_yaw_offset + *robot.weapon.shooter_offset.yaw;
      shooter_pitch_offset_temp =
          shooter_pitch_offset + *robot.weapon.shooter_offset.pitch;

      _shooter_yaw_offset =
          cos(gimbal_real_state.roll) * shooter_yaw_offset_temp +
          sin(gimbal_real_state.roll) * shooter_pitch_offset_temp;
      _shooter_pitch_offset =
          cos(gimbal_real_state.roll) * shooter_pitch_offset_temp -
          sin(gimbal_real_state.roll) * shooter_yaw_offset_temp;

      distance_xy = sqrtf(est_x * est_x + est_y * est_y);

      extern gimbal_state_t gimbal_real_state;
      target.x0 = distance_xy - 0.1f * cos(gimbal_real_state.pitch);
      target.z0 = est_z - 0.1f * sin(-gimbal_real_state.pitch) *
                              cos(gimbal_real_state.roll);

      center_distance_xy =
          sqrtf(center_est_x * center_est_x + center_est_y * center_est_y) -
          rd_filter.fltr_val;
      if (1) {
        distance_xy += distance_offset;
      }
      // distance_xy = real_distance;
      if (is_center_fire) {
        if (aim_spin_status == STOP) {
          *yaw_ang = atan2f(est_y, est_x) + _shooter_yaw_offset;
          *shooter_yaw_ang = *yaw_ang;
        } else {
          *yaw_ang = atan2f(yc, xc) + _shooter_yaw_offset;
          *shooter_yaw_ang = atan2f(est_y, est_x) + _shooter_yaw_offset;
        }
      } else { //
        if (1) {
          temp_yaw_angle = atan2f(est_y, est_x) + _shooter_yaw_offset;
          // *shooter_yaw_ang = temp_yaw_angle;
        } else {
          temp_yaw_angle = atan2f(yc, xc) + _shooter_yaw_offset;
          //*shooter_yaw_ang = atan2f(est_y, est_x) + _shooter_yaw_offset;
          // *shooter_yaw_ang = temp_yaw_angle;
        }
      }
      extern gimbal_state_t gimbal_real_state;
      target.x0 = distance_xy - 0.1f * cos(gimbal_real_state.pitch);
      target.z0 = est_z - 0.1f * sin(-gimbal_real_state.pitch) *
                              cos(gimbal_real_state.roll);

      center_target.x0 =
          center_distance_xy - 0.1f * cos(gimbal_real_state.pitch);
      center_target.z0 = est_z - 0.1f * sin(-gimbal_real_state.pitch) *
                                     cos(gimbal_real_state.roll);
      projectile_solve(cur_v0, &target, &solution);
      projectile_solve(cur_v0, &center_target, &center_solution);
//      extern vision_ctrl_t vision_request;
      if (0) {

        if (center_solution.solution_num > 0) {
          if (center_solution.ang_solution1 < center_solution.ang_solution2) {
            temp_pitch_angle =
                -center_solution.ang_solution1 + _shooter_pitch_offset;
          } else {
            temp_pitch_angle =
                -center_solution.ang_solution2 + _shooter_pitch_offset;
          }
        } else {
          temp_pitch_angle = atan2f(est_z, center_distance_xy);
        }
      } else {

        if (solution.solution_num > 0) {
          if (solution.ang_solution1 < solution.ang_solution2) {
            temp_pitch_angle = -solution.ang_solution1 + _shooter_pitch_offset;
          } else {
            temp_pitch_angle = -solution.ang_solution2 + _shooter_pitch_offset;
          }
        } else {
          temp_pitch_angle = atan2f(est_z, distance_xy);
        }
      }
    }
    if (aim_spin_status == SPIN_WAIT) {
      //			est_x = 0;
      //			est_y = 0;
      //			est_z = 0;
    }
    float initial_vector[3] = {cos(temp_pitch_angle) * cos(temp_yaw_angle),
                               cos(temp_pitch_angle) * sin(temp_yaw_angle),
                               sin(temp_pitch_angle)};
    Matrix3x3 R_initial = rotation_matrix(0, temp_pitch_angle, temp_yaw_angle);
    float rotated_initial_vector[3] = {R_initial.m[2][0], R_initial.m[2][1],
                                       R_initial.m[2][2]};
    Matrix3x3 R_roll = rotation_matrix(vision_request.roll, 0, 0);
    float rotated_vector[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rotated_vector[i] += R_roll.m[i][j] * rotated_initial_vector[j];
      }
    }

    approximate_euler_from_vector(rotated_vector[0], rotated_vector[1],
                                  rotated_vector[2], pitch_ang, yaw_ang);

    predict_time += dt;
    return VISION_OK;
  } else {
#if 0
		if (pitch_ang != NULL && yaw_ang != NULL)
		{
			extern gimbal_state_t gimbal_real_state;
			target.x0 = distance_xy-0.1f*cos(gimbal_real_state.pitch);
			target.z0 = est_z+0.1f*sin(gimbal_real_state.pitch);

			projectile_solve(cur_v0, &target, &solution);

			if (solution.solution_num > 0)
			{
				if (solution.ang_solution1 < solution.ang_solution2)
				{
					*pitch_ang = -solution.ang_solution1 + shooter_pitch_offset;
				}
				else
				{
					*pitch_ang = -solution.ang_solution2 + shooter_pitch_offset;
				}
			}
			else
			{
				*pitch_ang = atan2f(est_z, distance_xy);
			}
		}
		vision_ctrl_data.suggest_fire=true;
		return VISION_OK;
#else
    return VISION_NOTARGET;
#endif
  }
}
float shoot_delta_yaw_angle = 0;
float shoot_delta_yaw_scale = 0;
float shoot_delta_pitch_angle = 0;
float shoot_delta_pitch_scale = 0;
float shootable_yaw_angle = 0.65;
float shootable_pitch_angle = 0.35;

float shootable_angle_p = 10;
float shootable_angle_n = -10;
bool is_enter_shootable_angle;

// float shootable_angle=5;
bool force_enable_shoot = false;

float outpost_ref_v = 0;
float outpost_ref_v_thershold = 0.1;
float outpost_armor_v_error = 0;
float outpost_armor_v_error_thershold = 0.1;
float yaw_lllllll = 0;
float yaw_lll = 0;

void get_vision_suggest_fire(gimbal_state_t *gimbal_expt_state,
                             gimbal_state_t *gimbal_real_state) {

  float yaw_ang_ref = gimbal_expt_state->shooter_yaw;
  float yaw_ang_cur = gimbal_real_state->yaw;
  float pitch_ang_ref = gimbal_expt_state->pitch;
  float pitch_ang_cur = gimbal_real_state->pitch;

  shoot_delta_yaw_angle = fabs(get_delta_ang(yaw_ang_ref, yaw_ang_cur, 2 * PI));
  shoot_delta_yaw_scale = rad2scl(shoot_delta_yaw_angle);

  shoot_delta_pitch_angle =
      fabs(get_delta_ang(pitch_ang_ref, pitch_ang_cur, 2 * PI));
  shoot_delta_pitch_scale = rad2scl(shoot_delta_pitch_angle);

  is_enter_shootable_angle =
      (aim_spin_status == SPIN_FOLLOW &&
       ((0 < armor_theta_diff &&
         armor_theta_diff < deg2rad(shootable_angle_p)) ||
        (deg2rad(shootable_angle_n) < armor_theta_diff &&
         armor_theta_diff < 0))) ||
      (aim_spin_status == STOP);
  debug[3] = deg2rad(shootable_angle_p);
  debug[4] = armor_theta_diff;
  debug[5] = -deg2rad(shootable_angle_p);
  yaw_lllllll = shoot_delta_yaw_angle - deg2rad(shootable_yaw_angle);
  yaw_lll = shoot_delta_pitch_angle - deg2rad(shootable_pitch_angle);

  // float ang1 = vision_ctrl_data.yaw + predict_time *
  // vision_ctrl_data.v_yaw;
  if (shoot_delta_yaw_angle < deg2rad(shootable_yaw_angle) &&
      shoot_delta_pitch_angle <
          deg2rad(shootable_pitch_angle) && // 云台跟随情况
      is_enter_shootable_angle == true)
    //&& fabs(get_delta_ang(ang1, yaw_ang_cur, 2 * PI)) <
    // deg2rad(shoot_delta_angle)) //
    // 目标装甲板的角度(cj说此值有观测噪声,后续最后改一下)
    vision_ctrl_data.suggest_fire = true;
  else
    vision_ctrl_data.suggest_fire = false;

  if (force_enable_shoot == true)
    vision_ctrl_data.suggest_fire = true;
  // if (vision_ctrl_data.armor_num == 3 && vision_ctrl_data.id_num == 0) {
  //   float vx = vision_ctrl_data.vx;
  //   float vy = vision_ctrl_data.vy;
  //   outpost_ref_v = sqrt(vx * vx + vy * vy);
  //   outpost_armor_v_error =
  //       fabs(outpost_armor_v_filter.fltr_val -
  //       outpost_armor_v_filter.orig_val);
  //   if (outpost_ref_v > outpost_ref_v_thershold ||
  //       outpost_armor_v_error > outpost_armor_v_error_thershold)
  //     vision_ctrl_data.suggest_fire = false;
  // }
}
