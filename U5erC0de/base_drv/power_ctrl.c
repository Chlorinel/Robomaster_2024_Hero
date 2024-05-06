#include "./base_drv/power_ctrl.h"

#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/drv_conf.h"
#include "./base_drv/drv_referee/referee.h"
#include chassis_module_core_h

#include "./algorithm/filter.h"
#include "./algorithm/util.h"
#include <math.h>
#include <string.h>

#define P_idle (3.4418f) / 4
#define Kn (float)(0.013201)
float Kn_inv = Kn;
#define MLC (float)(2.2543e-05) // Mechanical loss coefficient
#define ESR (float)(0.29283)    // equivalent series resistance
/**
 * @brief 依据M3508的功率模型,计算出理论上设定功率对应的设定的电流
 *
 * @param P 设定功率
 * @param omega 电机输出端的角速度(弧度制,rad/s)
 * @param dirc 期望的旋转方向(对应期望转矩电流方向)
 * @return float 为计算得到的设定电流
 */
float get_M3508_Iq(float P, float omega, int8_t dirc) {
  float omegaS2 = omega * omega;
  float RS2 = 0, Iq1 = 0, Iq2 = 0;
  RS2 = Kn * Kn * omegaS2 - 4 * ESR * MLC * omegaS2 + 4 * ESR * P -
        4 * ESR * P_idle;
  if (RS2 < 0) {
    RS2 = 0;
  }
  Iq1 = (-(Kn * omega - sqrtf(RS2)) / (2 * ESR));
  //-(-sqrt(rs2)),为正的可能大
  Iq2 = (-(Kn * omega + sqrtf(RS2)) / (2 * ESR));
  //+(-sqrt(rs2)),为负的可能大
  if (dirc >= 0)
    return Iq1;
  else
    return Iq2;

  // float Iq1 = 0;
  // if (omega == 0)
  //   Iq1 = dirc * MAX_CURRENT_LIMIT;
  // else
  //   Iq1 = P / (Kn_inv * omega);
  // return Iq1;
}
/**
 * @brief Get the M3508 power object
 *
 * @param omega 输出端角速度(弧度制,rad/s)
 * @param Iq 输出端转矩电流(公制,A)
 * @return float
 */
float get_M3508_power(float omega, float Iq) {
  return (Kn * omega * Iq + MLC * omega * omega + ESR * Iq * Iq + P_idle);
  // return (Kn * omega * Iq + MLC * omega * omega);
};

float soft_HP_MAX = 200;
float soft_HP = 200;
/**
 * @brief 软件热量限制
 * @param pow_lim
 */

void update_soft_chassis_energy_buffer(chassis_power_lim_t *pow_lim) {
  cap_data_t *cap_data = get_cap_data();
  float referee_power_buffer = pow_lim->referee_power_buffer;
  float power_in = cap_data->input_voltage * cap_data->input_current;
  float power_limit = pow_lim->power_limit;

  static uint32_t tickstart = 0;
  uint32_t delta_tick = CLAMP_MAX(HAL_GetTick() - tickstart, 100);
  tickstart = HAL_GetTick();
  // 不会真的有人会用周期大于0.1s的频率控制底盘吧
  float t = delta_tick / 1000.f;
  float E_charge = (power_limit - power_in) * t;
  referee_power_buffer += E_charge;

  if (referee_power_buffer < 0.f) { // 超功率力!
    float K = (power_limit - power_in) / power_limit;
    float N = (K <= 0.1f) ? 0.1f : //
                  ((K <= 0.2f) ? 0.2f : 0.4f);
    soft_HP -= soft_HP_MAX * N * t;
  }
  pow_lim->power_limit = power_limit;
  pow_lim->referee_power_buffer = referee_power_buffer;
  LIMIT_MIN_MAX(pow_lim->referee_power_buffer, 0, 60.f);
}
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float power_limit_ratio = 1.f;
motor_power_parameter_calibrate_bag_t MPPCB = {0};
pid_struct_t referee_buffer_limit_pid = {.kp = 0.1,
                                         .out_max = 1}; // 0.1就对应正负10J
pid_struct_t power_limit_pid = {.kp = 500, .out_max = 300}; // 单位:W
LPF_t set_charge_power_filter = {.ts = 0.001f, .fc = 10.f};
float k_feedforward = 0.01;
float set_charge_power = 0;
/**
 * @brief 底盘功率分配
 * @param pow_lim
 */
void chassis_power_distributor(chassis_power_lim_t *pow_lim,
                               uint8_t is_cap_enable) {
  cap_data_t *cap_data = get_cap_data();
  static uint32_t tickstart = 0;
  uint32_t delta_tick = CLAMP_MAX(HAL_GetTick() - tickstart, 100);
  // 不会真的有人会用周期大于0.1s的频率控制底盘吧
  tickstart = HAL_GetTick();
	/// debug
	pow_lim->chassis_real_total_power=cap_data->input_voltage*cap_data->output_current;
	
  ///>>> 确定可调配的功率值
  float k_referee_buffer = // 通过改变闭环期望电容组剩余能量进而实现稳定缓冲能量
      pid_calc(&referee_buffer_limit_pid,          //
               pow_lim->expt_referee_power_buffer, //
               pow_lim->referee_power_buffer);
  LIMIT_MIN_MAX(k_referee_buffer, -1.f, 1.f);
  // 缓冲能量过低->k_referee_buffer向着1上升->缩小期望与实际电容剩余能量差->减低输出功率
  // 缓冲能量过高->k_referee_buffer向着-1下降->提高期望与实际电容剩余能量差->减低输出功率

  float k = _pow2(VCAP_MAX) - _pow2(VCAP_MIN);
  float real_cap_energy_ratio = // 实际电容剩余能量百分比
      (_pow2(cap_data->cap_voltage) - _pow2(VCAP_MIN)) / k;
	pow_lim->real_cap_ratio=real_cap_energy_ratio;
  float ratio_error = real_cap_energy_ratio - pow_lim->expt_cap_energy_ratio;
  float expt_cap_energy_ratio =
      pow_lim->expt_cap_energy_ratio + k_referee_buffer * ratio_error;
  LIMIT_MIN_MAX(expt_cap_energy_ratio, 0.f, 1.f);
  
  pow_lim->set_total_power = //
      pow_lim->power_limit - //
      pid_calc(&power_limit_pid, expt_cap_energy_ratio, real_cap_energy_ratio);
  LIMIT_MIN_MAX(pow_lim->set_total_power, pow_lim->power_limit,
                cap_data->cap_voltage * CAP_CURR_MAX + pow_lim->power_limit);
  // 电容组电压过低时,就应该自行降低输出功率以避免输入超功率
  /**参数设定+debug**/
  ///>>> null

#if ENABLE_MOTOR_POWER_PARAM_CALIBR_FLAG
  /**参数标定模式**/
  motor_response_msg_t *all_motor_data = get_motor_data_ptr();
  float real_power = cap_data->input_voltage * cap_data->input_current;
  float fitted_power = get_chasiss_motors_power( //
      pow_lim->real_omega[i], pow_lim->real_Iq[i]);
  MPPCB = (motor_power_parameter_calibrate_bag_t){
      .real_power = real_power,                  //
      .set_ctrl_value = pow_lim->raw_expt_Iq[0], //
      .real_ctrl_value = pow_lim->real_Iq[0],    //
      .real_omega = pow_lim->real_omega[0],      //
      .fitted_power = fitted_power,
  };
  pow_lim->set_Iq[0] = pow_lim->raw_expt_Iq[0];
  pow_lim->set_Iq[1] = 0;
  return;
#else
  /**软件开环限制输出原始功率**/
  ///>>> 获取参数
  float *raw_expt_Iq = pow_lim->raw_expt_Iq;
  float *real_Iq = pow_lim->real_Iq;
  float *real_omega = pow_lim->real_omega;
  float *expt_omega = pow_lim->expt_omega;

  ///>>> 根据电机功率模型求解原始设定值所需要的功率
  float total_power = 0;
  for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    pow_lim->raw_power[i] = get_M3508_power(expt_omega[i], raw_expt_Iq[i]);
    total_power += pow_lim->raw_power[i];
  }
  /**按功率占比分配**/
#if ENABLE_POWER_LIMIT_FLAG == 1
  if (total_power > pow_lim->set_total_power) {
    power_limit_ratio = pow_lim->set_total_power / total_power;
  } else {
    power_limit_ratio = 1.0f;
  }
#endif
  /**根据功率逆向解算,得到在当前角速度下,为达到设定功率所需的设定电流**/
  float set_total_power = 0;
  for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    pow_lim->set_power[i] = pow_lim->raw_power[i] * power_limit_ratio;
    set_total_power += pow_lim->set_power[i];
    int dirc = sign(raw_expt_Iq[i]);
    pow_lim->fitted_power[i] = get_M3508_power(real_omega[i], real_Iq[i]);
    float set_Iq = get_M3508_Iq( //
        pow_lim->set_power[i], real_omega[i], dirc);
    if (dirc >= 0) {
      LIMIT_MIN_MAX(set_Iq, 0, raw_expt_Iq[i]);
    } else {
      LIMIT_MIN_MAX(set_Iq, raw_expt_Iq[i], 0);
    }
    pow_lim->set_Iq[i] = set_Iq;
  }
#endif
  /**设置超电最大输入功率**/
  static float last_set_total_power = 0;
  float d_set_total_power =
      (set_total_power - last_set_total_power) * 1000.f / delta_tick;
  if (d_set_total_power < 0)
    d_set_total_power = 0;
  last_set_total_power = set_total_power;

  set_charge_power = pow_lim->power_limit - k_feedforward * d_set_total_power;
  set_charge_power_filter.ts = delta_tick / 1000.f;
  set_charge_power = LPF_update(&set_charge_power_filter, set_charge_power);
  LIMIT_MIN_MAX(set_charge_power, 0.f, 120.f);
  set_cap_power(&CAP_CAN, set_charge_power, is_cap_enable);
}
