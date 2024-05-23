#ifndef _SUPER_CAP_H
#define _SUPER_CAP_H

#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "main.h"

#define CAP_TRANSMIT_ID 0x222
#define CAP_RESPONSE_ID 0x223

#define SUPER_CAP_RX_MAX_LOST 0xff

#define VCAP_MAX 25.f // 电容组最高放电电压
// 你改高也没用,这玩意功率板里写死的
#define VCAP_MIN 5.f // 电容组最低放电电压
// 注意功率板的变压效率会随着电容电压与电池电压的压差绝对值增大而增大
// 且为了保护功率板避免其在过于极端恶劣的环境下工作,推荐低于5V后就不要再放电了
#define CAP_CURR_MAX 15.f // 电容组最高放电电流
// 电容组输出接着裁判系统,那玩意限流15A
// 你改高也没用,这玩意功率板里写死的
typedef __packed struct {

  uint8_t set_power_in : 8;          // 主控设置的输入功率
  uint16_t input_current_m1000 : 13; // 功率板采集的母线输入电流
  int16_t charge_current_m1000 : 14; // 功率板采集的超电充电电流
  int16_t bus_voltage_s24m100 : 11;  // 功率板认为的母线电压
  uint16_t cap_voltage_m66 : 11;     // 功率板认为的电容电压
  uint8_t remain_energery_m10 : 5; // 功率板认为的超电剩余存储能量

  uint8_t SUPERCAP_ENABLE_FLAG : 1;
  uint8_t LOW_VOLTAGE_FLAG : 1;

} raw_cap_data_t;

/**
 * @struct cap_data_t
 * @brief the feedback data struct of super cap
 */
typedef struct {
  uint8_t SUPERCAP_ENABLE_FLAG;
  uint8_t LOW_VOLTAGE_FLAG;

  float input_current;  // (电池端)输入电流,单位:A
  float output_current; // (电机端)输出电流,单位:A
  float input_voltage;  // (电池端)输入电压,单位:V
  float cap_voltage;    // (电容端)输出电压,单位:V
  float power_set;      // 功率板内部功率限制设定值,单位:W
} cap_data_t;
typedef __packed struct {

  float power_set;         // 4bytes
  uint8_t ENABLE_SUPERCAP; // 1bytes
  uint32_t reverse : 24;

} cap_cmd_t;

HAL_StatusTypeDef parse_cap_data(CAN_RxHeaderTypeDef *rx_header,
                                 uint8_t *rx_buffer);

HAL_StatusTypeDef set_cap_power(CAN_HandleTypeDef *hcan, float set_power,
                                uint8_t ENABLE_SUPERCAP_FLAG);
cap_data_t *get_cap_data(void);

void inc_cap_rx_lost(void);

#endif
