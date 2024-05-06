#include "./algorithm/filter.h"
#include "./algorithm/pid.h"
#include "./algorithm/pid_leso.h"
#include "./base_drv/drv_conf.h"
#include "./base_drv/drv_motor/motor.h"
#include <string.h>

#include HAL_INCLUDE
#include gimbal_module_motor_ctrl_h
#include chassis_module_motor_ctrl_h
#include "./robot_core/ctrl_core/robot.h"

/**
 *	@brief	基于给定mid_scale校正rel_angle
 *	@note  	于can初始化后调用
 */
extern gimbal_motors_t gimbal_motors;
extern chassis_motors_t chassis_motors;
float temp_scale = 0;
void set_motor_mid_position(motor_t *p_motor, uint16_t mid_scale) {
  if (pm_real_scale >
      delta_scale / p_motor->motor_type.reduction_ratio + mid_scale) {
    temp_scale = -((delta_scale - pm_real_scale) + mid_scale);
  } else
    temp_scale = pm_real_scale - mid_scale;

  pm_real_rel_angle = 2 * PI * temp_scale / (delta_scale * pm_RD_ratio);
  pm_real_abs_angle = pm_real_rel_angle;
}

/**
 * @brief transfer raw data to pubilc struct
 * @param[in]  rx_buffer  raw data(8 bytes) received
 * @return HAL_OK if success otherwise HAL_ERROR
 */
/*处理电机数据函数，会在CAN接收中断的回调函数内被调用*/
static uint8_t gimbal_motor_offline_cnt[num_of_all_gimbal_motors] = {0};
static uint8_t chassis_motor_offline_cnt[num_of_all_chassis_motors] = {0};

HAL_StatusTypeDef gimbal_parse_motor_data(CAN_RxHeaderTypeDef *header,
                                          uint8_t *rx_buffer) {
  for (uint8_t i = 0; i < num_of_all_gimbal_motors; i++) {
    if (header->StdId == rx_stdid_of(all_gimbal_motors[i])) {
      motor_t *p_motor = 0;
      p_motor = &all_gimbal_motors[i];
      gimbal_motor_offline_cnt[i] = 0;
      pm_real_scale = ((rx_buffer[0] << 8) | rx_buffer[1]);
      pm_real_speed = (int16_t)(((rx_buffer[2] << 8) | rx_buffer[3]));
      pm_real_currs = ((rx_buffer[4] << 8) | rx_buffer[5]);
      pm_real_temp = rx_buffer[6];

      // 通过对相邻时刻的编码器值做差获得算上减速比的相对输出角度与绝对输出角度
      float diff_output_ang =
          get_delta_ang(pm_real_scale, pm_real_last_scale, delta_scale) * 2 *
          PI / (delta_scale * pm_RD_ratio);
      pm_real_last_scale = pm_real_scale;
      pm_real_abs_angle += diff_output_ang;
      pm_real_rel_angle += diff_output_ang;
      pm_real_rel_angle = range_map(pm_real_rel_angle, 0, 2 * PI);
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef chassis_parse_motor_data(CAN_RxHeaderTypeDef *header,
                                           uint8_t *rx_buffer) {

  for (uint8_t i = 0; i < num_of_all_chassis_motors; i++) {
    if (header->StdId == rx_stdid_of(all_chassis_motors[i])) {
      motor_t *p_motor = 0;
      p_motor = &all_chassis_motors[i];
      chassis_motor_offline_cnt[i] = 0;
      pm_real_scale = ((rx_buffer[0] << 8) | rx_buffer[1]);
      pm_real_speed = (int16_t)(((rx_buffer[2] << 8) | rx_buffer[3]));
      pm_real_currs = ((rx_buffer[4] << 8) | rx_buffer[5]);
      pm_real_temp = rx_buffer[6];

      // 通过对相邻时刻的编码器值做差获得算上减速比的相对输出角度与绝对输出角度
      float diff_output_ang =
          get_delta_ang(pm_real_scale, pm_real_last_scale, delta_scale) * 2 *
          PI / (delta_scale * pm_RD_ratio);
      pm_real_last_scale = pm_real_scale;
      pm_real_abs_angle += diff_output_ang;
      pm_real_rel_angle += diff_output_ang;
      pm_real_rel_angle = range_map(pm_real_rel_angle, 0, 2 * PI);
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

/**
 * @brief set all motor output
 * @return HAL_OK if success otherwise HAL_ERROR
 */
motor_transmit_msg_t _0x1ffbus = {0};
motor_transmit_msg_t _0x200bus = {0};
motor_transmit_msg_t _0x2ffbus = {0};
HAL_StatusTypeDef set_all_gimbal_motor_output(void) {
  uint8_t flag = 0;
  for (uint8_t i = 0; i < num_of_all_gimbal_motors; i++) {
    motor_transmit_msg_t *dir_mailbus;
    switch (all_gimbal_motors[i].motor_type.tx_base_ID) {
    case 0x1ff:
      dir_mailbus = &_0x1ffbus;
      flag |= 1;
      break;
    case 0x200:
      dir_mailbus = &_0x200bus;
      flag |= 2;
      break;
    case 0x2ff:
      dir_mailbus = &_0x2ffbus;
      flag |= 4;
      break;
    default:
      break;
    }
    uint16_t ID = all_gimbal_motors[i].motor_type.offset_ID - 1;
    if (ID > 3)
      dir_mailbus->D[ID - 3 - 1] = all_gimbal_motors[i].output;
    else
      dir_mailbus->D[ID] = all_gimbal_motors[i].output;
  }
  HAL_StatusTypeDef return_flag = HAL_OK;
#if 1
  if ((flag & 1) >> 0 == 1)
    return_flag |= set_motor_output(&GIMBAL_MOTOR_CAN, &_0x1ffbus, 0x1ff);
  if ((flag & 2) >> 1 == 1)
    return_flag |= set_motor_output(&GIMBAL_MOTOR_CAN, &_0x200bus, 0x200);
  if ((flag & 4) >> 2 == 1)
    return_flag |= set_motor_output(&GIMBAL_MOTOR_CAN, &_0x2ffbus, 0x2ff);
#endif
  return return_flag;
}

HAL_StatusTypeDef set_all_chassis_motor_output(void) {
  uint8_t flag = 0;
  for (uint8_t i = 0; i < num_of_all_chassis_motors; i++) {
    motor_transmit_msg_t *dir_mailbus;
    switch (all_chassis_motors[i].motor_type.tx_base_ID) {
    case 0x1ff:
      dir_mailbus = &_0x1ffbus;
      flag |= 1;
      break;
    case 0x200:
      dir_mailbus = &_0x200bus;
      flag |= 2;
      break;
    case 0x2ff:
      dir_mailbus = &_0x2ffbus;
      flag |= 4;
      break;
    default:
      break;
    }
    uint16_t ID = all_chassis_motors[i].motor_type.offset_ID - 1;
    if (ID > 3)
      dir_mailbus->D[ID - 3 - 1] = all_chassis_motors[i].output;
    else
      dir_mailbus->D[ID] = all_chassis_motors[i].output;
  }
  HAL_StatusTypeDef return_flag = HAL_OK;
#if 1
  if ((flag & 1) >> 0 == 1)
    return_flag |= set_motor_output(&CHASSIS_MOTOR_CAN, &_0x1ffbus, 0x1ff);
  if ((flag & 2) >> 1 == 1)
    return_flag |= set_motor_output(&CHASSIS_MOTOR_CAN, &_0x200bus, 0x200);
  if ((flag & 4) >> 2 == 1)
    return_flag |= set_motor_output(&CHASSIS_MOTOR_CAN, &_0x2ffbus, 0x2ff);
#endif
  return return_flag;
}
/**
 * @brief set the motor output
 * @param[in]  phcan    the CAN handler to transmit raw data
 * @param[in]  tx_msg   the pointer to voltage or current data struct (4 motor
 * data per struct)
 * @param[in]  id      choose from
 * C620_ID_BASE,C620_ID_EXTEND,M6020_ID_BASE,M6020_ID_EXTEND
 * @return HAL_OK if success otherwise HAL_ERROR
 */
HAL_StatusTypeDef set_motor_output(CAN_HandleTypeDef *phcan,
                                   motor_transmit_msg_t *tx_msg, uint32_t id) {
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_buffer[CAN_DATA_LEN];
  uint32_t can_mailbox;

  //	uint32_t* ptx_msg = (uint32_t*)tx_msg;
  //	//此时直接调用ARM指令调转大小端
  //	*(uint32_t*)(&tx_buffer[0])=__REV16(ptx_msg[0]);
  //	*(uint32_t*)(&tx_buffer[4])=__REV16(ptx_msg[1]);

  uint16_t *ptx_msg = (uint16_t *)tx_msg;
  // 普通方法
  for (uint8_t i = 0; i < 4; i++) {
    tx_buffer[2 * i + 1] = (uint8_t)(ptx_msg[i]);
    tx_buffer[2 * i] = (uint8_t)(ptx_msg[i] >> 8);
  }

  tx_header.DLC = CAN_DATA_LEN;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.StdId = id;
  tx_header.TransmitGlobalTime = DISABLE;

#if CAN_TX_TIMEOUT != 0
  uint32_t tickstart = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(phcan, CAN_TX_MAILBOX0)) {
    if (HAL_GetTick() - tickstart > CAN_TX_TIMEOUT) {
      HAL_CAN_AbortTxRequest(phcan, CAN_TX_MAILBOX0);
      break;
    }
  }
#endif

  // 异步发送
  return HAL_CAN_AddTxMessage(phcan, &tx_header, tx_buffer, &can_mailbox);
}

/**
 * @brief shut the motor output to 0
 * @param[in]  hcan  the CAN handler to transmit raw data
 * @param[in]  id    choose from
 * C620_ID_BASE,C620_ID_EXTEND,M6020_ID_BASE,M6020_ID_EXTEND
 * @return HAL_OK if success otherwise HAL_ERROR
 */

HAL_StatusTypeDef shut_motors_output(CAN_HandleTypeDef *hcan, uint32_t id) {
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_buffer[CAN_DATA_LEN];
  uint32_t can_mailbox;

  memset(tx_buffer, 0, CAN_DATA_LEN);

  tx_header.DLC = CAN_DATA_LEN;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.StdId = id;
  tx_header.TransmitGlobalTime = DISABLE;

#if CAN_TX_TIMEOUT != 0
  uint32_t tickstart = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0)) {
    if (HAL_GetTick() - tickstart > CAN_TX_TIMEOUT) {
      HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
      break;
    }
  }
#endif

  return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer, &can_mailbox);
}

void update_gimbal_motor_status(void) {
  uint8_t num_of_online_motor = 0;
  robot.gimbal_motor_status = 0;

  for (uint8_t i = 0; i < num_of_all_gimbal_motors; i++) {
    if (gimbal_motor_offline_cnt[i] < 20)
      gimbal_motor_offline_cnt[i]++;

    if (gimbal_motor_offline_cnt[i] >= 19) {
      all_gimbal_motors[i].is_offline = true;
      robot.gimbal_motor_status |= (uint32_t)(1 << i);
    } else {
      all_gimbal_motors[i].is_offline = false;
      num_of_online_motor++;
    }
  }
  if (num_of_online_motor == num_of_all_gimbal_motors)
    robot.gimbal_motor_status = 0;
}

void update_chassis_motor_status(void) {
  uint8_t num_of_online_motor = 0;
  robot.chassis_motor_status = 0;

  for (uint8_t i = 0; i < num_of_all_chassis_motors; i++) {
    if (chassis_motor_offline_cnt[i] < 20)
      chassis_motor_offline_cnt[i]++;

    if (chassis_motor_offline_cnt[i] >= 19) {
      all_chassis_motors[i].is_offline = true;
      robot.chassis_motor_status |= (uint32_t)(1 << i);
    } else {
      all_chassis_motors[i].is_offline = false;
      num_of_online_motor++;
    }
  }
  if (num_of_online_motor == num_of_all_chassis_motors)
    robot.chassis_motor_status = 0;
}

float get_motor_current(motor_t *p_motor) {
  float K = 0.0;
#define rxID p_motor->motor_type.rx_base_ID
  if (rxID == DJI_motor_lib_M2006.L.rx_base_ID ||
      rxID == DJI_motor_lib_M2006.H.rx_base_ID) {
    // M2006电机,子琪说是16384对应10A
    K = 10.0 / 16384.0;
  } else if (rxID == DJI_motor_lib_M3508.L.rx_base_ID ||
             rxID == DJI_motor_lib_M3508.H.rx_base_ID) {
    // M3508电机,子琪说是16384对应20A
    K = 20.0 / 16384.0;
  } else if (rxID == DJI_motor_lib_GM6020.L.rx_base_ID ||
             rxID == DJI_motor_lib_GM6020.H.rx_base_ID) {
    // GM6020电机,此部分未知,需要实际测量一下
    K = 0;
  } else
    return 0;
  return p_motor->real.current * K;
}
float get_motor_voltage(void) {
  // 可能要塞adc的代码
  return 0;
}
float get_motor_power(motor_t *p_motor) {
  return get_motor_current(p_motor) * get_motor_voltage();
}
float get_motor_torque(motor_t *p_motor) {
  float w = 2 * PI * p_motor->real.speed_rpm / 60.0f; // 弧度制
  return get_motor_power(p_motor) / w;
} /*  */