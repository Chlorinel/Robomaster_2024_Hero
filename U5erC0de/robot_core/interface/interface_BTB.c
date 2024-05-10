#include "./robot_core/interface/interface_BTB.h"

#include "./base_drv/drv_can.h"
#include "./robot_core/ctrl_core/robot.h"
#include "can.h"
#include "string.h"
#include "tim.h"
#include <stdlib.h>

// 指针数组,存放着指向要从底盘发向云台的msg数据的指针

// 指针数组,存放着指向要从云台发向底盘的msg数据的地址

// 上边两个会被用在transmit_BTB_msg(msg发送函数)与parse_BTB_data(接收函数)中
// 多与#ifdef配合以选择在不同板子上该调用哪个数据
// 同时做为数组便于遍历,逝分的好用

/**
 * @brief   板间交互数据初始化函数
 * @note    新增加通讯消息时的步骤:
 *              1、添加到xxx_info
 *              2、改num_of_xxx_msg
 *              3、在BTB_init中排好指针、方向、长度与优先级
 *              4、配好CAN通讯滤波器
 *              5、ctrl+c到对方的代码
 **/

HAL_StatusTypeDef BTB_init(void) {

  // 获取各方向内的总包权重(优先级乘包长)值(get_random有用)
  extern uint16_t sum_weight[2];

// 测试收/发两通道的包权重分配
#if is_test_divide_enable
  // 算出各方向分别的包权重百分比
  for (uint8_t i = 0; i < num_of_chassis_to_gimbal_msg; i++)
    test_chassis_to_gimbal.k_of_num_expt[i] =
        (float)(p_chassis_to_gimbal_msg[i]->priority *
                p_chassis_to_gimbal_msg[i]->len) /
        sum_weight[chassis_to_gimbal];
  for (uint8_t i = 0; i < num_of_gimbal_to_chassis_msg; i++)
    test_gimbal_to_chassis.k_of_num_expt[i] =
        (float)(p_gimbal_to_chassis_msg[i]->priority *
                p_gimbal_to_chassis_msg[i]->len) /
        sum_weight[gimbal_to_chassis];
// 麻了
#endif

  // 依据包权重分配stdid

  // 赋值指针,以便于get_random与update使用
//  uint8_t temp[2] = {0}; // 记载偏移量

  // 开启CAN
  HAL_StatusTypeDef Flag = HAL_OK;
#ifdef THIS_IS_GIMBAL

#endif
#ifdef THIS_IS_CHASSIS
  uint32_t Fliter_List1[4] = {
      p_gimbal_to_chassis_msg[0]->stdid,
      p_gimbal_to_chassis_msg[1]->stdid,
      p_gimbal_to_chassis_msg[2]->stdid,
      p_gimbal_to_chassis_msg[3]->stdid,
  };
  Flag |= can_user_init(&BTB_CAN, Fliter_List1, BTB_FIFO, BTB_FLTRNUM);
  uint32_t Fliter_List2[4] = {
      p_gimbal_to_chassis_msg[4]->stdid,
      p_gimbal_to_chassis_msg[5]->stdid,
      p_gimbal_to_chassis_msg[6]->stdid,
  };
  Flag |= can_user_init(&BTB_CAN, Fliter_List2, BTB_FIFO, BTB_FLTRNUM + 4);
#endif

  // 开启中断,在运行过程中此定时器会被动态调频,保证板间通信数据能稳定传输

  return Flag;
};
/**
 * @brief   板间交互的发送函数
 */

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

/**
 * @brief   板间交互的接收函数
 */
HAL_StatusTypeDef parse_BTB_data(CAN_RxHeaderTypeDef *rx_header,
                                 uint8_t *rx_buffer) {

  return HAL_OK;
}

/**
 * @brief   更新各接收数据状态,配合is_the_receiver_offline以确定是否接收端掉线
 * @note    此函数于tim14的10hz中断中运行
 */
bool _is_the_receiver_offline = true;
void BTB_update_State(void) {}
/**
 * @brief   检测是否所有数据包均丢失以检测接收端是不是已经掉线
 * @return  true:对方掉线
 *          false:对方在线
 */
#define BTB_RX_MAX_LOST 10
bool is_the_receiver_offline(void) { return false; }
/**
 * @brief   检测接收端是不是刚刚上线
 * @return  true:刚刚上线
 *          false:还没上线 或 一直都在线
 */
bool _is_the_receiver_power_on_right_now = false;
bool is_the_receiver_power_on_right_now(void) {
  _is_the_receiver_power_on_right_now = (_is_the_receiver_offline == true) &&
                                        (is_the_receiver_offline() == false);
  return _is_the_receiver_power_on_right_now;
}
