#include "./base_drv/drv_can.h"
#include "can.h"

#include gimbal_module_core_h
#include chassis_module_core_h
#include "./base_drv/drv_motor/motor_ctrl.h"
// #include "./robot_core/interface/interface_BTB.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

HAL_StatusTypeDef flag_can = HAL_OK; // HAL_OK=0
/**
* @brief  	init can filter, start can, enable can rx interrupt
* @param[in] 	hcan: 	pointer to a CAN_HandleTypeDef structure that contains
                                        the configuration information for the
specified CAN.
* 	@param[in] FlierId: receive Id
* 	@param[in] FIFO: 	can FIFO channel choose( CAN_FILTER_FIFO0 or
CAN_FILTER_FIFO1 )
* @return HAL_OK if success otherwise HAL_ERROR
*/
HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef *hcan, uint32_t *FliterId,
                                _Bool FIFO, uint8_t FilterNum) {
  CAN_FilterTypeDef can_filter;
  can_filter.FilterActivation = ENABLE;

#if 1
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = FliterId[0] << 5;
  can_filter.FilterIdLow = FliterId[1] << 5;
  can_filter.FilterMaskIdHigh = FliterId[2] << 5;
  can_filter.FilterMaskIdLow = FliterId[3] << 5;
#else
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = 0x00;
  can_filter.FilterIdLow = 0x00;
  can_filter.FilterMaskIdHigh = 0x00;
  can_filter.FilterMaskIdLow = 0x00;
#endif

  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterFIFOAssignment = FIFO;
  can_filter.FilterActivation = ENABLE; // 激活滤波器0

  if (hcan->Instance == CAN1) {
    can_filter.FilterBank = 0 + FilterNum;
  } else {
    can_filter.FilterBank = 14 + FilterNum;
  }
  // can的FilterBank参数是用来配置过滤器组的(标准库叫做FilterNumber)
  // 双CAN系列的产品有27个滤波器,其中can1的滤波器号为0~13,can2为14~27
  // 为了一路can配置8个id(list模式)(FIFO0与FIFO1各自4个),就不应该反复塞到0 or
  // 14滤波器里(应该吧?) 这里就采用了起始滤波器+FilterNum选定通道

  flag_can |= HAL_CAN_ConfigFilter(hcan, &can_filter);

  flag_can |= HAL_CAN_Start(hcan);

  if (FIFO == CAN_FILTER_FIFO0)
    flag_can |= HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  else
    flag_can |= HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  return flag_can;
}

#include "./base_drv/drv_cap/super_cap.h"

/**
 * @brief 	can FIFO0的接收中断回调函数
 * @note	请务必按照drv_conf.h里写的那样给选择好各parse_xxx_data的位置
 * */
uint16_t Label_FIFO0;
uint8_t rx_buffer_FIFIO0_temp[8] = {0};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_hander;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hander, rx_buffer_FIFIO0_temp);
  Label_FIFO0 = rx_hander.StdId;

  if (hcan == &hcan1) {
    if (rx_hander.StdId == CAP_RESPONSE_ID) {
      parse_cap_data(&rx_hander, rx_buffer_FIFIO0_temp);
    } else
      chassis_parse_motor_data(&rx_hander, rx_buffer_FIFIO0_temp);
  }

  if (hcan == &hcan2) {
    gimbal_parse_motor_data(&rx_hander, rx_buffer_FIFIO0_temp);
  }
}

/**
 * @brief 	can FIFO1的接收中断回调函数
 * @note	请务必按照drv_conf.h里写的那样给选择好各parse_xxx_data的位置
 * */
uint16_t Label_FIFO1;
uint8_t rx_buffer_FIFIO1_temp[8] = {0};
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_hander;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_hander, rx_buffer_FIFIO1_temp);
  Label_FIFO1 = rx_hander.StdId;

  if (hcan == &hcan1) {
    chassis_parse_motor_data(&rx_hander, rx_buffer_FIFIO1_temp);
  }

  if (hcan == &hcan2) {
    gimbal_parse_motor_data(&rx_hander, rx_buffer_FIFIO1_temp);
  }
}

/*
hcan1
normal:
        txid	rxid
LF 			0x201
LR			0x203
RF			0x202
RR			0x204

gimbal		0x209
(gimbal)

extend:
cap			0x211

hcan2
normal:
        txid	rxid
fl			0x201
fr			0x202
fc			0x205
pr			0x207

extend:
        0x1a3 0x1a7
//子骐chassis代码中,这个ID是用来与gimbal通信用的
//解析函数位于gimbal_com.c里
#define GimbalMsgRX_ID_OFFSET 0x1A0U
-> 3 , 7
*/
