#include "super_cap.h"
#include "./robot_core/tool/bus_detect.h"
#include "base_drv/drv_can.h"
#include "string.h"

// setup super cap default value
cap_data_t cap_data = {.input_voltage = 24.0f,
                       .cap_voltage = 21.9f,
                       .power_set = 50,
                       .SUPERCAP_ENABLE_FLAG = 0};

static uint16_t cap_rx_lost = SUPER_CAP_RX_MAX_LOST;

/**
 * @brief parse can data from super cap board and store it
 * @param[in]  rx_header  the CAN header
 * @param[in]  rx_buffer  raw data(8 bytes) received
 * @return HAL_OK if the id is match otherwise HAL_ERROR
 */
float cap_data_rfreq = 0;
raw_cap_data_t raw_cap_data;
								
HAL_StatusTypeDef parse_cap_data(CAN_RxHeaderTypeDef *rx_header,
                                 uint8_t *rx_buffer) {
  memcpy(&raw_cap_data, rx_buffer, sizeof(raw_cap_data_t));
  if (rx_header->StdId == CAP_RESPONSE_ID) {
    get_data_refresh_freq(cap_data.input_voltage, cap_data_rfreq);

    cap_data.input_voltage =
        24.f + (int16_t)raw_cap_data.bus_voltage_s24m100 / 100.0f;
    cap_data.cap_voltage = (uint16_t)raw_cap_data.cap_voltage_m66 / 66.0f;
    cap_data.input_current =
        (uint16_t)raw_cap_data.input_current_m1000 / 1000.0f;
    cap_data.output_current =
        cap_data.input_current -
        (int16_t)raw_cap_data.charge_current_m1000 / 1000.0f;
    cap_data.power_set = (uint8_t)raw_cap_data.set_power_in;

    cap_data.SUPERCAP_ENABLE_FLAG = raw_cap_data.SUPERCAP_ENABLE_FLAG;
    cap_data.LOW_VOLTAGE_FLAG = raw_cap_data.LOW_VOLTAGE_FLAG;

    cap_rx_lost = 0;
    return HAL_OK;
  } else
    return HAL_ERROR;
}

/**
 * @brief  get a copy of super cap response data
 * @return a copy of data in cap_data_t
 */
cap_data_t *get_cap_data(void) {
  //__HAL_CAN_DISABLE_IT(&SUPER_CAP_CAN_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING);
  //__HAL_CAN_ENABLE_IT(&SUPER_CAP_CAN_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING);
  return &cap_data;
}

/**
 * @brief set the super cap power limit
 * @param[in]  hcan  the CAN handler to transmit raw data
 * @param[in]  set_power  the expected power limit
 * @return HAL_OK if transmission was success otherwise HAL_ERROR
 */
HAL_StatusTypeDef ret = HAL_OK;
uint32_t super_can_sent_freq = 1000;

cap_cmd_t cap_cmd = {0};
HAL_StatusTypeDef set_cap_power(CAN_HandleTypeDef *hcan, float set_power,
                                uint8_t ENABLE_SUPERCAP_FLAG) {
  static uint32_t cap_tickstart = 0;
  if (HAL_GetTick() - cap_tickstart >= 1000.f / super_can_sent_freq) {

    cap_cmd.power_set = set_power;
    cap_cmd.ENABLE_SUPERCAP = ENABLE_SUPERCAP_FLAG;
    uint8_t tx_buffer[CAN_DATA_LEN];
    memcpy(tx_buffer, &cap_cmd, CAN_DATA_LEN);

    CAN_TxHeaderTypeDef tx_header;
    uint32_t can_mailbox;

    tx_header.DLC = CAN_DATA_LEN;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.StdId = CAP_TRANSMIT_ID;
    tx_header.TransmitGlobalTime = DISABLE;

    ret = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer, &can_mailbox);
    cap_tickstart = HAL_GetTick();
  }
  return ret;
}

void inc_cap_rx_lost(void) {
  if (cap_rx_lost < SUPER_CAP_RX_MAX_LOST)
    cap_rx_lost++;
  else
    cap_data.SUPERCAP_ENABLE_FLAG = 0;
}
