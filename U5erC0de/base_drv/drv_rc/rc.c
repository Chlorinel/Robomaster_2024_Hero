#include "./base_drv/drv_rc/rc.h"
#include "./algorithm/crc.h"
#include "./base_drv/drv_uart.h"
#include "./drv_conf.h"
#include <string.h>

int32_t frame_offset = 0;
int32_t receive_size = 0;

// 缓冲区设为两倍帧长，实现双缓冲接收

#if USE_VT_RC_UART == 1
static uint8_t vt_rc_data_buffer[2 * VT_RC_FRAME_LEN];
int32_t vt_frame_offset = 0;
uint8_t vt_rc_rx_lost = VT_RC_RX_MAX_LOST;
extern UART_HandleTypeDef VT_RC_UART_HANDLE;
#endif

// RAM_PERSIST static rc_ctrl_t rc_ctrl_data;
rc_ctrl_t rc_ctrl_data;
uint8_t rc_rx_lost = RC_RX_LOST_MAX;

#if USE_RC_UART == 1

uint8_t rc_data_buffer[2 * RC_FRAME_LENGTH];
extern UART_HandleTypeDef RC_UART_HANDLE;

/**
 * @brief parse remote control data and store it
 * @param[in]  pData   pointer to data buffer
 * @return error code, err=0 no error, err!=0 with specific error
 * @attention  must ensure that the buffer len is larger than RC_FRAME_LENGTH
 */
extern robot_ctrl_t robot;
uint8_t parse_rc_data(uint8_t *pData) {
  rc_ctrl_data.rc.switch_left = (uint8_t)((uint8_t)(pData[5] >> 4) & 0x0C) >> 2;
  rc_ctrl_data.rc.switch_right = (uint8_t)((uint8_t)(pData[5] >> 4) & 0x03);
  if (robot.ctrl_mode == 0) {
    rc_ctrl_data.rc.ch0 =
        (int16_t)((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    if (rc_ctrl_data.rc.ch0 > RC_CH_VALUE_MAX ||
        rc_ctrl_data.rc.ch0 < RC_CH_VALUE_MIN)
      return RC_CH_ERROR;
    rc_ctrl_data.rc.ch0 -= RC_CH_VALUE_OFFSET;

    rc_ctrl_data.rc.ch1 =
        (int16_t)(((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    if (rc_ctrl_data.rc.ch1 > RC_CH_VALUE_MAX ||
        rc_ctrl_data.rc.ch1 < RC_CH_VALUE_MIN)
      return RC_CH_ERROR;
    rc_ctrl_data.rc.ch1 -= RC_CH_VALUE_OFFSET;

    rc_ctrl_data.rc.ch2 =
        (int16_t)(((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                  ((int16_t)pData[4] << 10)) &
        0x07FF;
    if (rc_ctrl_data.rc.ch2 > RC_CH_VALUE_MAX ||
        rc_ctrl_data.rc.ch2 < RC_CH_VALUE_MIN)
      return RC_CH_ERROR;
    rc_ctrl_data.rc.ch2 -= RC_CH_VALUE_OFFSET;

    rc_ctrl_data.rc.ch3 =
        (int16_t)(((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;
    if (rc_ctrl_data.rc.ch3 > RC_CH_VALUE_MAX ||
        rc_ctrl_data.rc.ch3 < RC_CH_VALUE_MIN)
      return RC_CH_ERROR;
    rc_ctrl_data.rc.ch3 -= RC_CH_VALUE_OFFSET;

    rc_ctrl_data.rc.ch4 =
        (int16_t)((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
    rc_ctrl_data.rc.ch4 -= RC_CH_VALUE_OFFSET;

    rc_ctrl_data.mouse.x =
        (int16_t)((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    rc_ctrl_data.mouse.y =
        (int16_t)((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    rc_ctrl_data.mouse.z =
        (int16_t)((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

    rc_ctrl_data.mouse.press_left = (uint8_t)pData[12];
    rc_ctrl_data.mouse.press_right = (uint8_t)pData[13];

    rc_ctrl_data.keyboard.keycode =
        (((uint16_t)pData[14]) | ((uint16_t)pData[15] << 8));
  }
  return RC_NO_ERROR;
}

/**
 * @brief      enable remote control uart it and initialize DMA
 * @return     set HAL_OK or HAL_BUSY
 */
HAL_StatusTypeDef rc_recv_dma_init(void) {
  rc_rx_lost = RC_RX_LOST_MAX;
  memset(&rc_ctrl_data, 0, sizeof(rc_ctrl_t));

  HAL_StatusTypeDef result = uart_recv_dma_init(
      &(RC_UART_HANDLE), rc_data_buffer, 2 * RC_FRAME_LENGTH);
  __HAL_UART_ENABLE_IT(&(RC_UART_HANDLE), UART_IT_IDLE);
  return result;
}
#endif

#if USE_VT_RC_UART == 1

uint8_t parse_vt_rc_data(uint8_t *data_ptr) {
  vt_rc_frame_t *frame = (vt_rc_frame_t *)data_ptr;
  if (frame->vt_rc_cmdid == VT_FRAME_CMDID &&
      Verify_CRC8_Check_Sum(data_ptr, sizeof(frame_header_t)) &&
      Verify_CRC16_Check_Sum(data_ptr, sizeof(vt_rc_frame_t))) {
    if (robot.ctrl_mode == 1) {
      rc_ctrl_data.mouse.x = frame->data.mouse_x;
      rc_ctrl_data.mouse.y = frame->data.mouse_y;
      rc_ctrl_data.mouse.z = frame->data.mouse_z;
      rc_ctrl_data.mouse.press_left = frame->data.left_button_down;
      rc_ctrl_data.mouse.press_right = frame->data.right_button_down;
      rc_ctrl_data.keyboard.keycode = frame->data.keyboard_value;
    }
    return RC_NO_ERROR;
  } else
    return RC_VERIFY_ERR;
}

/**
 * @brief      enable vt remote control uart it and initialize DMA
 * @return     set HAL_OK or HAL_BUSY
 */
HAL_StatusTypeDef vt_rc_recv_dma_init(void) {
  rc_rx_lost = RC_RX_LOST_MAX;
  memset(&rc_ctrl_data, 0, sizeof(rc_ctrl_t));

  HAL_StatusTypeDef result = uart_recv_dma_init(
      &(VT_RC_UART_HANDLE), vt_rc_data_buffer, 2 * VT_RC_FRAME_LEN);
  __HAL_UART_ENABLE_IT(&(VT_RC_UART_HANDLE), UART_IT_IDLE);
  return result;
}
#endif

#if USE_RC_UART == 1 || USE_VT_RC_UART == 1
/**
 * @brief uart idle interrupt routine implementation
 * @param[in]  huart   pointer to UART_HandleTypeDef
 * @return none
 */
void rc_uart_idle_handle(UART_HandleTypeDef *huart) {
#if USE_RC_UART == 1

  if (huart->Instance == RC_UART &&
      __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    DMA_Stream_TypeDef *uhdma = huart->hdmarx->Instance;
    // 获取DMA接收到的一帧的长度
    receive_size = (int32_t)(2 * RC_FRAME_LENGTH - uhdma->NDTR) - frame_offset;
    // 验证数据长度
    if (receive_size == RC_FRAME_LENGTH || receive_size == -RC_FRAME_LENGTH) {
      if (!parse_rc_data(&rc_data_buffer[frame_offset])) {
        // 数据正确
        rc_rx_lost = 0;
      } else
        rc_rx_lost = RC_RX_LOST_MAX;
      // 设置帧偏移，实现双缓冲
      // 当偏移在中间时，下一次应该在起始，若在起始则下一次在中间
      frame_offset = (int32_t)(frame_offset == 0) * RC_FRAME_LENGTH;
    } else if (receive_size != 0) {
      // 数据长度错误，需要重置DMA并且将偏移设回起始
      // some bytes lost, reset DMA buffer
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
      __HAL_DMA_DISABLE(huart->hdmarx);
      frame_offset = 0;
      uhdma->NDTR = (uint32_t)(RC_FRAME_LENGTH * 2);
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_HT_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_TE_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_ENABLE(huart->hdmarx);
      SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
  }

#endif

#if USE_VT_RC_UART == 1
  if (huart->Instance == VT_RC_UART &&
      __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    DMA_Stream_TypeDef *uhdma = huart->hdmarx->Instance;
    receive_size =
        (int32_t)(2 * VT_RC_FRAME_LEN - uhdma->NDTR) - vt_frame_offset;
    if (receive_size == VT_RC_FRAME_LEN || receive_size == -VT_RC_FRAME_LEN) {
      if (parse_vt_rc_data(&vt_rc_data_buffer[vt_frame_offset]) ==
          RC_NO_ERROR) {
        vt_rc_rx_lost = 0;
      } else
        rc_rx_lost = VT_RC_RX_MAX_LOST;
      vt_frame_offset = (int32_t)(vt_frame_offset == 0) * VT_RC_FRAME_LEN;
    } else if (receive_size != 0) {
      // some bytes lost, reset DMA buffer
      __HAL_DMA_ENABLE(huart->hdmarx);
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
      __HAL_DMA_DISABLE(huart->hdmarx);
      uhdma->NDTR = (uint32_t)(VT_RC_FRAME_LEN * 2);
      vt_frame_offset = 0;
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_HT_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_TE_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_ENABLE(huart->hdmarx);
      SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
  }
#endif
}

#endif

/**
 * @brief  get a copy of remote control data
 * @return a copy of data in rc_ctrl_t
 */
rc_ctrl_t get_rc_data(void) {
  rc_ctrl_t tmp;
// 关闭中断防止复制数据时发生中断使数据无效
#if USE_RC_UART == 1
  __HAL_UART_DISABLE_IT(&(RC_UART_HANDLE), UART_IT_IDLE);
#endif
#if USE_VT_RC_UART == 1
  __HAL_UART_DISABLE_IT(&(VT_RC_UART_HANDLE), UART_IT_IDLE);
#endif
  tmp = rc_ctrl_data;
#if USE_VT_RC_UART == 1
  __HAL_UART_ENABLE_IT(&(VT_RC_UART_HANDLE), UART_IT_IDLE);
#endif
#if USE_RC_UART == 1
  __HAL_UART_ENABLE_IT(&(RC_UART_HANDLE), UART_IT_IDLE);
#endif
  return tmp;
}

/**
 * @brief  update last state of keyboard and mouse
 * @return none
 */
/*更新过去遥控输入状态状态*/
void update_rc_last_key(void) {
  rc_ctrl_data.keyboard.last_keycode = rc_ctrl_data.keyboard.keycode;
  rc_ctrl_data.mouse.last_press_left = rc_ctrl_data.mouse.press_left;
  rc_ctrl_data.mouse.last_press_right = rc_ctrl_data.mouse.press_right;
  rc_ctrl_data.rc.last_switch_left = rc_ctrl_data.rc.switch_left;
  rc_ctrl_data.rc.last_switch_right = rc_ctrl_data.rc.switch_right;
}

/**
 * @brief  check if remote control is offline
 * @return 0 for remote control online, others for offline
 */
uint32_t is_rc_offline(void) {
  if (rc_rx_lost >= RC_RX_LOST_MAX &&
      rc_ctrl_data.rc.ch0 <= RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch0 >= -RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch1 <= RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch1 >= -RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch2 <= RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch2 >= -RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch3 <= RC_CH_VALUE_RANGE &&
      rc_ctrl_data.rc.ch3 >= -RC_CH_VALUE_RANGE) {
    rc_ctrl_data.rc.switch_left = 0x00;
    rc_ctrl_data.rc.switch_right = 0x00;
    return true;
  } else {
    return 0;
  }
}

#if USE_VT_RC_UART == 1
/**
 * @brief  check if vt remote control is offline
 * @return 0 for remote control online, others for offline
 */
uint32_t is_vt_rc_offline(void) { return vt_rc_rx_lost >= VT_RC_RX_MAX_LOST; }

#endif

/**
 * @brief  increment of rc_rx_lost counter, should be call after process of all
 * rc data
 * @return none
 */
void inc_rc_rx_lost(void) {
  if (rc_rx_lost < RC_RX_LOST_MAX)
    rc_rx_lost++;
#if USE_VT_RC_UART == 1
  if (vt_rc_rx_lost < VT_RC_RX_MAX_LOST)
    vt_rc_rx_lost++;
#endif
}
