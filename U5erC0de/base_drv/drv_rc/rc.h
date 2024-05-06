#ifndef _REMOTE_CTRL_H
#define _REMOTE_CTRL_H

#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE

/* ----------------------Internal Data------------------------------*/

/* --------------- RC Channel Definition-----------------*/
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH_VALUE_RANGE ((uint16_t)660)
/* ----------RC Switch Definition------------------------- */
typedef enum {
  rc_sw_top = 1,
  rc_sw_bottom,
  rc_sw_middle,
} rc_switch_t;
/* ----------PC Key Definition---------------------- ------*/
#define MOUSE_PRESS ((uint8_t)1)
#define MOUSE_NOT_PRESS ((uint8_t)0)
#define MOUSE_NOMRAL_SPD_MAX ((uint16_t)64)
#define MOUSE_MAX_ABS ((int16_t)32768)
// 左拖x减小,右拖x增大
// 上拖y减小,下拖y增大
// 滚轮上滚z增大,下滚z减小				//键鼠控制
// //配置模式
#define KEY_W ((uint16_t)0x01 << 0)     // 前进占用
#define KEY_S ((uint16_t)0x01 << 1)     // 后退占用
#define KEY_A ((uint16_t)0x01 << 2)     // 左移占用
#define KEY_D ((uint16_t)0x01 << 3)     // 右移占用
#define KEY_SHIFT ((uint16_t)0x01 << 4) // 超级电容占用
#define KEY_CTRL ((uint16_t)0x01 << 5)  // 视觉自瞄占用
#define KEY_Q ((uint16_t)0x01 << 6)     // 坦克模式占用
#define KEY_E ((uint16_t)0x01 << 7)     // 跟随模式占用

#define KEY_R ((uint16_t)0x01 << 8)  // 一键重启
#define KEY_F ((uint16_t)0x01 << 9)  // FUCk模式占用
#define KEY_G ((uint16_t)0x01 << 10) // 倍镜占用
#define KEY_Z                                                                  \
  ((uint16_t)0x01 << 11)             // 小陀螺占用				//配置正向移动
#define KEY_X ((uint16_t)0x01 << 12) // 吊射模式占用
#define KEY_C ((uint16_t)0x01 << 13) // 开摩擦轮占用
#define KEY_V ((uint16_t)0x01 << 14) // 关摩擦轮占用
#define KEY_B ((uint16_t)0x01 << 15) // 基地模式占用
/* ------------------Defined Errors------------------*/
#define RC_NO_ERROR 0
#define RC_CH_ERROR 0xFF
#define RC_VERIFY_ERR 0xFE
#define RC_RX_LOST_MAX ((uint8_t)15)

/* ------------------Defined Marcos------------------*/

#define IS_KEY_PRESS(KEY)                                                      \
  ((rc_ctrl_data.keyboard.keycode & (KEY)) == (KEY) ? true : false)
#define IS_KEY_LAST_PRESS(KEY)                                                 \
  ((rc_ctrl_data.keyboard.last_keycode & (KEY)) == (KEY) ? true : false)

/* ------------------Data struct Data __packed struct -------*/
#define RC_FRAME_LENGTH 18

/**
 * @struct rc_ctrl_t
 * @brief all remote control data
 */
typedef struct {
  struct {
    int16_t ch0; ///< joystick channel 1 (11bit, 364-1684)
                 ///< //右拨杆左右(左-660,右-660)
    int16_t ch1; ///< joystick channel 2 (11bit, 364-1684)
                 ///< //右拨杆上下(上+660,下-660)
    int16_t ch2; ///< joystick channel 3 (11bit, 364-1684)
                 ///< //左拨杆左右(左-660,右-660)
    int16_t ch3; ///< joystick channel 4 (11bit, 364-1684)
                 ///< //左拨杆上下(上+660,下-660)
    int16_t ch4;
    rc_switch_t switch_left;   ///< left switch(2bit, 0-2)
                               ///< //高->1,中->3,低->2
    rc_switch_t switch_right;  ///< right switch(2bit, 0-2)
                               ///< //
    uint8_t last_switch_left;  ///< last left switch(2bit, 0-2)
    uint8_t last_switch_right; ///< last right switch(2bit, 0-2)
  } rc;
  struct {
    int16_t x;               ///< mouse velocity of x axis(16bit, -32767-32767)
    int16_t y;               ///< mouse velocity of y axis(16bit, -32767-32767)
    int16_t z;               ///< mouse velocity of z axis(16bit, -32767-32767)
    uint8_t press_left;      ///< the left key of mouse(8bit,0 or 1)
    uint8_t press_right;     ///< the right key of mouse(8bit,0 or 1)
    uint8_t last_press_left; ///< the last press left key of mouse(8bit,0 or 1)
    uint8_t
        last_press_right; ///< the last press right key of mouse(8bit,0 or 1)
  } mouse;
  struct {
    uint16_t keycode; ///< the flags current pressed key, use marco IS_KEY_PRESS
                      ///< to parse
    uint16_t last_keycode; ///< the flags last pressed key, use marco
                           ///< IS_KEY_PRESS to parse
  } keyboard;
} rc_ctrl_t;

extern rc_ctrl_t rc_ctrl_data;
#define rc_left_y (float)rc_ctrl_data.rc.ch2 / RC_CH_VALUE_RANGE
#define rc_left_x (float)rc_ctrl_data.rc.ch3 / RC_CH_VALUE_RANGE
#define rc_right_y (float)rc_ctrl_data.rc.ch1 / RC_CH_VALUE_RANGE
#define rc_right_x (float)rc_ctrl_data.rc.ch0 / RC_CH_VALUE_RANGE
#define rc_clickwheel (float)rc_ctrl_data.rc.ch4 / RC_CH_VALUE_RANGE
#define rc_left_switch rc_ctrl_data.rc.switch_left
#define rc_right_switch rc_ctrl_data.rc.switch_right
#define rc_left_last_switch rc_ctrl_data.rc.last_switch_left
#define rc_right_last_switch rc_ctrl_data.rc.last_switch_right

#define Mouse_x ((float)rc_ctrl_data.mouse.x / MOUSE_NOMRAL_SPD_MAX)
#define Mouse_y ((float)-rc_ctrl_data.mouse.y / MOUSE_NOMRAL_SPD_MAX)
#define Mouse_z ((float)rc_ctrl_data.mouse.z)

enum key_codes {
  W = 1,
  S = 2,
  A = 3,
  D = 4,
  SHIFT = 5,
  CTRL = 6,
  Q = 7,
  E = 8,
  R = 9,
  F = 10,
  G = 11,
  Z = 12,
  X = 13,
  C = 14,
  V = 15,
  B = 16
};

#if USE_VT_RC_UART == 1

#include "./drv_referee/referee.h"
#include "./drv_referee/referee_conf.h"
typedef __packed struct {
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t left_button_down;
  uint8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
} vt_rc_data_t;

typedef __packed struct {
  frame_header_t header;
  uint16_t vt_rc_cmdid;
  vt_rc_data_t data;
  uint16_t crc16;
} vt_rc_frame_t;

#define VT_RC_RX_MAX_LOST 20

#define VT_FRAME_CMDID (0x304)
#define VT_RC_FRAME_LEN (sizeof(vt_rc_frame_t))

HAL_StatusTypeDef vt_rc_recv_dma_init(void);

#endif

#if USE_RC_UART == 1
HAL_StatusTypeDef rc_recv_dma_init(void);
#endif

rc_ctrl_t get_rc_data(void);

void update_rc_last_key(void);

uint32_t is_rc_offline(void);
uint32_t is_vt_rc_offline(void);
void inc_rc_rx_lost(void);

#if USE_VT_RC_UART == 1 || USE_RC_UART == 1
void rc_uart_idle_handle(UART_HandleTypeDef *huart);
#endif

#endif
