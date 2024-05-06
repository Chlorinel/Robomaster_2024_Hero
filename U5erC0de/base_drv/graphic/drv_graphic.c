#include "./base_drv/graphic/drv_graphic.h"
#include "./algorithm/crc.h"
#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/drv_referee/referee.h"
#include "./base_drv/drv_uart.h"
#include "./base_drv/graphic/graphic_def.h"
#include "stm32f4xx_hal.h"

// #include "./robot_core/weapon/vision.h"
// #include "./algorithm/imu_fusion.h"
#include <math.h>
#include <string.h>

#include "./algorithm/util.h"
#include "./robot_core/ctrl_core/chassis.h"
#include "./robot_core/interface/interface_BTB.h"

extern UART_HandleTypeDef huart6;
#define UI_HUART huart6
#define DEFINE_FRAME_PROC(num, id)
DEFINE_FRAME_PROC(1, 0x0101)
DEFINE_FRAME_PROC(2, 0x0102)
DEFINE_FRAME_PROC(5, 0x0103)
DEFINE_FRAME_PROC(7, 0x0104)

uint8_t test_status = 0;
static uint8_t tx_buffer[128];

ext_client_custom_graphic_single_t fl;
ext_client_custom_character_t char_data;
interaction_figure_4_t dynamic_layer_data1;
interaction_figure_4_t dynamic_layer_data2;
interaction_figure_4_t dynamic_layer_data3;

uint16_t fill_tx_buffer(uint8_t *p_data, uint16_t len) {
  memset(tx_buffer, 0, 128);

  uint16_t total_size;
  frame_header_t p_header;

  p_header.sof = 0xA5;
  p_header.data_len = len;
  p_header.seq = 0;
  p_header.crc8 = Get_CRC8_Check_Sum((uint8_t *)&p_header,
                                     sizeof(frame_header_t) - 1, 0xff);
  memcpy(&tx_buffer, &p_header, sizeof(frame_header_t));

  // uint16_t cmd_id = 0x0301;
  // memcpy(&tx_buffer[sizeof(frame_header_t)], (uint8_t*)&cmd_id,
  // sizeof(uint16_t));
  *(uint16_t *)&tx_buffer[sizeof(frame_header_t)] = 0x0301;

  memcpy(&tx_buffer[sizeof(frame_header_t) + sizeof(uint16_t)], p_data, len);

  total_size =
      sizeof(frame_header_t) + sizeof(uint16_t) + len + sizeof(uint16_t);
  Get_CRC16_Check_Sum(tx_buffer, total_size, 0xFFFF);
  return total_size;
}
// 不能用来设置字符串类型的ui
void Static_UI(uint8_t rank, uint8_t type, uint8_t color, uint16_t start_x,
               uint16_t start_y, uint16_t details_d, uint16_t details_e,
               uint8_t width) {
  // 八个参数：1.序号 2、种类 3、颜色 4-7x和y 8、线宽
  // 如果类型是圆，请在details_d变量写入半径
  ext_client_custom_graphic_single_t buffer;
  buffer.interaction_figure.figure_name[0] = 0x30;
  buffer.interaction_figure.figure_name[1] = 0x23;
  buffer.interaction_figure.figure_name[2] = 0x22 + rank;

  buffer.interaction_figure.operate_type = OP_TYPE_ADD;
  buffer.interaction_figure.figure_type = type;
  buffer.interaction_figure.layer = DYNAMIC_LAYER;
  buffer.interaction_figure.color = color;
  buffer.interaction_figure.details_a = 0;
  buffer.interaction_figure.details_b = 0;
  buffer.interaction_figure.width = width;

  buffer.interaction_figure.start_x = start_x;
  buffer.interaction_figure.start_y = start_y;

  if (type == 2) {
    buffer.interaction_figure.details_c = details_d;
    buffer.interaction_figure.details_d = 0;
    buffer.interaction_figure.details_e = 0;
  } else {
    buffer.interaction_figure.details_c = 0;
    buffer.interaction_figure.details_d = details_d;
    buffer.interaction_figure.details_e = details_e;
  }
  if (rank < 7)
    memcpy(&dynamic_layer_data1.interaction_figure[rank],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
  else if (7 <= rank && rank < 14)
    memcpy(&dynamic_layer_data2.interaction_figure[rank - 7],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
  else if (14 <= rank && rank < 21)
    memcpy(&dynamic_layer_data3.interaction_figure[rank - 14],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
}

void Static_ARC_UI(uint8_t rank, uint8_t type, uint8_t color, uint16_t start_x,
                   uint16_t start_y, uint16_t details_a, uint16_t details_b,
                   uint16_t details_d, uint16_t details_e, uint8_t width) {
  // 八个参数：1.序号 2、种类 3、颜色 4-7x和y 8、线宽
  // 如果类型是圆，请在details_d变量写入半径
  ext_client_custom_graphic_single_t buffer;
  buffer.interaction_figure.figure_name[0] = 0x30;
  buffer.interaction_figure.figure_name[1] = 0x23;
  buffer.interaction_figure.figure_name[2] = 0x22 + rank;

  buffer.interaction_figure.operate_type = OP_TYPE_ADD;
  buffer.interaction_figure.figure_type = type;
  buffer.interaction_figure.layer = DYNAMIC_LAYER;
  buffer.interaction_figure.color = color;
  buffer.interaction_figure.details_a = 0;
  buffer.interaction_figure.details_b = 0;
  buffer.interaction_figure.width = width;

  buffer.interaction_figure.start_x = start_x;
  buffer.interaction_figure.start_y = start_y;

  if (type == 2) {
    buffer.interaction_figure.details_c = details_d;
    buffer.interaction_figure.details_d = 0;
    buffer.interaction_figure.details_e = 0;
  } else {
    buffer.interaction_figure.details_c = 0;
    buffer.interaction_figure.details_d = details_d;
    buffer.interaction_figure.details_e = details_e;
  }
  if (rank < 7)
    memcpy(&dynamic_layer_data1.interaction_figure[rank],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
  else if (7 <= rank && rank < 14)
    memcpy(&dynamic_layer_data2.interaction_figure[rank - 7],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
  else if (14 <= rank && rank < 21)
    memcpy(&dynamic_layer_data3.interaction_figure[rank - 14],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
}

void modify(uint8_t rank, uint8_t appear, uint8_t color, uint16_t details_d,
            uint16_t details_e) { // 1.序号 2.出现 3.颜色 4.x 5.y
  ext_client_custom_graphic_single_t buffer;

  if (rank < 7)
    memcpy(&buffer.interaction_figure,
           &dynamic_layer_data1.interaction_figure[rank],
           sizeof(interaction_figure_t));

  if (appear == 1)
    buffer.interaction_figure.operate_type = OP_TYPE_ADD;
  else if (!appear)
    buffer.interaction_figure.operate_type = OP_TYPE_DEL;
  else if (appear == 2)
    buffer.interaction_figure.operate_type = OP_TYPE_MOD;

  if (appear == OP_TYPE_MOD) // 当appear=2时，才可以手动更改数值
  {
    buffer.interaction_figure.color = color;
    buffer.interaction_figure.details_d = details_d;
    buffer.interaction_figure.details_e = details_e;
  }

  if (rank < 7)
    memcpy(&dynamic_layer_data1.interaction_figure[rank],
           &buffer.interaction_figure, sizeof(interaction_figure_t));
}

void send_7(interaction_figure_4_t layer_data7) {
  layer_data7.header.data_cmd_id = 0x104;
  layer_data7.header.sender_ID = game_robot_status.robot_id;
  layer_data7.header.receiver_ID = game_robot_status.robot_id + 0x100;

  HAL_Delay(10);
  send_referee_single(&UI_HUART, 0x0301, (uint8_t *)&layer_data7,
                      sizeof(layer_data7));
}

void send_5(interaction_figure_3_t layer_data) {
  layer_data.header.data_cmd_id = 0x103;
  layer_data.header.sender_ID = game_robot_status.robot_id;
  layer_data.header.receiver_ID = game_robot_status.robot_id + 0x100;

  HAL_Delay(10);
  send_referee_single(&UI_HUART, 0x0301, (uint8_t *)&layer_data,
                      sizeof(layer_data));
}

void char_data_init(void) {
  char_data.grapic_data_struct.interaction_figure.figure_name[0] = 0x30;
  char_data.grapic_data_struct.interaction_figure.figure_name[1] = 0x23;
  char_data.grapic_data_struct.interaction_figure.figure_name[2] = 0x22;

  char_data.grapic_data_struct.interaction_figure.operate_type = OP_TYPE_ADD;
  char_data.grapic_data_struct.interaction_figure.figure_type =
      GRAPHIC_CHAR; // 0-line 2- circle 7-char
  char_data.grapic_data_struct.interaction_figure.layer = TEXT_LAYER;
  char_data.grapic_data_struct.interaction_figure.color =
      COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange
  char_data.grapic_data_struct.interaction_figure.details_a = 20;
  char_data.grapic_data_struct.interaction_figure.details_b = 20;
  char_data.grapic_data_struct.interaction_figure.width = 2;
  char_data.grapic_data_struct.interaction_figure.start_x = 1100;
  char_data.grapic_data_struct.interaction_figure.start_y = 500;
  char_data.grapic_data_struct.interaction_figure.details_c = 0;
  char_data.grapic_data_struct.interaction_figure.details_d = 0;
  char_data.grapic_data_struct.interaction_figure.details_e = 0; // 350 +300
}

// void send_char(uint16_t x,uint16_t y,uint8_t width,uint8_t size,uint8_t
// size_of,uint8_t str)
void send_char(uint8_t rank, uint8_t appear, uint16_t x, uint16_t y,
               uint8_t color, uint8_t size, uint8_t width, char *str) {
  uint16_t total_len = 0;

  char_data.grapic_data_struct.interaction_figure.color = color;

  if (appear)
    char_data.grapic_data_struct.interaction_figure.operate_type = OP_TYPE_ADD;
  else
    char_data.grapic_data_struct.interaction_figure.operate_type = OP_TYPE_DEL;

  char_data.grapic_data_struct.interaction_figure.figure_name[2] = 0x22 + rank;
  char_data.grapic_data_struct.interaction_figure.start_x = x;
  char_data.grapic_data_struct.interaction_figure.start_y = y;
  char_data.grapic_data_struct.interaction_figure.width = width;
  char_data.grapic_data_struct.interaction_figure.details_a = size;
  char_data.grapic_data_struct.interaction_figure.details_b = strlen(str);
  memset(char_data.data, 0, 30);
  memcpy(char_data.data, str, strlen(str));
  char_data.grapic_data_struct.header.data_cmd_id = 0x110; // 绘制字符
  char_data.grapic_data_struct.header.sender_ID = game_robot_status.robot_id;
  char_data.grapic_data_struct.header.receiver_ID =
      game_robot_status.robot_id + 0x100;
  total_len = fill_tx_buffer((uint8_t *)&char_data, sizeof(char_data));
  uart_send_async(&UI_HUART, tx_buffer, total_len);
}

void float_data_init(void) {
  uint16_t total_len = 0;
  fl.header.data_cmd_id = 0x101; // 绘制字符
  fl.header.sender_ID = game_robot_status.robot_id;
  fl.header.receiver_ID = game_robot_status.robot_id + 0x100;
  fl.interaction_figure.details_d = 0;
  fl.interaction_figure.details_e = 0;
  fl.interaction_figure.operate_type = OP_TYPE_ADD;
  fl.interaction_figure.figure_type = 5; // 0-line 2- circle 7-char
  fl.interaction_figure.layer = DYNAMIC_LAYER;
  total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
  uart_send_async(&UI_HUART, tx_buffer, total_len);
}
void send_float(uint8_t appear, uint16_t x, uint16_t y, uint8_t color,
                uint8_t size, uint8_t width, float fl_num) {

  uint16_t total_len = 0;
  int32_t buf;
  fl.interaction_figure.figure_name[0] = 0x13;
  fl.interaction_figure.figure_name[1] = 0x09 + x / 30;
  fl.interaction_figure.figure_name[2] = 0x12 + y / 40;

  if (appear == 0)
    fl.interaction_figure.operate_type = OP_TYPE_DEL;
  else if (appear == 1)
    fl.interaction_figure.operate_type = OP_TYPE_ADD;
  else if (appear == 2)
    fl.interaction_figure.operate_type = OP_TYPE_MOD;

  fl.interaction_figure.color = color; // 2/6-green,1-yellow,0-main,3-orange
  fl.interaction_figure.details_a = size;
  fl.interaction_figure.details_b = 5;
  fl.interaction_figure.width = width;

  fl.interaction_figure.start_x = x;
  fl.interaction_figure.start_y = y;

  // 2/6-green,1-yellow,0-main,3-orange
  buf = fl_num * 10.0f;
  fl.interaction_figure.details_c = buf;
  total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
  uart_send_async(&UI_HUART, tx_buffer, total_len);
}

void int_data_init(void) {
  uint16_t total_len = 0;
  fl.header.data_cmd_id = 0x101; // 绘制字符
  fl.header.sender_ID = game_robot_status.robot_id;
  fl.header.receiver_ID = game_robot_status.robot_id + 0x100;
  fl.interaction_figure.details_d = 0;
  fl.interaction_figure.details_e = 0;
  fl.interaction_figure.operate_type = OP_TYPE_ADD;
  fl.interaction_figure.figure_type = 6; // 0-line 2- circle 7-char
  fl.interaction_figure.layer = DYNAMIC_LAYER;
  total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
  uart_send_async(&UI_HUART, tx_buffer, total_len);
}

void send_int(uint8_t appear, uint16_t x, uint16_t y, uint8_t color,
              uint8_t size, uint8_t width, int32_t int_num) {

  uint16_t total_len = 0;

  fl.interaction_figure.figure_name[0] = 0x13;
  fl.interaction_figure.figure_name[1] = 0x09 + x / 30;
  fl.interaction_figure.figure_name[2] = 0x12 + y / 40 + x / 30;

  if (appear == 0)
    fl.interaction_figure.operate_type = OP_TYPE_DEL;
  else if (appear == 1)
    fl.interaction_figure.operate_type = OP_TYPE_ADD;
  else if (appear == 2)
    fl.interaction_figure.operate_type = OP_TYPE_MOD;

  fl.interaction_figure.color = color; // 2/6-green,1-yellow,0-main,3-orange
  fl.interaction_figure.details_a = size;
  fl.interaction_figure.details_b = 0;
  fl.interaction_figure.width = width;

  fl.interaction_figure.start_x = x;
  fl.interaction_figure.start_y = y;

  // 2/6-green,1-yellow,0-main,3-orange
  fl.interaction_figure.details_c = int_num;
  total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
  uart_send_async(&UI_HUART, tx_buffer, total_len);
}
