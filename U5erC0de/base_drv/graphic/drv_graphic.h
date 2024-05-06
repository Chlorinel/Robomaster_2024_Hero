#ifndef _DRV_GRAPHIC_H
#define _DRV_GRAPHIC_H

#include "stdint.h"
#define BACKGROUND_LAYER 1
#define DYNAMIC_LAYER 2
#define TEXT_LAYER 3
#define VISION_LAYER 4

#define CHASSIS_STATE 2
#define GIMBAL_STATE 6
#define SHOOT_STATE 10
#define IMU_STATE 14
#define FRIC_STATE 18
#define NUC_STATE 22
#define SHELL_STATE 24

#define MAX_X 1920
#define MAX_Y 1080
typedef __packed struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct {
  ext_student_interactive_header_data_t header;
  uint8_t operate_type;
  uint8_t layer;
} ext_client_custom_graphic_delete_t;
///////////
typedef __packed struct {
  uint8_t figure_name[3];
  uint32_t operate_type : 3;
  uint32_t figure_type : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t details_a : 9;
  uint32_t details_b : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t details_c : 10;
  uint32_t details_d : 11;
  uint32_t details_e : 11;
} interaction_figure_t;

typedef __packed struct {
  ext_student_interactive_header_data_t header;
  interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

typedef __packed struct {
  ext_student_interactive_header_data_t header;
  interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

typedef __packed struct {
  ext_student_interactive_header_data_t header;
  interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

/////////////
typedef __packed struct {
  ext_student_interactive_header_data_t header;
  interaction_figure_t interaction_figure;
} ext_client_custom_graphic_single_t;

typedef __packed struct {
  ext_student_interactive_header_data_t header;
  interaction_figure_t interaction_figure;
} graphic_data_struct_t;

typedef __packed struct {
  graphic_data_struct_t grapic_data_struct;
  uint8_t data[30];
} ext_client_custom_character_t;

typedef __packed struct {
  uint8_t SOF;
  uint16_t length;
  uint8_t seq, crc8;
  uint16_t cmd_id, sub_id;
  uint16_t send_id, recv_id;
} frame_header;

typedef __packed struct {
  frame_header header;
  interaction_figure_t option;
  uint16_t crc16;
} string_frame_t;

enum { OP_TYPE_NOP = 0, OP_TYPE_ADD = 1, OP_TYPE_MOD = 2, OP_TYPE_DEL = 3 };

enum {
  LAYER_OP_TYPE_NOP = 0,
  LAYER_OP_TYPE_DEL = 1,
  LAYER_OP_TYPE_DEL_ALL = 2
};

enum {
  GRAPHIC_LINE = 0,
  GRAPHIC_SQUARE = 1,
  GRAPHIC_CIRCLE = 2,
  GRAPHIC_ELLIPSE = 3,
  GRAPHIC_ARC = 4,
  GRAPHIC_FP_NUM = 5,
  GRAPHIC_INT_NUM = 6,
  GRAPHIC_CHAR = 7
};

enum {
  COLOR_MAIN_RB = 0,
  COLOR_YELLOW = 1,
  COLOR_GREEN = 2,
  COLOR_ORANGE = 3,
  COLOR_PURPLE = 4,
  COLOR_PINK = 5,
  COLOR_CYAN = 6,
  COLOR_BLACK = 7,
  COLOR_WHITE = 8
};

uint16_t fill_tx_buffer(uint8_t *p_data, uint16_t len);
void Static_UI(uint8_t rank, uint8_t type, uint8_t color, uint16_t start_x,
               uint16_t start_y, uint16_t details_d, uint16_t details_e,
               uint8_t width);
void Static_ARC_UI(uint8_t rank, uint8_t type, uint8_t color, uint16_t start_x,
                   uint16_t start_y, uint16_t details_a, uint16_t details_b,
                   uint16_t details_d, uint16_t details_e, uint8_t width);
void modify(uint8_t rank, uint8_t appear, uint8_t color, uint16_t details_d,
            uint16_t details_e);
void send_7(interaction_figure_4_t layer_data7);
void send_5(interaction_figure_3_t layer_data);
void char_data_init(void);
void send_char(uint8_t rank, uint8_t appear, uint16_t x, uint16_t y,
               uint8_t color, uint8_t size, uint8_t width, char *str);
void float_data_init(void);
void send_float(uint8_t appear, uint16_t x, uint16_t y, uint8_t color,
                uint8_t size, uint8_t width, float fl_num);
void int_data_init(void);
void send_int(uint8_t appear, uint16_t x, uint16_t y, uint8_t color,
              uint8_t size, uint8_t width, int32_t int_num);
int Char_ReFresh(ext_client_custom_character_t string_Data);
#endif
