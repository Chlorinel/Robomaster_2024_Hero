#ifndef __GRAPHIC_BASE__
#define __GRAPHIC_BASE__

#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
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

#pragma pack(1)
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

typedef __packed struct
{
	ext_student_interactive_header_data_t header;
	uint8_t operate_type;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

typedef __packed struct
{
uint8_t graphic_name[3];

uint32_t operate_type:3;
uint32_t graphic_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;

uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;

uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t ;

typedef __packed struct
{
	ext_student_interactive_header_data_t header;
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

typedef __packed struct
{
	ext_student_interactive_header_data_t header;
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef __packed struct
{
	ext_student_interactive_header_data_t header;
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

typedef __packed struct
{
	ext_student_interactive_header_data_t header;
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;

typedef __packed struct
{
	ext_student_interactive_header_data_t header;
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;
#pragma pack()

enum
{
	OP_TYPE_NOP = 0,
	OP_TYPE_ADD = 1,
	OP_TYPE_MOD = 2,
	OP_TYPE_DEL = 3
};

enum
{
	LAYER_OP_TYPE_NOP = 0,
	LAYER_OP_TYPE_DEL = 1,
	LAYER_OP_TYPE_DEL_ALL = 2
};

enum
{
	GRAPHIC_LINE = 0,
	GRAPHIC_SQUARE = 1,
	GRAPHIC_CIRCLE = 2,
	GRAPHIC_ELLIPSE = 3,
	GRAPHIC_ARC = 4,
	GRAPHIC_FP_NUM = 5,
	GRAPHIC_INT_NUM = 6,
	GRAPHIC_CHAR = 7
};

enum
{
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

void ui_update_test(void);
void init_ui_static(void);
void ui_graphic_delete(uint8_t operate_type, uint8_t layer);

void set_indicator_color(uint32_t color);
void update_module_state(uint32_t pos,char state);
void set_indicator(uint8_t chassis_mode, uint8_t energy_mode);
void update_hurt(int16_t hurt_dir, uint32_t op_type);

void update_background(void);
// void update_vision_ui(uint8_t add_mode);
void update_ui(float cap_voltage);

#endif
