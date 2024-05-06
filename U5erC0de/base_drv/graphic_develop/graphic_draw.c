//#include "stm32f4xx_hal.h"
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "./algorithm/crc.h"
#include "./algorithm/util.h"
#include "./base_drv/drv_referee/referee.h"
#include "./base_drv/drv_uart.h"
#include "./base_drv/graphic_develop/graphic_base.h"

//#include "zepi_manager.h"

#include "./base_drv/drv_cap/super_cap.h"

// #include "vision.h"
// #include "algorithm/imu_fusion.h"
// #include "algorithm/util.h"

#include <math.h>

struct
{
	bool Lob_mode;
	bool SuperCOWpower_mode;
	bool WeaponStart;
}CmdCode;


//////////////////
#ifndef PI
#define PI 3.141593f
#endif
/////////////////
#include <string.h>

// vt 11280*720 60fps angle?
// aim r=30 width = 3 (1060??640)
// x(960+-120*2.75),y(540+-280)
#define UI_HUART REFEREE_UART_HANDLE
#define UI_UART_SEND uart_send_async

extern UART_HandleTypeDef UI_HUART;

#define BAR_LEFT 760
#define BAR_LEN 400
#define BAR_RIGHT (BAR_LEFT + BAR_LEN)
#define BAR_HORIZ 130
#define BAR_WIDTH 6
#define MARGIN 4

#define LONG_BRANCH 30
#define SHORT_BRANCH 15
#define CHASSIS_DIR_X 960
#define CHASSIS_DIR_Y 70

#define INDIC_HALF_WIDTH 7
#define INDIC_LEN 7
#define INDIC_MARGIN 10

#define CHASSIS_STATE_BASE_POS_X 1330
#define CHASSIS_STATE_BASE_POS_Y 200

#define INDIC_Y_OFFSET (CHASSIS_STATE_BASE_POS_Y - INDIC_HALF_WIDTH)

// char state_text[30] = "C X G X S X\nI X F X N X\n-";
char chassis_state_text[30] = "NORMAL\nAnti-Targeting";
// char aim_state_text[30] = "SMALL BUFF\nLARGE BUFF";
char cap_state_text[30] = "CAP ON";
static uint8_t tx_buffer[128];

uint16_t mid_x = 1920 / 2;

// ext_client_custom_graphic_seven_t test_data;
//  ext_client_custom_graphic_single_t test_data;

ext_client_custom_graphic_double_t voltage_bar_data;
// ext_client_custom_graphic_double_t chassis_dir_data;
// ext_client_custom_graphic_double_t aim_tracer_data;
ext_client_custom_graphic_single_t state_indicator;
// ext_client_custom_graphic_single_t air_factor_data;
ext_client_custom_graphic_double_t unableFIRE_data = {0};
ext_client_custom_graphic_double_t outpost_data = {0};

ext_client_custom_graphic_five_t dynamic_layer_data;
ext_client_custom_character_t char_data;
// ext_client_custom_graphic_single_t hurt_pos_data;
ext_client_custom_character_t cap_state_data;

ext_client_custom_graphic_seven_t LobMode_data = {0};

ext_client_custom_graphic_delete_t delete_data;

extern cap_data_t cap_data;

uint16_t indicator_y = INDIC_Y_OFFSET;

uint16_t fill_tx_buffer(uint8_t *p_data, uint16_t len)
{
	memset(tx_buffer, 0, 128);

	uint16_t total_size;
	frame_header_t p_header;

	p_header.sof = 0xA5;
	p_header.data_len = len;
	p_header.seq = 0;
	p_header.crc8 = Get_CRC8_Check_Sum((uint8_t *)&p_header, sizeof(frame_header_t) - 1, 0xff);
	memcpy(&tx_buffer, &p_header, sizeof(frame_header_t));

	// uint16_t cmd_id = 0x0301;
	// memcpy(&tx_buffer[sizeof(frame_header_t)], (uint8_t*)&cmd_id, sizeof(uint16_t));
	*(uint16_t *)&tx_buffer[sizeof(frame_header_t)] = 0x0301;

	memcpy(&tx_buffer[sizeof(frame_header_t) + sizeof(uint16_t)], p_data, len);

	total_size = sizeof(frame_header_t) + sizeof(uint16_t) + len + sizeof(uint16_t);
	Append_CRC16_Check_Sum(tx_buffer, total_size);
	return total_size;
}

void set_indicator_color(uint32_t color)
{
	dynamic_layer_data.grapic_data_struct[3].color = color;
}

void set_indicator(uint8_t chassis_mode, uint8_t energy_mode)
{
	if (energy_mode == 0)
	{
		indicator_y = INDIC_Y_OFFSET - chassis_mode * INDIC_HALF_WIDTH * 3 + 2;
	}
	else
	{
		indicator_y = (INDIC_Y_OFFSET - 4 * INDIC_HALF_WIDTH) - energy_mode * INDIC_HALF_WIDTH * 2;
	}
}

void init_ui_static(void)
{
	memset(&voltage_bar_data, 0, sizeof(voltage_bar_data));
	voltage_bar_data.grapic_data_struct[0].graphic_name[0] = 0x21;
	voltage_bar_data.grapic_data_struct[0].graphic_name[1] = 0x01;
	voltage_bar_data.grapic_data_struct[0].graphic_name[2] = 0x01;
	voltage_bar_data.grapic_data_struct[0].operate_type = OP_TYPE_ADD;
	voltage_bar_data.grapic_data_struct[0].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	voltage_bar_data.grapic_data_struct[0].layer = DYNAMIC_LAYER;
	voltage_bar_data.grapic_data_struct[0].color = COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange
	voltage_bar_data.grapic_data_struct[0].start_angle = 0;
	voltage_bar_data.grapic_data_struct[0].end_angle = 0;
	voltage_bar_data.grapic_data_struct[0].width = BAR_WIDTH * 2;
	voltage_bar_data.grapic_data_struct[0].start_x = BAR_LEFT;
	voltage_bar_data.grapic_data_struct[0].start_y = BAR_HORIZ;
	voltage_bar_data.grapic_data_struct[0].radius = 0;
	voltage_bar_data.grapic_data_struct[0].end_x = BAR_RIGHT;
	voltage_bar_data.grapic_data_struct[0].end_y = BAR_HORIZ;
	memcpy(&dynamic_layer_data.grapic_data_struct[0], &voltage_bar_data.grapic_data_struct[0], sizeof(graphic_data_struct_t));

	voltage_bar_data.grapic_data_struct[1].graphic_name[0] = 0x21;
	voltage_bar_data.grapic_data_struct[1].graphic_name[1] = 0x01;
	voltage_bar_data.grapic_data_struct[1].graphic_name[2] = 0x02;
	voltage_bar_data.grapic_data_struct[1].operate_type = OP_TYPE_ADD;
	voltage_bar_data.grapic_data_struct[1].graphic_type = GRAPHIC_SQUARE; // 0-line 2- circle 7-char
	voltage_bar_data.grapic_data_struct[1].layer = BACKGROUND_LAYER;
	voltage_bar_data.grapic_data_struct[1].color = COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange
	voltage_bar_data.grapic_data_struct[1].start_angle = 0;
	voltage_bar_data.grapic_data_struct[1].end_angle = 0;
	voltage_bar_data.grapic_data_struct[1].width = 2;
	voltage_bar_data.grapic_data_struct[1].start_x = BAR_LEFT - MARGIN;
	voltage_bar_data.grapic_data_struct[1].start_y = BAR_HORIZ + BAR_WIDTH + MARGIN;
	voltage_bar_data.grapic_data_struct[1].radius = 0;
	voltage_bar_data.grapic_data_struct[1].end_x = BAR_RIGHT + MARGIN;
	voltage_bar_data.grapic_data_struct[1].end_y = BAR_HORIZ - BAR_WIDTH - MARGIN;

	state_indicator.grapic_data_struct.graphic_name[0] = 0x21;
	state_indicator.grapic_data_struct.graphic_name[1] = 0x02;
	state_indicator.grapic_data_struct.graphic_name[2] = 0x01;

	state_indicator.grapic_data_struct.operate_type = OP_TYPE_ADD;
	state_indicator.grapic_data_struct.graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	state_indicator.grapic_data_struct.layer = DYNAMIC_LAYER;
	state_indicator.grapic_data_struct.color = COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange
	state_indicator.grapic_data_struct.start_angle = 0;
	state_indicator.grapic_data_struct.end_angle = 0;
	state_indicator.grapic_data_struct.width = (INDIC_HALF_WIDTH * 2);
	//(460,350)(460,650)
	state_indicator.grapic_data_struct.start_x = (CHASSIS_STATE_BASE_POS_X - INDIC_MARGIN - INDIC_LEN);
	state_indicator.grapic_data_struct.start_y = (CHASSIS_STATE_BASE_POS_Y - INDIC_HALF_WIDTH);
	state_indicator.grapic_data_struct.radius = 0;
	state_indicator.grapic_data_struct.end_x = (CHASSIS_STATE_BASE_POS_X - INDIC_MARGIN);
	state_indicator.grapic_data_struct.end_y = (CHASSIS_STATE_BASE_POS_Y - INDIC_HALF_WIDTH);
	memcpy(&dynamic_layer_data.grapic_data_struct[3], &state_indicator.grapic_data_struct, sizeof(graphic_data_struct_t));

	char_data.grapic_data_struct.graphic_name[0] = 0x21;
	char_data.grapic_data_struct.graphic_name[1] = 0x20;
	char_data.grapic_data_struct.graphic_name[2] = 0x00;

	char_data.grapic_data_struct.operate_type = OP_TYPE_ADD;
	char_data.grapic_data_struct.graphic_type = GRAPHIC_CHAR;
	char_data.grapic_data_struct.layer = TEXT_LAYER;
	char_data.grapic_data_struct.color = COLOR_GREEN;
	char_data.grapic_data_struct.width = 2;

	char_data.grapic_data_struct.start_x = 1100;
	char_data.grapic_data_struct.start_y = 100;
	char_data.grapic_data_struct.radius = 0;
	char_data.grapic_data_struct.end_x = 0;
	char_data.grapic_data_struct.end_y = 0; // 350 + 300

	cap_state_data.grapic_data_struct.graphic_name[0] = 0x22;
	cap_state_data.grapic_data_struct.graphic_name[1] = 0x01;
	cap_state_data.grapic_data_struct.graphic_name[2] = 0x01;

	cap_state_data.grapic_data_struct.operate_type = OP_TYPE_ADD;
	cap_state_data.grapic_data_struct.graphic_type = GRAPHIC_CHAR; // 0-line 2- circle 7-char
	cap_state_data.grapic_data_struct.layer = TEXT_LAYER;
	cap_state_data.grapic_data_struct.color = COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange
	cap_state_data.grapic_data_struct.start_angle = 20;
	cap_state_data.grapic_data_struct.end_angle = strlen(cap_state_text);
	cap_state_data.grapic_data_struct.width = 3;
	cap_state_data.grapic_data_struct.start_x = 800;
	cap_state_data.grapic_data_struct.start_y = 100;
	cap_state_data.grapic_data_struct.radius = 0;
	cap_state_data.grapic_data_struct.end_x = 0;
	cap_state_data.grapic_data_struct.end_y = 0;

#define UNABLESHOOT_X (1920 / 2)
#define UNABLESHOOT_Y (1080 / 2)
#define UNABLESHOOT_R 50
	unableFIRE_data.grapic_data_struct[0].graphic_name[0] = 0x22;
	unableFIRE_data.grapic_data_struct[0].graphic_name[1] = 0x02;
	unableFIRE_data.grapic_data_struct[0].graphic_name[2] = 0x01;

	unableFIRE_data.grapic_data_struct[0].operate_type = OP_TYPE_ADD;
	unableFIRE_data.grapic_data_struct[0].graphic_type = GRAPHIC_CIRCLE; // 0-line 2- circle 7-char
	unableFIRE_data.grapic_data_struct[0].layer = DYNAMIC_LAYER;
	unableFIRE_data.grapic_data_struct[0].color = COLOR_ORANGE;
	unableFIRE_data.grapic_data_struct[0].start_angle = 0;
	unableFIRE_data.grapic_data_struct[0].end_angle = 0;
	unableFIRE_data.grapic_data_struct[0].width = 5;
	unableFIRE_data.grapic_data_struct[0].start_x = UNABLESHOOT_X;
	unableFIRE_data.grapic_data_struct[0].start_y = UNABLESHOOT_Y;
	unableFIRE_data.grapic_data_struct[0].radius = UNABLESHOOT_R;
	unableFIRE_data.grapic_data_struct[0].end_x = 0;
	unableFIRE_data.grapic_data_struct[0].end_y = 0;

	unableFIRE_data.grapic_data_struct[1].graphic_name[0] = 0x22;
	unableFIRE_data.grapic_data_struct[1].graphic_name[1] = 0x02;
	unableFIRE_data.grapic_data_struct[1].graphic_name[2] = 0x02;

	unableFIRE_data.grapic_data_struct[1].operate_type = OP_TYPE_ADD;
	unableFIRE_data.grapic_data_struct[1].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	unableFIRE_data.grapic_data_struct[1].layer = DYNAMIC_LAYER;
	unableFIRE_data.grapic_data_struct[1].color = COLOR_ORANGE;
	unableFIRE_data.grapic_data_struct[1].start_angle = 0;
	unableFIRE_data.grapic_data_struct[1].end_angle = 0;
	unableFIRE_data.grapic_data_struct[1].width = 5;
	unableFIRE_data.grapic_data_struct[1].start_x = UNABLESHOOT_X - UNABLESHOOT_R * Sqrt2 / 2;
	unableFIRE_data.grapic_data_struct[1].start_y = UNABLESHOOT_Y - UNABLESHOOT_R * Sqrt2 / 2;
	unableFIRE_data.grapic_data_struct[1].radius = 0;
	unableFIRE_data.grapic_data_struct[1].end_x = UNABLESHOOT_X + UNABLESHOOT_R * Sqrt2 / 2;
	unableFIRE_data.grapic_data_struct[1].end_y = UNABLESHOOT_Y + UNABLESHOOT_R * Sqrt2 / 2;

	#define LobMode_xBase 90
	#define LobMode_Limx 35
	#define LobMode_x1 -30
	#define LobMode_x2 -40
	#define LobMode_y1 (1080 - 565)
	#define LobMode_y2 (1080 - 590)
	#define LobMode_y3 (1080 - 620)
	LobMode_data.grapic_data_struct[0].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[0].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[0].graphic_name[2] = 0x01;

	LobMode_data.grapic_data_struct[0].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[0].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	LobMode_data.grapic_data_struct[0].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[0].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[0].start_angle = 0;
	LobMode_data.grapic_data_struct[0].end_angle = 0;
	LobMode_data.grapic_data_struct[0].width = 1;
	LobMode_data.grapic_data_struct[0].start_x = mid_x;
	LobMode_data.grapic_data_struct[0].start_y = 0;
	LobMode_data.grapic_data_struct[0].radius = 0;
	LobMode_data.grapic_data_struct[0].end_x = mid_x;
	LobMode_data.grapic_data_struct[0].end_y = 1080;

	LobMode_data.grapic_data_struct[1].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[1].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[1].graphic_name[2] = 0x02;

	LobMode_data.grapic_data_struct[1].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[1].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	LobMode_data.grapic_data_struct[1].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[1].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[1].start_angle = 0;
	LobMode_data.grapic_data_struct[1].end_angle = 0;
	LobMode_data.grapic_data_struct[1].width = 1;
	LobMode_data.grapic_data_struct[1].start_x = mid_x - LobMode_xBase;
	LobMode_data.grapic_data_struct[1].start_y = 540;
	LobMode_data.grapic_data_struct[1].radius = 0;
	LobMode_data.grapic_data_struct[1].end_x = mid_x + LobMode_xBase;
	LobMode_data.grapic_data_struct[1].end_y = 540;

	LobMode_data.grapic_data_struct[2].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[2].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[2].graphic_name[2] = 0x03;

	LobMode_data.grapic_data_struct[2].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[2].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	LobMode_data.grapic_data_struct[2].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[2].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[2].start_angle = 0;
	LobMode_data.grapic_data_struct[2].end_angle = 0;
	LobMode_data.grapic_data_struct[2].width = 1;
	LobMode_data.grapic_data_struct[2].start_x = mid_x - LobMode_xBase - LobMode_x1;
	LobMode_data.grapic_data_struct[2].start_y = LobMode_y1;
	LobMode_data.grapic_data_struct[2].radius = 0;
	LobMode_data.grapic_data_struct[2].end_x = mid_x + LobMode_xBase + LobMode_x1;
	LobMode_data.grapic_data_struct[2].end_y = LobMode_y1;

	LobMode_data.grapic_data_struct[3].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[3].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[3].graphic_name[2] = 0x04;

	LobMode_data.grapic_data_struct[3].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[3].graphic_type = GRAPHIC_LINE; // 0-line 3- circle 7-char
	LobMode_data.grapic_data_struct[3].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[3].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[3].start_angle = 0;
	LobMode_data.grapic_data_struct[3].end_angle = 0;
	LobMode_data.grapic_data_struct[3].width = 1;
	LobMode_data.grapic_data_struct[3].start_x = mid_x - LobMode_xBase - LobMode_x1;
	LobMode_data.grapic_data_struct[3].start_y = LobMode_y2;
	LobMode_data.grapic_data_struct[3].radius = 0;
	LobMode_data.grapic_data_struct[3].end_x = mid_x + LobMode_xBase + LobMode_x1;
	LobMode_data.grapic_data_struct[3].end_y = LobMode_y2;

	LobMode_data.grapic_data_struct[4].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[4].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[4].graphic_name[2] = 0x05;

	LobMode_data.grapic_data_struct[4].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[4].graphic_type = GRAPHIC_LINE; // 0-line 3- circle 7-char
	LobMode_data.grapic_data_struct[4].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[4].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[4].start_angle = 0;
	LobMode_data.grapic_data_struct[4].end_angle = 0;
	LobMode_data.grapic_data_struct[4].width = 1;
	LobMode_data.grapic_data_struct[4].start_x = mid_x - LobMode_xBase - LobMode_x2;
	LobMode_data.grapic_data_struct[4].start_y = LobMode_y3;
	LobMode_data.grapic_data_struct[4].radius = 0;
	LobMode_data.grapic_data_struct[4].end_x = mid_x + LobMode_xBase + LobMode_x2;
	LobMode_data.grapic_data_struct[4].end_y = LobMode_y3;

	LobMode_data.grapic_data_struct[5].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[5].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[5].graphic_name[2] = 0x06;

	LobMode_data.grapic_data_struct[5].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[5].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	LobMode_data.grapic_data_struct[5].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[5].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[5].start_angle = 0;
	LobMode_data.grapic_data_struct[5].end_angle = 0;
	LobMode_data.grapic_data_struct[5].width = 1;
	LobMode_data.grapic_data_struct[5].start_x = mid_x - LobMode_Limx;
	LobMode_data.grapic_data_struct[5].start_y = 540;
	LobMode_data.grapic_data_struct[5].radius = 0;
	LobMode_data.grapic_data_struct[5].end_x = mid_x - LobMode_Limx;
	LobMode_data.grapic_data_struct[5].end_y = LobMode_y3;

	LobMode_data.grapic_data_struct[6].graphic_name[0] = 0x22;
	LobMode_data.grapic_data_struct[6].graphic_name[1] = 0x03;
	LobMode_data.grapic_data_struct[6].graphic_name[2] = 0x07;

	LobMode_data.grapic_data_struct[6].operate_type = OP_TYPE_ADD;
	LobMode_data.grapic_data_struct[6].graphic_type = GRAPHIC_LINE; // 0-line 2- circle 7-char
	LobMode_data.grapic_data_struct[6].layer = DYNAMIC_LAYER;
	LobMode_data.grapic_data_struct[6].color = COLOR_CYAN;
	LobMode_data.grapic_data_struct[6].start_angle = 0;
	LobMode_data.grapic_data_struct[6].end_angle = 0;
	LobMode_data.grapic_data_struct[6].width = 1;
	LobMode_data.grapic_data_struct[6].start_x = mid_x + LobMode_Limx;
	LobMode_data.grapic_data_struct[6].start_y = 540;
	LobMode_data.grapic_data_struct[6].radius = 0;
	LobMode_data.grapic_data_struct[6].end_x = mid_x + LobMode_Limx;
	LobMode_data.grapic_data_struct[6].end_y = LobMode_y3;
}

void update_background(void)
{
	uint16_t total_len = 0;

	voltage_bar_data.header.data_cmd_id = 0x102;
	voltage_bar_data.header.sender_ID = game_robot_status.robot_id;
	voltage_bar_data.header.receiver_ID = game_robot_status.robot_id + 0x100;
	total_len = fill_tx_buffer((uint8_t *)&voltage_bar_data, sizeof(voltage_bar_data));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);

	state_indicator.header.data_cmd_id = 0x101;
	state_indicator.header.sender_ID = game_robot_status.robot_id;
	state_indicator.header.receiver_ID = game_robot_status.robot_id + 0x100;
	state_indicator.grapic_data_struct.operate_type = OP_TYPE_ADD;
	total_len = fill_tx_buffer((uint8_t *)&state_indicator, sizeof(state_indicator));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);

	char_data.header.data_cmd_id = 0x110;
	char_data.header.sender_ID = game_robot_status.robot_id;
	char_data.header.receiver_ID = game_robot_status.robot_id + 0x100;
	char_data.grapic_data_struct.operate_type = OP_TYPE_ADD;
	char_data.grapic_data_struct.color = COLOR_GREEN;

	char_data.grapic_data_struct.graphic_name[0] = 0x22;
	char_data.grapic_data_struct.graphic_name[1] = 0x20;
	char_data.grapic_data_struct.graphic_name[2] = 0x01;

	char_data.grapic_data_struct.start_x = CHASSIS_STATE_BASE_POS_X;
	char_data.grapic_data_struct.start_y = CHASSIS_STATE_BASE_POS_Y;
	char_data.grapic_data_struct.start_angle = 12;
	char_data.grapic_data_struct.end_angle = strlen(chassis_state_text);
	memset(char_data.data, 0, 30);
	memcpy(char_data.data, chassis_state_text, strlen(chassis_state_text));
	total_len = fill_tx_buffer((uint8_t *)&char_data, sizeof(char_data));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);

	LobMode_data.header.data_cmd_id = 0x104;
	LobMode_data.header.sender_ID = game_robot_status.robot_id;
	LobMode_data.header.receiver_ID = game_robot_status.robot_id + 0x100;
	if(CmdCode.Lob_mode)
	{
		LobMode_data.grapic_data_struct[0].operate_type = OP_TYPE_ADD;
		LobMode_data.grapic_data_struct[1].operate_type = OP_TYPE_ADD;
		LobMode_data.grapic_data_struct[2].operate_type = OP_TYPE_ADD;
		LobMode_data.grapic_data_struct[3].operate_type = OP_TYPE_ADD;
		LobMode_data.grapic_data_struct[4].operate_type = OP_TYPE_ADD;
		LobMode_data.grapic_data_struct[5].operate_type = OP_TYPE_ADD;
		LobMode_data.grapic_data_struct[6].operate_type = OP_TYPE_ADD;
	}
	else
	{
		LobMode_data.grapic_data_struct[0].operate_type = OP_TYPE_DEL;
		LobMode_data.grapic_data_struct[1].operate_type = OP_TYPE_DEL;
		LobMode_data.grapic_data_struct[2].operate_type = OP_TYPE_DEL;
		LobMode_data.grapic_data_struct[3].operate_type = OP_TYPE_DEL;
		LobMode_data.grapic_data_struct[4].operate_type = OP_TYPE_DEL;
		LobMode_data.grapic_data_struct[5].operate_type = OP_TYPE_DEL;
		LobMode_data.grapic_data_struct[6].operate_type = OP_TYPE_DEL;
	}
	total_len = fill_tx_buffer((uint8_t *)&LobMode_data, sizeof(LobMode_data));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);
}


float cap_limitvolt=12.f;
void update_ui(float cap_voltage)
{
	uint16_t total_len = 0;
	dynamic_layer_data.header.data_cmd_id = 0x103;
	dynamic_layer_data.header.sender_ID = game_robot_status.robot_id;
	dynamic_layer_data.header.receiver_ID = game_robot_status.robot_id + 0x100;

	if (cap_voltage < cap_limitvolt)
		dynamic_layer_data.grapic_data_struct[0].color = COLOR_PURPLE; // 2/6-green,1-yellow,0-main,3-orange
	else if (cap_voltage < cap_limitvolt * 1.26f)
		dynamic_layer_data.grapic_data_struct[0].color = COLOR_YELLOW; // 2/6-green,1-yellow,0-main,3-orange
	else
		dynamic_layer_data.grapic_data_struct[0].color = COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange

	dynamic_layer_data.grapic_data_struct[0].end_x = BAR_LEFT + clamp((cap_voltage - cap_limitvolt) / (cap_data.input_voltage - 2.1f - cap_limitvolt), 0, 1) * BAR_LEN; // 350 +300
	dynamic_layer_data.grapic_data_struct[0].operate_type = OP_TYPE_MOD;

	dynamic_layer_data.grapic_data_struct[1].start_y = indicator_y;
	dynamic_layer_data.grapic_data_struct[1].end_y = indicator_y;

	dynamic_layer_data.grapic_data_struct[1].operate_type = OP_TYPE_MOD;
	dynamic_layer_data.grapic_data_struct[2].operate_type = OP_TYPE_MOD;
	dynamic_layer_data.grapic_data_struct[3].operate_type = OP_TYPE_MOD;
	dynamic_layer_data.grapic_data_struct[4].operate_type = OP_TYPE_MOD;
	total_len = fill_tx_buffer((uint8_t *)&dynamic_layer_data, sizeof(dynamic_layer_data));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);

	cap_state_data.header.data_cmd_id = 0x110;
	cap_state_data.header.sender_ID = game_robot_status.robot_id;
	cap_state_data.header.receiver_ID = game_robot_status.robot_id + 0x100;

	if (CmdCode.SuperCOWpower_mode)
		cap_state_data.grapic_data_struct.operate_type = OP_TYPE_ADD;
	else
		cap_state_data.grapic_data_struct.operate_type = OP_TYPE_DEL;
	cap_state_data.grapic_data_struct.start_x = BAR_LEFT - 150;
	cap_state_data.grapic_data_struct.start_y = BAR_HORIZ + 5;
	cap_state_data.grapic_data_struct.start_angle = 15;
	memcpy(cap_state_data.data, cap_state_text, strlen(cap_state_text));
	total_len = fill_tx_buffer((uint8_t *)&cap_state_data, sizeof(cap_state_data));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);

	unableFIRE_data.header.data_cmd_id = 0x102;
	unableFIRE_data.header.sender_ID = game_robot_status.robot_id;
	unableFIRE_data.header.receiver_ID = game_robot_status.robot_id + 0x100;

	if (!CmdCode.WeaponStart)
		unableFIRE_data.grapic_data_struct[0].operate_type = OP_TYPE_ADD;
	else
		unableFIRE_data.grapic_data_struct[0].operate_type = OP_TYPE_DEL;
	if (!CmdCode.WeaponStart)
		unableFIRE_data.grapic_data_struct[1].operate_type = OP_TYPE_ADD;
	else
		unableFIRE_data.grapic_data_struct[1].operate_type = OP_TYPE_DEL;
	total_len = fill_tx_buffer((uint8_t *)&unableFIRE_data, sizeof(unableFIRE_data));
	UI_UART_SEND(&UI_HUART, tx_buffer, total_len);

	// LobMode_data.header.data_cmd_id = 0x101;
	// LobMode_data.header.sender_ID = game_robot_status.robot_id;
	// LobMode_data.header.receiver_ID = game_robot_status.robot_id + 0x100;

	// if (CmdCode.Lob_mode)
	// 	LobMode_data.grapic_data_struct.operate_type = OP_TYPE_ADD;
	// else
	// 	LobMode_data.grapic_data_struct.operate_type = OP_TYPE_DEL;
	// total_len = fill_tx_buffer((uint8_t *)&LobMode_data, sizeof(LobMode_data));
	// UI_UART_SEND(&UI_HUART, tx_buffer, total_len);
}
