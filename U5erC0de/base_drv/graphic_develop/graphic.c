#include "string.h"
#include "./algorithm/crc.h"
#include "./base_drv/drv_uart.h"
#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/drv_referee/referee.h"
#include "./base_drv/graphic_develop/graphic.h"
//#include "./base_drv/graphic_develop/graphic_base.h"

//#include "zepi_manager.h"

float x = 0.5f;
float y = 0.5f;
uint8_t color = GEc_Main_RaB;

static char chassis_state_text[30] = "NORMAL\nAnti-Targeting";
static char cap_state_text[30] = "CAP ON";

/*
RMGL_GI_d_t voltage_bar_data;
// RMGL_GI_d_t chassis_dir_data;
// RMGL_GI_d_t aim_tracer_data;
RMGL_GI_s_t state_indicator;
// RMGL_GI_s_t air_factor_data;
RMGL_GI_d_t unableFIRE_data = {0};
RMGL_GI_d_t outpost_data = {0};

RMGL_GI_5_t dynamic_layer_data;
RMGL_GI_c_t char_data;
// RMGL_GI_s_t hurt_pos_data;
RMGL_GI_c_t cap_state_data;

RMGL_GI_7_t LobMode_data = {0};

RMGL_GI_rm_t delete_data;

extern cap_data_t cap_data;
*/

#define UI_HUART REFEREE_UART_HANDLE
#define UI_UART_SEND uart_send_async
extern UART_HandleTypeDef UI_HUART;
static uint8_t tx_buffer[128];

#define Frame_x GraphicMode_list[GraphicMode][0]
#define Frame_y GraphicMode_list[GraphicMode][1]
#define Frame_AspectRatio (Frame_x / Frame_y)
uint8_t GraphicMode = GraphicMode_1080p; // Unuseable
uint_fast16_t GraphicMode_list[5][2] =
{
	{640, 480},
	{1280, 720},
	{1600, 900},
	{1680, 1050},
	{1920, 1080}
};

void* GI_GQueue[RMGL_GraphicQueue_Depth] = {0};
uint8_t GI_GQueue_type[RMGL_GraphicQueue_Depth] = {0};
uint8_t GI_GQueue_pointer = 0;
uint8_t GI_GQueue_counter = 0;

#include "./base_drv/graphic_develop/gui_profile.h"

RMGL_GI_7_t RMUI_GI_7_0 =
{
	.header.data_cmd_id = 0x104
};

RMGL_GI_c_t RMUI_GI_c_0 =
{
	.header.data_cmd_id = 0x110
};

bool RMGL_GF_Register(RMGL_GF_t* pGF, void* pGE, uint8_t order, uint8_t type)
{
	if(pGF == NULL || pGE == NULL || order > GF_ElementsLimit) return 1;

	pGF->elements[order] = pGE;
	pGF->elements_type[order] = type;

	((RMGL_GE_Null_t*)pGE)->pfGF = pGF;

	return 0;
}

#define _pfGF ((RMGL_GF_t*)(pGE->pfGF))
bool RMGL_GI_Formatter(RMGL_GP_t* pGP, void* rpGE, uint8_t GE_type)
{
	if(pGP == NULL || rpGE == NULL) return 1;

	RMGL_GE_Null_t* pGE = (RMGL_GE_Null_t*)rpGE;

	pGP->GID[0] = RMGL_GID_Header;
	pGP->GID[1] = _pfGF->GFID;
	pGP->GID[2] = pGE->GEID;

	switch(pGE->status)
	{
		case GEstatus_Uninit:
			pGP->operate_type = Gop_Add;
		break;
		case GEstatus_Static:
			pGP->operate_type = Gop_Nop;
		break;
		case GEstatus_Dynamic:
			pGP->operate_type = Gop_Mod;
		break;
		case GEstatus_ToRemove:
			pGP->operate_type = Gop_Del;
		break;
		case GEstatus_ToDraw:
			pGP->operate_type = Gop_Add;
			break;
		default:
			pGP->operate_type = Gop_Nop;
	}

	if(pGE->private_layer == GE_pri_layer0)
		pGP->layer = 0;
	else if(pGE->private_layer == 0)
		pGP->layer = _pfGF->layer;
	else
		pGP->layer = pGE->private_layer;

	if(pGE->private_color == GEc_pri_RaB)
		pGP->color = GEc_Main_RaB;
	else if(pGE->private_color == 0)
		pGP->color = _pfGF->color;
	else
		pGP->color = pGE->private_color;

	pGP->width = pGE->width;

	switch(GE_type)
	{
		case GE_Line:
		{
			RMGL_GE_Line_t* pGE_Line = rpGE;

			pGP->graphic_type = 0;

			pGP->start_x = (pGE_Line->start_x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Line->start_y + _pfGF->y)* Frame_y;
			pGP->end_x = (pGE_Line->end_x + _pfGF->x)* Frame_x;
			pGP->end_y = (pGE_Line->end_y + _pfGF->y)* Frame_y;
		}
		break;
		case GE_Square:
		{
			RMGL_GE_Square_t* pGE_Square = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Square->start_x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Square->start_y + _pfGF->y)* Frame_y;
			pGP->end_x = (pGE_Square->end_x + _pfGF->x)* Frame_x;
			pGP->end_y = (pGE_Square->end_y + _pfGF->y)* Frame_y;
		}
		break;
		case GE_Circle:
		{
			RMGL_GE_Circle_t* pGE_Circle = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Circle->x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Circle->y + _pfGF->y)* Frame_y;
			pGP->radius = pGE_Circle->radius * Frame_x;
		}
		break;
		case GE_Ellipse:
		{
			RMGL_GE_Ellipse_t* pGE_Ellipse = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Ellipse->x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Ellipse->y + _pfGF->y)* Frame_y;
			pGP->end_x = pGE_Ellipse->x_halflen * Frame_x;
			pGP->end_y = pGE_Ellipse->y_halflen * Frame_y;
		}
		break;
		case GE_Arc:
		{
			RMGL_GE_Arc_t* pGE_Arc = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Arc->x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Arc->y + _pfGF->y)* Frame_y;
			pGP->end_x = pGE_Arc->x_halflen * Frame_x;
			pGP->end_y = pGE_Arc->y_halflen * Frame_y;

			pGP->start_angle = pGE_Arc->start_angle;
			pGP->end_angle = pGE_Arc->end_angle;
		}
		break;
		case GE_Float:
		{
			RMGL_GE_Float_t* pGE_Float = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Float->x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Float->y + _pfGF->y)* Frame_y;

			pGP->start_angle = pGE_Float->csize;

			pGP->end_angle = pGE_Float->digital;

			pGP->value = pGE_Float->value * 1000.f;
		}
		break;
		case GE_Int:
		{
			RMGL_GE_Int_t* pGE_Int = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Int->x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Int->y + _pfGF->y)* Frame_y;

			pGP->start_angle = pGE_Int->csize;

			pGP->value = pGE_Int->value;
		}
		break;
		case GE_Chars:
		{
			RMGL_GE_Chars_t* pGE_Chars = rpGE;

			pGP->graphic_type = GE_type;

			pGP->start_x = (pGE_Chars->x + _pfGF->x)* Frame_x;
			pGP->start_y = (pGE_Chars->y + _pfGF->y)* Frame_y;

			pGP->start_angle = pGE_Chars->csize;
			pGP->end_angle = strlen(pGE_Chars->text);
		}
		break;
		default: ;
	}

	return 0;
}

bool RMGL_GF_Init(RMGL_GF_t* pGF)
{
	uint8_t tx_num = 0;
	uint8_t p = 0;

	while(p < GF_ElementsLimit-1)
	{
		if(pGF->elements_type[p] == GE_Chars)
		p++, tx_num++;
		else
		{
			if(pGF->elements_type[p+1] == GE_Chars)
				p+=2, tx_num+=2;
			else
				p++;
		}
	}
	tx_num++;

	if(RMGL_GI_HangUp(pGF)) return 1;

	for(uint_fast8_t i=0; i<tx_num; i++)
	{
		RMGL_GI_Tunnel();
		HAL_Delay(200);
	}

	return 0;
}

void RMGL_GF_setColor(RMGL_GF_t* pGF, uint8_t color)
{
	pGF->color = color;

	for(uint_fast8_t i = 0; i <GF_ElementsLimit; i++)
		if(pGF->elements_type[i] != GE_Null)
			((RMGL_GE_Null_t*)(pGF->elements[i]))->status = GEstatus_Dynamic;
}

void RMGL_GF_setLayer(RMGL_GF_t* pGF, uint8_t layer)
{

}

void RMGL_GF_setCoord(RMGL_GF_t* pGF, float x, float y)
{
	pGF->x = x;
	pGF->y = y;

	for(uint_fast8_t i = 0; i <GF_ElementsLimit; i++)
		if(pGF->elements_type[i] != GE_Null)
			((RMGL_GE_Null_t*)(pGF->elements[i]))->status = GEstatus_Dynamic;
}

void RMGL_GF_setFlagStatus(RMGL_GF_t* pGF, uint8_t status)
{
	for(uint_fast8_t i = 0; i <GF_ElementsLimit; i++)
		if(pGF->elements_type[i] != GE_Null)
			((RMGL_GE_Null_t*)(pGF->elements[i]))->status = status;
}

void RMGL_GF_setGEColor(RMGL_GF_t* pGF, uint8_t order, uint8_t color)
{

}

void RMGL_GF_setGELayer(RMGL_GF_t* pGF, uint8_t order, uint8_t layer)
{

}

bool RMGL_GI_HangUp(RMGL_GF_t * pGF)
{
	if(GI_GQueue_counter >= RMGL_GraphicQueue_Depth) return 1;

	uint8_t GI_GQueue_tail;

	for(uint_fast8_t i = 0; i<GF_ElementsLimit; i++)
	{
		if(pGF->elements_type[i] == GE_Null || ((RMGL_GE_Null_t*)(pGF->elements[i]))->status == GEstatus_Static) continue;

		GI_GQueue_tail = GI_GQueue_pointer + GI_GQueue_counter;
		while(GI_GQueue_tail >= RMGL_GraphicQueue_Depth) GI_GQueue_tail -= RMGL_GraphicQueue_Depth;

		GI_GQueue[GI_GQueue_tail] = pGF->elements[i];
		GI_GQueue_type[GI_GQueue_tail] = pGF->elements_type[i];

		GI_GQueue_counter++;
		if(GI_GQueue_counter >= RMGL_GraphicQueue_Depth) return 1;
	}

	return 0;
}
void RMGL_GI_Tunnel(void)
{
	if(GI_GQueue_counter)
	{
	
		memset(&RMUI_GI_c_0.grapic_data_struct,0,sizeof(RMUI_GI_c_0.grapic_data_struct));
		memset(RMUI_GI_c_0.data,0,sizeof(RMUI_GI_c_0.data));

		if(GI_GQueue_type[GI_GQueue_pointer] == GE_Chars)
		{
			RMGL_GI_Formatter(&RMUI_GI_c_0.grapic_data_struct, GI_GQueue[GI_GQueue_pointer], GI_GQueue_type[GI_GQueue_pointer]);
			memcpy(RMUI_GI_c_0.data, ((RMGL_GE_Chars_t*)(GI_GQueue[GI_GQueue_pointer]))->text, 30);

			if(((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status == GEstatus_Uninit ||
			   ((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status == GEstatus_ToRemove ||
			   ((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status == GEstatus_ToDraw)
				((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status = GEstatus_Static;

			GI_GQueue_counter--, GI_GQueue_pointer++;
			while(GI_GQueue_pointer >= RMGL_GraphicQueue_Depth) GI_GQueue_pointer -= RMGL_GraphicQueue_Depth;

			UI_UART_SEND(&UI_HUART, tx_buffer, fill_tx_buffer((uint8_t *)&RMUI_GI_c_0, sizeof(RMUI_GI_c_0)));
		}
		else
		{
			for(uint_fast8_t i = 0; i<GF_ElementsLimit; i++)
			{
				if(GI_GQueue_type[GI_GQueue_pointer] == GE_Chars || GI_GQueue_counter == 0) break;

				RMGL_GI_Formatter(&RMUI_GI_7_0.grapic_data_struct[i], GI_GQueue[GI_GQueue_pointer], GI_GQueue_type[GI_GQueue_pointer]);

				if(((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status == GEstatus_Uninit ||
				   ((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status == GEstatus_ToRemove ||
				   ((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status == GEstatus_ToDraw)
					((RMGL_GE_Null_t*)(GI_GQueue[GI_GQueue_pointer]))->status = GEstatus_Static;

				GI_GQueue_counter--, GI_GQueue_pointer++;
				while(GI_GQueue_pointer >= RMGL_GraphicQueue_Depth) GI_GQueue_pointer -= RMGL_GraphicQueue_Depth;
			}

			UI_UART_SEND(&UI_HUART, tx_buffer, fill_tx_buffer((uint8_t *)&RMUI_GI_7_0, sizeof(RMUI_GI_7_0)));
		}
	}

}

void update_GUI(void)
{
	static uint8_t uinit = 1;

	if(uinit)
	{
		RMUI_GI_7_0.header.sender_ID = game_robot_status.robot_id;
		RMUI_GI_7_0.header.receiver_ID = game_robot_status.robot_id + 0x100;
		RMUI_GI_c_0.header.sender_ID = game_robot_status.robot_id;
		RMUI_GI_c_0.header.receiver_ID = game_robot_status.robot_id + 0x100;

		uinit = 0;

		// RMGL_GF_Register(&test_UI_GF, &test_UI_Line1, 0, GE_Line);
		// RMGL_GF_Register(&test_UI_GF, &test_UI_Line2, 1, GE_Line);
		// RMGL_GF_Register(&test_UI_GF, &test_UI_Float, 2, GE_Float);
		// RMGL_GF_Register(&test_UI_GF, &test_UI_Chars, 3, GE_Chars);
		// RMGL_GF_Register(&test_UI_GF, &test_UI_Chars2, 4, GE_Chars);
		// RMGL_GF_Register(&test_UI_GF, &test_UI_Line3, 5, GE_Line);

		// RMGL_GF_Init(&test_UI_GF);

		UI_Sight1_Init();
		UI_Sight2_Init();
		UI_Sight3_Init();

		UI_Cap_Shoot_Init();
		UI_Status_Init();
		UI_Posture_Init();
		UI_Status_String_Init();
	}
	else
	{
		RMGL_GI_HangUp(&UI_Sight1);
		
		RMGL_GI_Tunnel();
	}
}