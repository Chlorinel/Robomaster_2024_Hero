#ifndef __RMGL_DEF__
#define __RMGL_DEF__

#include <stdint.h>

//#include "preprocess/lib_macro.h"

#define RMGLAPI

#define RMGL_GID_Header 0x00

#if	defined (__CC_ARM) || (__ARMCC_VERSION)
	#define PACKED __packed
#else
	#define PACKED
	#pragma pack(1)
#endif

typedef PACKED union _int32_10b11b11T
{
	PACKED struct
	{
		uint32_t a:10;
		uint32_t b:11;
		uint32_t c:11;
	};

	uint32_t i;
}int32_10b11b11T;

// Graphic pack define
typedef PACKED struct _RMGL_GP_Typedef
{
	uint8_t GID[3];

	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;

	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;

	PACKED union
	{
		PACKED struct
		{
			uint32_t radius:10;
			uint32_t end_x:11;
			uint32_t end_y:11;
		};
		int32_t value;
	};
}RMGL_GP_t ;

typedef PACKED struct _RMGL_GP_header_Typedef
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}RMGL_GP_header_t;

// Graphical interface define
typedef PACKED struct _RMGL_GI_rm_Typedef
{
	RMGL_GP_header_t header;
	uint8_t operate_type;
	uint8_t layer;
}RMGL_GI_rm_t;
typedef PACKED struct _RMGL_GI_s_Typedef
{
	RMGL_GP_header_t header;
	RMGL_GP_t grapic_data_struct;
}RMGL_GI_s_t;

typedef PACKED struct _RMGL_GI_d_Typedef
{
	RMGL_GP_header_t header;
	RMGL_GP_t grapic_data_struct[2];
}RMGL_GI_d_t;

typedef PACKED struct _RMGL_GI_5_Typedef
{
	RMGL_GP_header_t header;
	RMGL_GP_t grapic_data_struct[5];
}RMGL_GI_5_t;

typedef PACKED struct _RMGL_GI_7_Typedef
{
	RMGL_GP_header_t header;
	RMGL_GP_t grapic_data_struct[7];
}RMGL_GI_7_t;

typedef PACKED struct _RMGL_GI_c_Typedef
{
	RMGL_GP_header_t header;
	RMGL_GP_t grapic_data_struct;
	uint8_t data[30];
}RMGL_GI_c_t;

// Graphic elements define
typedef PACKED struct _RMGL_GE_Null // This element type shouldnot be used
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;
}RMGL_GE_Null_t;

typedef PACKED struct _RMGL_GE_Null2 // This element type shouldnot be used
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;

	uint16_t angle1_csize;
	uint16_t angle2_clen;

	uint16_t x1;
	uint16_t y1;

	PACKED union
	{
		PACKED struct
		{
			uint32_t radius:10;
			uint32_t x2:11;
			uint32_t y2:11;
		};

		int32_t value;
	};
}RMGL_GE_Null2_t;

typedef PACKED struct _RMGL_GE_Line
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;
	float start_x;
	float start_y;
	float end_x;
	float end_y;
}RMGL_GE_Line_t;

typedef PACKED struct _RMGL_GE_Square
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;
	float start_x;
	float start_y;
	float end_x;
	float end_y;
}RMGL_GE_Square_t;

typedef PACKED struct _RMGL_GE_Circle
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;
	float x;
	float y;
	float radius;
}RMGL_GE_Circle_t;

typedef PACKED struct _RMGL_GE_Ellipse
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;
	float x;
	float y;
	float x_halflen;
	float y_halflen;
}RMGL_GE_Ellipse_t;

typedef PACKED struct _RMGL_GE_Arc
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;
	float x;
	float y;
	float x_halflen;
	float y_halflen;

	uint16_t start_angle;
	uint16_t end_angle;
}RMGL_GE_Arc_t;

typedef PACKED struct _RMGL_GE_Float
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;

	uint16_t csize;
	uint8_t digital;

	float x;
	float y;
	float value;
}RMGL_GE_Float_t;

typedef PACKED struct _RMGL_GE_Int
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;

	uint16_t csize;

	float x;
	float y;
	int32_t value;
}RMGL_GE_Int_t;

typedef PACKED struct _RMGL_GE_Chars
{
	void* pfGF;
	uint8_t GEID;

	uint8_t private_layer;
	uint8_t private_color;

	uint8_t status;

	uint16_t width;

	uint16_t csize;

	float x;
	float y;

	char text[30];
}RMGL_GE_Chars_t;

// Graphic fragment define
#define GF_ElementsLimit 7
typedef PACKED struct _RMGL_GF_Typedef
{
	uint8_t GFID;
	uint8_t color;
	uint8_t layer;
	float x;
	float y;
	uint8_t elements_type[GF_ElementsLimit];
	void* elements[GF_ElementsLimit];
}RMGL_GF_t;

// #define RMGL_GF_x_t(n) \
// typedef PACKED struct _RMGL_GF_##n##_Typedef \
// { \
// 	uint8_t GFID; \
// 	uint8_t color; \
// 	uint8_t layer; \
// 	float x; \
// 	float y; \
// 	uint8_t elements_type[n]; \
// 	uint8_t elements_status[n]; \
// 	void* elements[n]; \
// }RMGL_GF_##n##_t;
// _Macro_For(RMGL_GF_x_t, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14);
// #define RMGL_GF_t(n) RMGL_GF_##n##_t

#if	!defined (__CC_ARM) || (__ARMCC_VERSION)
	#pragma pack()
#endif

#define GE_pri_layer0 0xFF
#define GEc_pri_RaB (uint8_t)-1

enum
{
	GraphicMode_480p = 0,
	GraphicMode_720p,
	GraphicMode_900p,
	GraphicMode_1050p,
	GraphicMode_1080p,
};

enum
{
	Gop_Nop = 0,
	Gop_Add,
	Gop_Mod,
	Gop_Del
};

enum
{
	GEstatus_Uninit = 0,
	GEstatus_Static,
	GEstatus_Dynamic,
	GEstatus_ToRemove,
	GEstatus_ToDraw
};

enum
{
	GE_Line = 0xFF,
	GE_Null = 0,
	GE_Square,
	GE_Circle,
	GE_Ellipse,
	GE_Arc,
	GE_Float,
	GE_Int,
	GE_Chars
};

enum
{
	GEc_Main_RaB = 0,
	GEc_yellow,
	GEc_green,
	GEc_orange,
	GEc_purple,
	GEc_pink,
	GEc_cyan,
	GEc_black,
	GEc_white
};

#endif
