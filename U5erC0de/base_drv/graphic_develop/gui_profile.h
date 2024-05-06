//#include "./base_drv/graphic_develop/graphic_base.h"
#include "./base_drv/graphic_develop/RMGL_def.h"
enum
{
	GFLayer_Sight = 1,
	GFLayer_Status
};

#define Sight_x1_1 0.025f
#define Sight_x1_2 0.10f
#define Sight_x2_1 0.02f
#define Sight_x2_2 0.075f
#define Sight_x3 0.075f
#define Sight_x4 0.05f
#define Sight_x5 0.04f

#define Sight_y1 0.f
#define Sight_y2 -0.05f
#define Sight_y3 -0.1f
#define Sight_y4 -0.15f
#define Sight_y5 -0.20f
#define Sight_y6 -0.25f

#define Sight_m1 0.15f
#define Sight_m2 0.05f
#define Sight_m3 -0.05f
#define Sight_m4 -0.3f

RMGL_GF_t UI_Sight1 =
{
	.GFID = 0x01,

	.x = 0.5f,
	.y = 0.5f,

	.layer = GFLayer_Sight,

	.color = GEc_yellow
};

RMGL_GE_Line_t UI_Sight1_Line1 =
{
	.GEID = 0x01,

	.width = 1,

	.start_x = Sight_x1_1,
	.end_x = Sight_x1_2,
	.start_y = Sight_y1,
	.end_y = Sight_y1
};

RMGL_GE_Line_t UI_Sight1_Line2 =
{
	.GEID = 0x02,

	.width = 1,

	.start_x = -Sight_x1_1,
	.end_x = -Sight_x1_2,
	.start_y = Sight_y1,
	.end_y = Sight_y1
};

RMGL_GE_Line_t UI_Sight1_Line3 =
{
	.GEID = 0x03,

	.width = 1,

	.start_x = Sight_x2_1,
	.end_x = Sight_x2_2,
	.start_y = Sight_y2,
	.end_y = Sight_y2
};

RMGL_GE_Line_t UI_Sight1_Line4 =
{
	.GEID = 0x04,

	.width = 1,

	.start_x = -Sight_x2_1,
	.end_x = -Sight_x2_2,
	.start_y = Sight_y2,
	.end_y = Sight_y2
};

RMGL_GE_Line_t UI_Sight1_Line5 =
{
	.GEID = 0x05,

	.width = 1,

	.start_x = Sight_x4,
	.end_x = -Sight_x4,
	.start_y = Sight_y4,
	.end_y = Sight_y4
};

RMGL_GE_Line_t UI_Sight1_Line6 =
{
	.GEID = 0x06,

	.width = 1,

	.start_y = Sight_m1,
	.end_y = Sight_m2
};

RMGL_GE_Line_t UI_Sight1_Line7 =
{
	.GEID = 0x07,

	.width = 1,

	.start_y = Sight_m3,
	.end_y = Sight_m4
};

void UI_Sight1_Init(void)
{
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line1, 0, GE_Line);
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line2, 1, GE_Line);
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line3, 2, GE_Line);
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line4, 3, GE_Line);
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line5, 4, GE_Line);
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line6, 5, GE_Line);
	RMGL_GF_Register(&UI_Sight1, &UI_Sight1_Line7, 6, GE_Line);

	RMGL_GF_Init(&UI_Sight1);
}


RMGL_GF_t UI_Sight2 =
{
	.GFID = 0x02,

	.x = 0.5f,
	.y = 0.5f,

	.layer = GFLayer_Sight,

	.color = GEc_yellow
};

RMGL_GE_Line_t UI_Sight2_Line1 =
{
	.GEID = 0x01,

	.width = 1,

	.start_x = Sight_x2_1,
	.end_x = Sight_x2_2,
	.start_y = Sight_y3,
	.end_y = Sight_y3
};

RMGL_GE_Line_t UI_Sight2_Line2 =
{
	.GEID = 0x02,

	.width = 1,

	.start_x = -Sight_x2_1,
	.end_x = -Sight_x2_2,
	.start_y = Sight_y3,
	.end_y = Sight_y3
};

RMGL_GE_Line_t UI_Sight2_Line3 =
{
	.GEID = 0x03,

	.width = 1,

	.start_x = Sight_x2_1,
	.end_x = Sight_x2_2 * 0.75f,
	.start_y = (Sight_y1 + Sight_y2)/ 2.f,
	.end_y = (Sight_y1 + Sight_y2)/ 2.f
};

RMGL_GE_Line_t UI_Sight2_Line4 =
{
	.GEID = 0x04,

	.width = 1,

	.start_x = -Sight_x2_1,
	.end_x = -Sight_x2_2 * 0.75f,
	.start_y = (Sight_y1 + Sight_y2)/ 2.f,
	.end_y = (Sight_y1 + Sight_y2)/ 2.f
};

RMGL_GE_Line_t UI_Sight2_Line5 =
{
	.GEID = 0x05,

	.width = 1,

	.start_x = Sight_x2_1,
	.end_x = Sight_x2_2 * 0.75f,
	.start_y = (Sight_y2 + Sight_y3)/ 2.f,
	.end_y = (Sight_y2 + Sight_y3)/ 2.f
};

RMGL_GE_Line_t UI_Sight2_Line6 =
{
	.GEID = 0x06,

	.width = 1,

	.start_x = -Sight_x2_1,
	.end_x = -Sight_x2_2 * 0.75f,
	.start_y = (Sight_y2 + Sight_y3)/ 2.f,
	.end_y = (Sight_y2 + Sight_y3)/ 2.f
};

RMGL_GE_Line_t UI_Sight2_Line7 =
{
	.GEID = 0x07,

	.width = 1,

	.start_x = -Sight_x5,
	.end_x = Sight_x5,
	.start_y = Sight_y5,
	.end_y = Sight_y5
};

void UI_Sight2_Init(void)
{
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line1, 0, GE_Line);
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line2, 1, GE_Line);
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line3, 2, GE_Line);
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line4, 3, GE_Line);
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line5, 4, GE_Line);
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line6, 5, GE_Line);
	RMGL_GF_Register(&UI_Sight2, &UI_Sight2_Line7, 6, GE_Line);

	RMGL_GF_Init(&UI_Sight2);
}

RMGL_GF_t UI_Sight3 =
{
	.GFID = 0x03,

	.x = 0.5f,
	.y = 0.5f,

	.layer = GFLayer_Sight,

	.color = GEc_yellow
};

RMGL_GE_Line_t UI_Sight3_Line1 =
{
	.GEID = 0x01,

	.width = 1,

	.start_x = Sight_x2_1,
	.end_x = Sight_x4,
	.start_y = (Sight_y3 + Sight_y4)/ 2.f,
	.end_y = (Sight_y3 + Sight_y4)/ 2.f
};

RMGL_GE_Line_t UI_Sight3_Line2 =
{
	.GEID = 0x02,

	.width = 1,

	.start_x = -Sight_x2_1,
	.end_x = -Sight_x4,
	.start_y = (Sight_y3 + Sight_y4)/ 2.f,
	.end_y = (Sight_y3 + Sight_y4)/ 2.f
};

RMGL_GE_Line_t UI_Sight3_Line3 =
{
	.GEID = 0x03,

	.width = 1,

	.start_x = -Sight_x5,
	.end_x = Sight_x5,
	.start_y = Sight_y6,
	.end_y = Sight_y6
};

RMGL_GE_Line_t UI_Sight3_Line4 =
{
	.GEID = 0x04,

	.width = 1,

	.start_x = -Sight_x5,
	.end_x = Sight_x5,
	.start_y = (Sight_y4 + Sight_y5)/ 2.f,
	.end_y = (Sight_y4 + Sight_y5)/ 2.f
};

RMGL_GE_Line_t UI_Sight3_Line5 =
{
	.GEID = 0x05,

	.width = 1,

	.start_x = -Sight_x5,
	.end_x = Sight_x5,
	.start_y = (Sight_y5 + Sight_y6)/ 2.f,
	.end_y = (Sight_y5 + Sight_y6)/ 2.f
};

RMGL_GE_Float_t UI_Sight3_Distance =
{
	.GEID = 0x06,

	.width = 2,
	.digital = 1,
	.csize = 14,

	.private_color = GEc_orange,

	.x = 0.02f,
	.y = 0.12f,

	.value = 10.1f
};

RMGL_GE_Float_t UI_Sight3_Pitch =
{
	.GEID = 0x07,

	.width = 3,
	.digital = 2,
	.csize = 25,

	.x = 0.095f,
	.y = -0.01f,

	.value = 10.01f
};

void UI_Sight3_Init(void)
{
	RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Line1, 0, GE_Line);
	RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Line2, 1, GE_Line);
	RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Line3, 2, GE_Line);
	RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Line4, 3, GE_Line);
	RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Line5, 4, GE_Line);
	// RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Distance, 5, GE_Float);
	// RMGL_GF_Register(&UI_Sight3, &UI_Sight3_Pitch, 6, GE_Float);

	RMGL_GF_Init(&UI_Sight3);
}

#define Cap_r 0.325f
#define Cap_rdelta 0.01f

RMGL_GF_t UI_Cap_Shoot =
{
	.GFID = 0x04,

	.x = 0.5f,
	.y = 0.5f,

	.layer = GFLayer_Status,

	.color = GEc_yellow
};

RMGL_GE_Arc_t UI_Cap_Arc1 =
{
	.GEID = 0x01,

	.width = 1,

	.x_halflen = (Cap_r - Cap_rdelta) / 1.777777f,
	.y_halflen = (Cap_r - Cap_rdelta),

	.start_angle = 45,
	.end_angle = 135
};

RMGL_GE_Arc_t UI_Cap_Arc2 =
{
	.GEID = 0x02,

	.width = 1,

	.x_halflen = (Cap_r + Cap_rdelta) / 1.777777f,
	.y_halflen = (Cap_r + Cap_rdelta),

	.start_angle = 45,
	.end_angle = 135
};

RMGL_GE_Line_t UI_Cap_Line1 =
{
	.GEID = 0x03,

	.width = 1,

	.start_x = (Cap_r - Cap_rdelta) * 0.398f,
	.end_x = (Cap_r + Cap_rdelta) * 0.398f,
	.start_y = (Cap_r - Cap_rdelta) * 0.707f,
	.end_y = (Cap_r + Cap_rdelta) * 0.707f
};

RMGL_GE_Line_t UI_Cap_Line2 =
{
	.GEID = 0x04,

	.width = 1,

	.start_x = (Cap_r - Cap_rdelta) * 0.398f,
	.end_x = (Cap_r + Cap_rdelta) * 0.398f,
	.start_y = -(Cap_r - Cap_rdelta) * 0.707f,
	.end_y = -(Cap_r + Cap_rdelta) * 0.707f
};

RMGL_GE_Arc_t UI_Cap_Arc_Dym =
{
	.GEID = 0x05,

	.width = 12,
	.private_color = GEc_green,

	.x_halflen = Cap_r / 1.777777f,
	.y_halflen = Cap_r,

	.start_angle = 45 + 1,
	.end_angle = 135 - 1
};

#define Shootbarrier_size 0.035f
RMGL_GE_Circle_t UI_Shoot_Circle =
{
	.GEID = 0x06,

	.width = 8,

	.radius = Shootbarrier_size * 0.75f,

	.private_color = GEc_pink
};

RMGL_GE_Line_t UI_Shoot_Line =
{
	.GEID = 0x07,

	.width = 8,

	.start_x = -Shootbarrier_size / 2,
	.end_x = Shootbarrier_size / 2,
	.start_y = -Shootbarrier_size * 1.777777f / 2,
	.end_y = Shootbarrier_size * 1.777777f  / 2,

	.private_color = GEc_pink
};


void UI_Cap_Shoot_Init(void)
{
	RMGL_GF_Register(&UI_Cap_Shoot, &UI_Cap_Arc1, 0, GE_Arc);
	RMGL_GF_Register(&UI_Cap_Shoot, &UI_Cap_Arc2, 1, GE_Arc);
	RMGL_GF_Register(&UI_Cap_Shoot, &UI_Cap_Line1, 2, GE_Line);
	RMGL_GF_Register(&UI_Cap_Shoot, &UI_Cap_Line2, 3, GE_Line);
	RMGL_GF_Register(&UI_Cap_Shoot, &UI_Cap_Arc_Dym, 4, GE_Arc);
	// RMGL_GF_Register(&UI_Cap_Shoot, &UI_Shoot_Circle, 5, GE_Circle);
	// RMGL_GF_Register(&UI_Cap_Shoot, &UI_Shoot_Line, 6, GE_Line);

	RMGL_GF_Init(&UI_Cap_Shoot);
}

#define sShoot_x -0.18f
#define sShoot_y 0.f
#define sTankMode_x (sShoot_x * 0.866f)
#define sTankMode_y (sShoot_x * 0.5f * 1.777777f)
#define sLob_x (sShoot_x * 0.866f)
#define sLob_y (-sShoot_x * 0.5f * 1.777777f)
#define Status_r 0.008f

RMGL_GF_t UI_Status =
{
	.GFID = 0x05,

	.layer = GFLayer_Status,

	.x = 0.5f,
	.y = 0.5f,

	.color = GEc_yellow
};

RMGL_GE_Circle_t UI_Status_TankMode =
{
	.GEID = 0x01,

	.width = 14,

	.radius = Status_r * 0.4f,

	.x = sTankMode_x,
	.y = sTankMode_y,

	.private_color = GEc_orange
};

RMGL_GE_Circle_t UI_Status_TankMode_edge =
{
	.GEID = 0x02,

	.width = 2,

	.radius = Status_r,

	.x = sTankMode_x,
	.y = sTankMode_y
};

RMGL_GE_Circle_t UI_Status_Shoot =
{
	.GEID = 0x03,

	.width = 14,

	.radius = Status_r * 0.4f,

	.x = sShoot_x,
	.y = sShoot_y,

	.private_color = GEc_orange
};

RMGL_GE_Circle_t UI_Status_Shoot_edge =
{
	.GEID = 0x04,

	.width = 2,

	.radius = Status_r,

	.x = sShoot_x,
	.y = sShoot_y
};

RMGL_GE_Circle_t UI_Status_Lob =
{
	.GEID = 0x05,

	.width = 14,

	.radius = Status_r * 0.4f,

	.x = sLob_x,
	.y = sLob_y,

	.private_color = GEc_orange
};

RMGL_GE_Circle_t UI_Status_Lob_edge =
{
	.GEID = 0x06,

	.width = 2,

	.radius = Status_r,

	.x = sLob_x,
	.y = sLob_y
};

RMGL_GE_Arc_t UI_Status_Vision =
{
	.GEID = 0x07,

	.width = 2,

	.x_halflen = 0.05f,
	.y_halflen = 0.05f * 1.777777f,

	.start_angle = 260,
	.end_angle = 100,

	.private_color = GEc_green
};

void UI_Status_Init(void)
{
	RMGL_GF_Register(&UI_Status, &UI_Status_TankMode, 0, GE_Circle);
	RMGL_GF_Register(&UI_Status, &UI_Status_TankMode_edge, 1, GE_Circle);
	RMGL_GF_Register(&UI_Status, &UI_Status_Shoot, 2, GE_Circle);
	RMGL_GF_Register(&UI_Status, &UI_Status_Shoot_edge, 3, GE_Circle);
	RMGL_GF_Register(&UI_Status, &UI_Status_Lob, 4, GE_Circle);
	RMGL_GF_Register(&UI_Status, &UI_Status_Lob_edge, 5, GE_Circle);
	RMGL_GF_Register(&UI_Status, &UI_Status_Vision, 6, GE_Arc);

	RMGL_GF_Init(&UI_Status);
}

RMGL_GF_t UI_Status_String =
{
	.GFID = 0x07,

	.layer = GFLayer_Status,

	.x = 0.5f,
	.y = 0.5f,

	.color = GEc_yellow
};

RMGL_GE_Chars_t UI_Status_String_Base =
{
	.GEID = 0x01,

	.csize = 15,
	.width = 2,

	.x = -0.18f,
	.y = 0.11f,

	.text = "LOB\n\n\n\n\n  SHOOT\n\n\n\n\nTANK"
};

RMGL_GE_Chars_t UI_Status_String_AT =
{
	.GEID = 0x02,

	.csize = 12,
	.width = 2,

	.x = 0.15f,
	.y = -0.33f,

	.private_color = GEc_cyan,

	.text = "Anti-Targeting"
};

RMGL_GE_Chars_t UI_Status_String_Buff =
{
	.GEID = 0x03,

	.csize = 12,
	.width = 2,

	.x = 0.15f,
	.y = -0.375f,

	.private_color = GEc_cyan,

	.text = "LARGE BUFF\nSMALL BUFF"
};

void UI_Status_String_Init(void)
{
	RMGL_GF_Register(&UI_Status_String, &UI_Status_String_Base, 0, GE_Chars);
	RMGL_GF_Register(&UI_Status_String, &UI_Status_String_AT, 1, GE_Chars);
	RMGL_GF_Register(&UI_Status_String, &UI_Status_String_Buff, 2, GE_Chars);
	// RMGL_GF_Register(&UI_Status, &UI_Status_Shoot_edge, 3, GE_Circle);
	// RMGL_GF_Register(&UI_Status, &UI_Status_Lob, 4, GE_Circle);
	// RMGL_GF_Register(&UI_Status, &UI_Status_Lob_edge, 5, GE_Circle);
	// RMGL_GF_Register(&UI_Status, &UI_Status_Vision, 6, GE_Arc);

	RMGL_GF_Init(&UI_Status_String);
}

RMGL_GF_t UI_Posture =
{
	.GFID = 0x06,

	.layer = GFLayer_Status,

	.x = 0.575f,
	.y = 0.125f,

	.color = GEc_yellow
};

RMGL_GE_Square_t UI_Posture_Square =
{
	.GEID = 0x01,

	.width = 4,

	.start_x = -0.02f,
	.end_x = 0.02f,
	.start_y = 0.05f,
	.end_y = -0.05f
};

RMGL_GE_Circle_t UI_Posture_Circle =
{
	.GEID = 0x02,

	.width = 3,

	.radius = 0.012f
};

RMGL_GE_Line_t UI_Posture_Direction =
{
	.GEID = 0x03,

	.width = 4,

	.end_x = 0.f,
	.end_y = 0.09f,

	.private_color = GEc_cyan
};

void UI_Posture_Init(void)
{
	RMGL_GF_Register(&UI_Posture, &UI_Posture_Square, 0, GE_Square);
	RMGL_GF_Register(&UI_Posture, &UI_Posture_Circle, 1, GE_Circle);
	RMGL_GF_Register(&UI_Posture, &UI_Posture_Direction, 2, GE_Line);
	// RMGL_GF_Register(&UI_Posture, &UI_Status_Shoot_edge, 3, GE_Circle);
	// RMGL_GF_Register(&UI_Posture, &UI_Status_Lob, 4, GE_Circle);
	// RMGL_GF_Register(&UI_Posture, &UI_Status_Lob_edge, 5, GE_Circle);
	// RMGL_GF_Register(&UI_Posture, &UI_Status_Vision, 6, GE_Arc);

	RMGL_GF_Init(&UI_Posture);
}
