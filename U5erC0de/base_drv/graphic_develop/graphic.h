#ifndef __GRAPHIC__
#define __GRAPHIC__

#include "./base_drv/graphic_develop/graphic_base.h"
#include "./base_drv/graphic_develop/RMGL_def.h"

#include <stdbool.h>

#define RMGL_GraphicQueue_Depth 49

RMGLAPI bool RMGL_GF_Register(RMGL_GF_t* pGF, void* pGE, uint8_t order, uint8_t type);
RMGLAPI bool RMGL_GI_Formatter(RMGL_GP_t* pGP, void* rpGE, uint8_t GE_type);
RMGLAPI bool RMGL_GF_Init(RMGL_GF_t* pGF);
RMGLAPI void RMGL_GF_setColor(RMGL_GF_t* pGF, uint8_t color);
RMGLAPI void RMGL_GF_setLayer(RMGL_GF_t* pGF, uint8_t layer);
RMGLAPI void RMGL_GF_setCoord(RMGL_GF_t* pGF, float x, float y);
RMGLAPI void RMGL_GF_setFlagStatus(RMGL_GF_t* pGF, uint8_t status);
RMGLAPI void RMGL_GF_setGEColor(RMGL_GF_t* pGF, uint8_t order, uint8_t color);
RMGLAPI void RMGL_GF_setGELayer(RMGL_GF_t* pGF, uint8_t order, uint8_t layer);
RMGLAPI bool RMGL_GI_HangUp(RMGL_GF_t * pGF);
RMGLAPI void RMGL_GI_Tunnel(void);

void update_GUI(void);
#endif
