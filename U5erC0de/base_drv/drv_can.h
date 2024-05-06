#ifndef _DRV_CAN_H
#define _DRV_CAN_H

#include "drv_conf.h"
#include HAL_INCLUDE
#include "can.h"

HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef *hcan, uint32_t *FliterId, _Bool FIFO, uint8_t FilterNum);

#endif
