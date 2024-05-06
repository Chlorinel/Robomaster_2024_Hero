#ifndef _motor_lib_h_
#define _motor_lib_h_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE

typedef struct
{
	uint16_t measure_min;  // Lower limit of encoder output value
	uint16_t measure_max;  // Upper limit of encoder output value
	float reduction_ratio; // reduction_ratio
	uint16_t tx_base_ID;   // Identifier of the message received by the motor (sent by us)
	uint16_t rx_base_ID;   // Identifier of motor feedback (we receive) message
	uint16_t offset_ID;	   // rx_hander.StdId=rx_base_ID+offset_ID
						   // offset_ID number starts from 1
} motor_type_t;

typedef struct
{
	motor_type_t H;
	motor_type_t L;
} _Motor;

typedef enum
{
	motor_DJI_M2006 = 0,
	motor_DJI_M3508,
	motor_DJI_GM6020,
} DJI_motor_model_t;
extern const _Motor DJI_motor_lib_M2006;
extern const _Motor DJI_motor_lib_M3508;
extern const _Motor DJI_motor_lib_GM6020;
#endif
