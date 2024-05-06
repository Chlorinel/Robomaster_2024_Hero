#ifndef _MOVE_CTRL_H_
#define _MOVE_CTRL_H_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include chassis_module_core_h
#include chassis_module_motor_ctrl_h

chassis_state_t *mecanum_forward(void);
float get_four_wheeler_wz(
    chassis_state_t *gimbal_expt_state,
    chassis_state_t *chassis_real_state);
typedef struct
{
    float LF;
    float LB;
    float RB;
    float RF;
} wheel_eta_t;
#endif
