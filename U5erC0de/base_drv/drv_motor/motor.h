#ifndef _motor_h_
#define _motor_h_

#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "./algorithm/util.h"
#include "./algorithm/pid.h" //控制算法
#include "./algorithm/pid_leso.h"
#include "./algorithm/pid_ladrc.h"
#include "./algorithm/filter.h"				 //滤波
#include "./base_drv/drv_motor/motor_lib.h"	 //电机类型
#include "./base_drv/drv_motor/motor_ctrl.h" //收发数据

typedef enum
{
	single_pid_ctrl = 0,
	dual_pid_ctrl,
	dual_pid_leso_ctrl,
	dual_ladrc_ctrl,
	single_adrc_ctrl,
} ctrl_mode_t;

typedef struct
{
	bool is_offline;
	float refresh_freq;
	motor_type_t motor_type; // 电机型号
	ctrl_mode_t ctrl_mode;
	union
	{
		pid_struct_t pid[2];
		pid_leso_t pid_leso;
		ladrc_para_t ladrc;
		bool adrc[2]; // ADRC adrc[2];
	};
	motor_data_t real; // real motor data
	motor_data_t expt; // expect motor data
	union
	{
		LPF_t lpf_fltr;
	};
	int output;
} motor_t;

#define rx_stdid_of(_motor_) _motor_.motor_type.rx_base_ID + _motor_.motor_type.offset_ID

// 通用电机初始化函数
void motor_init(
	motor_t *p_motor,
	DJI_motor_model_t motor_model,
	uint8_t ID,
	void *param_k_pos,
	void *param_k_vel);

void set_motor_mid_position(motor_t *p_motor, uint16_t mid_scale);
#endif
