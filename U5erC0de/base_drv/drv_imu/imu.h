#ifndef _imu_h_
#define _imu_h_
#include "./base_drv/drv_conf.h"
#include gimbal_module_core_h
#include "./base_drv/drv_imu/drv_bmi088.h"
#include "./base_drv/drv_imu/imu_bmi088.h"
#include "./base_drv/drv_imu/imu_fusion.h"
extern imu_data_fp_t imu_real_info;
extern eular_t _imu_eular;
void imu_init(void);
void imu_loop(void);
#endif
