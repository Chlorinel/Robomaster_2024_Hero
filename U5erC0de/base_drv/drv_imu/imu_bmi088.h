#ifndef _IMU_BMI088_H
#define _IMU_BMI088_H

/*bmi088 offical api support: https://github.com/BoschSensortec/BMI08x-Sensor-API */
/*download files and the following change include path*/
#include "drv_bmi088.h"
#include "bmi08x.h"

#include "imu_dcm.h"
#include "imu_calibrate.h"
#include "motion_fx_manager.h"

#define ACCEL_RANGE_G 3
#define GYRO_RANGE_DPS 1000

//import motion fx and motion_fx_manager.c first
#define USE_MOTION_FX 0

#define IMU_DATA_NOT_RDY -53
#define IMU_CONF_ERR -47
#define IMU_ACCEL_ERR -24

#define MFX_NUM_AXES    3
#define MFX_QNUM_AXES   4

#ifndef _MOTION_FX_H_
typedef struct
{
  float mag[MFX_NUM_AXES];                 /* Calibrated mag [uT/50] */
  float acc[MFX_NUM_AXES];                 /* Acceleration in [g] */
  float gyro[MFX_NUM_AXES];                /* Angular rate [dps] */
} MFX_input_t;
typedef struct
{
  float rotation[MFX_NUM_AXES];            /* yaw, pitch and roll */
  float quaternion[MFX_QNUM_AXES];         /* quaternion */
  float gravity[MFX_NUM_AXES];             /* device frame gravity */
  float linear_acceleration[MFX_NUM_AXES]; /* device frame linear acceleration */
  float heading;                           /* heading */
  float headingErr;                        /* heading error in deg */
} MFX_output_t;
#endif

void imu_init(void);
int8_t init_bmi08x(void);
int8_t enable_bmi08x_interrupt(void);
int8_t disable_bmi08x_interrupt(void);

int8_t get_imu_att(imu_data_fp_t* _imu_fp, eular_t* eular);
int8_t get_imu_data_raw(imu_data_raw_t* _imu, imu_data_fp_t* _imu_fp);

#if USE_MOTION_FX == 1
int8_t mfx_get_imu_att(imu_data_fp_t* _imu_fp, eular_t* eular);
#endif

#endif

