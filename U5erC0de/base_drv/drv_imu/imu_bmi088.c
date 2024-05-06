#include "imu_bmi088.h"
#include "./algorithm/filter.h"
#include <string.h>

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;

/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

#define SYNC_ACCEL_RDY 0
static imu_acc_cal_t acc_cali = {
    .accel_T = {{1.009886, -0.000258, -0.007465},
                {0.01008, 1.009024, 0.000240},
                {-0.000451, 0.005957, 1.005738}},
    .accel_offs = {-34.419434, -25.750000, 78.289063}};

imu_gyro_cal_t gyro_cali = {

    .gx_bias = 5.12838354, .gy_bias = 1.01459844, .gz_bias = 1.50401866,
    // set tco k=0 to disable tempture compensate
    //	.gx_tco_k = 0.14753359f,
    //	.gy_tco_k = -0.27116829f,
    //	.gz_tco_k = -0.26021563f,
    //
    //	.gx_tco_b0 = -10.01137403f,
    //	.gy_tco_b0 = -3.91955357f,
    //	.gz_tco_b0 = 11.72579286f,

    // 	.accel_T = {{1.006753 ,-0.023650 ,0.045106},
    // 				{0.032397 ,1.009002 ,0.022700},
    // 				{-0.021697 ,0.021613 ,1.004586}},
    // 	.accel_offs = {-67.020996 ,18.620117 ,52.804688}};

    // imu_gyro_cal_t gyro_cali = {
    // 	.gx_bias =	7.6097765,//7.48922157,//7.1216197,//7.4435153
    // //7.59471798
    // //7.88310528 	.gy_bias = 1.88734686,//1.71643579,//0.782712996,
    // //-1.45880222 //-1.18114698	//-0.690207541 	.gz_bias =
    // 0.878592253,//0.874109507,//1.01514244,	//2.63865829 	//2.30805779
    // //1.73564923
};

dcm_init_t init_para = {.state = NULL,
                        .covariance = NULL,
                        .init_dcm_variance = DEFAULT_Q_DCM2_INIT,
                        .init_bias_variance = DEFAULT_Q_GYRO_BIAS2_INIT};

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return BMI08X_OK if all process is ok
 *
 */
int8_t init_bmi08x(void) {
  int8_t rslt;

  rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_SPI_INTF, BMI088_VARIANT);

  if (rslt == BMI08X_OK) {
    rslt = bmi08a_soft_reset(&bmi08xdev);
    if (rslt == BMI08X_OK) {
      rslt = bmi08a_init(&bmi08xdev);
    }
  }
  // bmi08x_error_codes_print_result("bmi08a_init", rslt);

  // if (rslt == BMI08X_OK)
  //   {
  //       //printf("Uploading config file !\n");
  // mandatory step only for data synchronization
  //       rslt = bmi08a_load_config_file(&bmi08xdev);
  //       //bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);
  //   }

  if (rslt == BMI08X_OK) {
    rslt = bmi08g_soft_reset(&bmi08xdev);
    if (rslt == BMI08X_OK) {
      rslt = bmi08g_init(&bmi08xdev);
    }
    // bmi08x_error_codes_print_result("bmi08g_init", rslt);
  }

  bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;
  bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
  bmi08xdev.accel_cfg.power =
      BMI08X_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power];
                               */
  bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_OSR4;

  bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
  bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
  bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
  bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

  if (rslt == BMI08X_OK) {

    rslt = bmi08a_set_power_mode(&bmi08xdev);
    if (rslt != BMI08X_OK)
      return rslt;
    // bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);
    rslt = bmi08a_set_meas_conf(&bmi08xdev);
    if (rslt != BMI08X_OK)
      return rslt;
    // bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

    rslt = bmi08a_get_power_mode(&bmi08xdev);
    if (rslt != BMI08X_OK ||
        bmi08xdev.accel_cfg.power != BMI08X_ACCEL_PM_ACTIVE)
      return IMU_CONF_ERR;

    rslt = bmi08g_set_power_mode(&bmi08xdev);
    if (rslt != BMI08X_OK)
      return rslt;
    // bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);

    rslt = bmi08g_set_meas_conf(&bmi08xdev);
    if (rslt != BMI08X_OK)
      return rslt;

    rslt = bmi08a_get_power_mode(&bmi08xdev);
    if (rslt != BMI08X_OK || bmi08xdev.gyro_cfg.power != BMI08X_GYRO_PM_NORMAL)
      return IMU_CONF_ERR;
    // bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
  }
  struct bmi08x_err_reg err;

  rslt = bmi08a_get_error_status(&err, &bmi08xdev);
  if (rslt == BMI08X_OK) {
    if (err.err_code != 0 || err.fatal_err != 0) {
      return IMU_ACCEL_ERR;
    }
  }

  init_imu_dcm(&init_para);

  return rslt;
}

/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return BMI08X_OK if all process is ok
 *
 */
int8_t enable_bmi08x_interrupt(void) {
  int8_t rslt;
  uint8_t data = 0;

  /* Set accel interrupt pin configuration */
  accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
  accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
  accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
  accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
  accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

  /* Enable accel data ready interrupt channel */
  rslt = bmi08a_set_int_config(
      (const struct bmi08x_accel_int_channel_cfg *)&accel_int_config,
      &bmi08xdev);
  // bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

  if (rslt == BMI08X_OK) {
    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
    gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /* Enable gyro data ready interrupt channel */
    rslt = bmi08g_set_int_config(
        (const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config,
        &bmi08xdev);
    // bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);

    rslt =
        bmi08g_get_regs(BMI08X_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi08xdev);
    // bmi08x_error_codes_print_result("bmi08g_get_regs", rslt);
  }

  return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return BMI08X_OK if all process is ok
 *
 */
int8_t disable_bmi08x_interrupt(void) {
  int8_t rslt;

  /* Set accel interrupt pin configuration */
  accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
  accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
  accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
  accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
  accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

  /* Disable accel data ready interrupt channel */
  rslt = bmi08a_set_int_config(
      (const struct bmi08x_accel_int_channel_cfg *)&accel_int_config,
      &bmi08xdev);
  // bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

  if (rslt == BMI08X_OK) {
    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
    gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

    /* Disable gyro data ready interrupt channel */
    rslt = bmi08g_set_int_config(
        (const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config,
        &bmi08xdev);
    // bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
  }
  return rslt;
}

/*!
 *  @brief get bmi088 raw data
 *  @param[in] imu_data_raw_t* _imu: pointer to raw imu data output
 *  @param[in] imu_data_fp_t* _imu_fp : pointer to imu data output in float
 * point struct
 *  @return BMI08X_OK if all process is ok
 */
int8_t get_imu_data_raw(imu_data_raw_t *_imu, imu_data_fp_t *_imu_fp) {
  int8_t rslt;
  int32_t temp;
  float real_tempture;

  rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
  if (rslt == BMI08X_OK) {
    rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
  }
  if (rslt == BMI08X_OK) {
    if (_imu != NULL) {
      //_imu->ax = bmi08x_accel.x; _imu->ay = bmi08x_accel.y; _imu->ay =
      // bmi08x_accel.y; _imu->gx = bmi08x_gyro.x; _imu->gy = bmi08x_gyro.z;
      //_imu->gz = bmi08x_gyro.z;
      memcpy(&(_imu->ax), &(bmi08x_accel.x), sizeof(short) * 3);
      memcpy(&(_imu->gx), &(bmi08x_gyro.x), sizeof(short) * 3);
      if (_imu_fp != NULL) {
        rslt = bmi08a_get_sensor_temperature(&bmi08xdev, &temp);
        if (rslt == BMI08X_OK) {
          real_tempture = ((float)temp) / 1000.0f;
          *_imu_fp = imu_correct(_imu, &acc_cali, &gyro_cali, real_tempture);
        }
      }
    }
  }
  return rslt;
}

/*!
 *  @brief parse bmi088 data and output
 *  @param[in] imu_data_fp_t* _imu_fp : pointer to imu data float point struct
 *  @param[in] eular_t* eular : pointer to eular output struct
 *  @return BMI08X_OK if all process is ok
 */
int8_t get_imu_att(imu_data_fp_t *_imu_fp, eular_t *eular) {
  int8_t rslt = 0;
  uint32_t delta_tick;
  int32_t temp;
  float real_tempture;

  static uint32_t sensor_time, last_sensor_time;
  imu_data_raw_t imu_raw;

  uint8_t gyro_status = 0;
#if SYNC_ACCEL_RDY == 1
  uint8_t accel_status = 0 rslt |=
      bmi08a_get_data_int_status(&accel_status, &bmi08xdev);
#endif
  rslt |= bmi08g_get_data_int_status(&gyro_status, &bmi08xdev);

  if (rslt == BMI08X_OK
#if SYNC_ACCEL_RDY == 1
      && (accel_status & BMI08X_ACCEL_DATA_READY_INT)
#endif
      && (gyro_status & BMI08X_GYRO_DATA_READY_INT)) {

    last_sensor_time = sensor_time;
    rslt = bmi08a_get_sensor_time(&bmi08xdev, &sensor_time);
    if (rslt == BMI08X_OK) {
      rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
    }
    if (rslt == BMI08X_OK) {
      rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
    }
    if (rslt == BMI08X_OK) {
      memcpy(&(imu_raw.ax), &(bmi08x_accel.x), sizeof(short) * 3);
      memcpy(&(imu_raw.gx), &(bmi08x_gyro.x), sizeof(short) * 3);
      // imu_raw.ax = bmi08x_accel.y; imu_raw.ay = bmi08x_accel.x; imu_raw.az =
      // -bmi08x_accel.z; imu_raw.gx = bmi08x_gyro.y; imu_raw.gy =
      // bmi08x_gyro.x; imu_raw.gz = -bmi08x_gyro.z;
      if (last_sensor_time > sensor_time) {
        delta_tick = 0x00FFFFFF - (last_sensor_time - sensor_time);
      } else {
        delta_tick = sensor_time - last_sensor_time;
      }
      if (_imu_fp != NULL) {
        imu_raw.a_fullscale = ACCEL_RANGE_G * 9.7833f;
        imu_raw.g_fullscale = GYRO_RANGE_DPS * PI / 180.0f;

        rslt = bmi08a_get_sensor_temperature(&bmi08xdev, &temp);
        if (rslt == BMI08X_OK) {
          real_tempture = ((float)temp) / 1000.0f;
          *_imu_fp =
              imu_correct(&imu_raw, &acc_cali, &gyro_cali, real_tempture);

          //					mahony_imu(_imu_fp,&gyro_cali,delta_tick/(float)25600);
          //					if(eular != NULL)
          // get_imu_eular(eular);

          imu_dcm_update(_imu_fp, &gyro_cali, delta_tick / (float)25600);
          if (eular != NULL)
            imu_dcm_get_eular(eular);
        }
      }
      bmi08x_delay_us(400, 0);
    }
  } else
    return IMU_DATA_NOT_RDY;
  return rslt;
}

/*!
 *  @brief parse bmi088 data and output using ST Lib(MotionFX)
 *  @param[in] imu_data_fp_t* _imu_fp : pointer to imu data float point struct
 *  @param[in] eular_t* eular : pointer to eular output struct
 *  @return BMI08X_OK if all process is ok
 */
int8_t mfx_get_imu_att(imu_data_fp_t *_imu_fp, eular_t *eular) {
  int8_t rslt = 0;
  static uint32_t sensor_time, last_sensor_time;
  uint32_t delta_tick;
  // imu_data_raw_t imu_raw;
  MFX_input_t imu_in;
  MFX_output_t imu_out;

  uint8_t gyro_status = 0;
#if SYNC_ACCEL_RDY == 1
  uint8_t accel_status = 0 rslt |=
      bmi08a_get_data_int_status(&accel_status, &bmi08xdev);
#endif
  rslt |= bmi08g_get_data_int_status(&gyro_status, &bmi08xdev);

  if (rslt == BMI08X_OK
#if SYNC_ACCEL_RDY == 1
      && (accel_status & BMI08X_ACCEL_DATA_READY_INT)
#endif
      && (gyro_status & BMI08X_GYRO_DATA_READY_INT)) {

    last_sensor_time = sensor_time;
    rslt = bmi08a_get_sensor_time(&bmi08xdev, &sensor_time);
    if (rslt == BMI08X_OK) {
      rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
    }
    if (rslt == BMI08X_OK) {
      rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
    }
    if (rslt == BMI08X_OK) {

      imu_in.acc[0] =
          ((float)bmi08x_accel.x) / ((float)IMU_RAW_RANGE / ACCEL_RANGE_G);
      imu_in.acc[1] =
          ((float)bmi08x_accel.y) / ((float)IMU_RAW_RANGE / ACCEL_RANGE_G);
      imu_in.acc[2] =
          ((float)bmi08x_accel.z) / ((float)IMU_RAW_RANGE / ACCEL_RANGE_G);

      imu_in.gyro[0] =
          ((float)bmi08x_gyro.x) / ((float)IMU_RAW_RANGE / GYRO_RANGE_DPS);
      imu_in.gyro[1] =
          ((float)bmi08x_gyro.y) / ((float)IMU_RAW_RANGE / GYRO_RANGE_DPS);
      imu_in.gyro[2] =
          ((float)bmi08x_gyro.z) / ((float)IMU_RAW_RANGE / GYRO_RANGE_DPS);

      if (last_sensor_time > sensor_time) {
        delta_tick = 0x00FFFFFF - (last_sensor_time - sensor_time);
      } else {
        delta_tick = sensor_time - last_sensor_time;
      }
      mfx_manager_run(&imu_in, &imu_out, ((float)delta_tick) / 25600.0f);
      if (eular != NULL) {
        // memcpy(eular,imu_out.rotation,sizeof(float)*3);
        eular->yaw = imu_out.rotation[0];
        eular->pitch = imu_out.rotation[1];
        eular->roll = imu_out.rotation[2];
      }
    }
  }
  return rslt;
}
