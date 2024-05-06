#ifndef _pid_H
#define _pid_H

#include "./algorithm/util.h"

#define POS_LOOP 0
#define RPM_LOOP 1
/*
  速度+电流环为
  #define RPM_LOOP 0
  #define CUR_LOOP 1
*/

/**
 * @struct pid_struct_t
 * @brief struct of pid parameters
 */
typedef struct _pid_struct_t
{
  float kp;         ///< Kp
  float ki;         ///< Ki
  float kd;         ///< Kd
  float i_max;      ///< limit maximum of absolute value of integral
  float out_max;    ///< limit maximum of absolute value of output
  float k_deadband; ///< set the deadband

  int deadband_zero_output;
  float err[2]; ///< error and last error

  float p_out;
  float i_out;
  float d_out;

  float t_out; ///< total out
} pid_struct_t;
#define pid_update_index(_pid, _kpid) (_pid.kp = _kpid[0], _pid.ki = _kpid[1], _pid.kd = _kpid[2])
void pid_init(
    pid_struct_t *pid,
    const float *kpid,
    float i_max,
    float out_max,
    float deadband);

void pid_reset(pid_struct_t *pid);
float pid_calc(pid_struct_t *pid, float ref, float cur);
float pid_calc_pos(pid_struct_t *pid, float ref, float cur, int angle_range);
float pid_calc_deadband(pid_struct_t *pid, float ref, float cur);
float pid_dual_loop(pid_struct_t *pid, float err, float err_out);

#endif
