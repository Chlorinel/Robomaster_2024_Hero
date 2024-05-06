#include "./algorithm/pid.h"

/**
 * @brief  init pid parameter
 * @param  pid      the pid struct containing parameters
 * @param[in] kp
 * @param[in] ki
 * @param[in] kd
 * @param[in] i_max
 * @param[in] out_max
 * @param[in] deadband
 * @return none
 */
void pid_init(pid_struct_t *pid,
              const float *kpid,
              float i_max,
              float out_max,
              float deadband)
{

  if (pid == ((void *)0))
    return;
  pid->kp = kpid[0];
  pid->ki = kpid[1];
  pid->kd = kpid[2];
  pid->i_max = i_max;
  pid->out_max = out_max;
  // pid->k_deadband = deadband*deadband;
  pid->k_deadband = deadband;
}

/**
 * @brief  pid reset
 * @param  pid    the pid struct containing parameters
 * @return none
 */
void pid_reset(pid_struct_t *pid)
{
  pid->err[1] = pid->err[0] = 0.0f;
  pid->i_out = 0.0f;
}

/**
 * @brief  pid calculation
 * @param  pid    the pid struct containing parameters
 * @param  ref    reference value
 * @param  cur    current value
 * @return pid calculation result
 * @note pid->k_deadband is invaild in this process
 */
float pid_calc(pid_struct_t *pid, float ref, float cur)
{
  pid->err[1] = pid->err[0];
  pid->err[0] = ref - cur;
  pid->p_out = pid->kp * pid->err[0];
	if(pid->ki!=0)
		pid->i_out += pid->ki * pid->err[0];
	else
		pid->i_out=0;
	
  pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
  CLAMP(pid->i_out, -pid->i_max, pid->i_max);

  pid->t_out = pid->p_out + pid->i_out + pid->d_out;
  CLAMP(pid->t_out, -pid->out_max, pid->out_max);
  return pid->t_out;
}

/**
 * @brief  pid calculation
 * @param  pid    the pid struct containing parameters
 * @param  ref    reference value
 * @param  cur    current value
 * @return pid calculation result
 * @note pid->k_deadband is invaild in this process
 */
float pid_calc_pos(pid_struct_t *pid, float ref, float cur, int angle_range)
{
  pid->err[1] = pid->err[0];
  pid->err[0] = get_delta_ang(ref, cur, angle_range);
  pid->p_out = pid->kp * pid->err[0];
  if (pid->ki != 0)
    pid->i_out += pid->ki * pid->err[0];
  else
    pid->i_out = 0;
  pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
  CLAMP(pid->i_out, -pid->i_max, pid->i_max);

  pid->t_out = pid->p_out + pid->i_out + pid->d_out;
  CLAMP(pid->t_out, -pid->out_max, pid->out_max);
  return pid->t_out;
}

/**
 * @brief  pid calculation with deadband
 * @param  pid    the pid struct containing parameters
 * @param  ref    reference value
 * @param  cur    current value
 * @return pid calculation result
 * @todo two deadband ways: asymptote or normal?
 */
float pid_calc_deadband(pid_struct_t *pid, float ref, float cur)
{
  pid->err[1] = pid->err[0];

  // float err_square = err*err;
  // pid->err[0] = err*(err_square/(err_square + pid->k_deadband));
  pid->err[0] = ref - cur;
  if (pid->err[0] > pid->k_deadband)
  {
    pid->err[0] -= pid->k_deadband;
  }
  else if (pid->err[0] < -pid->k_deadband)
  {
    pid->err[0] += pid->k_deadband;
  }
  else
  {
    if (ref < pid->k_deadband && ref > -pid->k_deadband // reference value get into deadband
        && (pid->err[1] < -pid->k_deadband || pid->err[1] > pid->k_deadband))
    { // last error out of deadband
      pid->i_out = 0.0f;
    }
    if (pid->deadband_zero_output)
      return 0.0f;
  }

  pid->p_out = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
  CLAMP(pid->i_out, -pid->i_max, pid->i_max);

  pid->t_out = pid->p_out + pid->i_out + pid->d_out;
  CLAMP(pid->t_out, -pid->out_max, pid->out_max);
  return pid->t_out;
}

/**
  * @brief  pid dual loop calculation
  * @param  pid     the inner and outter loop pid struct containing parameters

  * @return pid calculation result
  */
float pid_dual_loop(pid_struct_t *pid, float err, float err_out)
{
  float pid_in_output;
  pid_in_output = -pid_calc(&pid[POS_LOOP], 0, err);
  return pid_calc(&pid[RPM_LOOP], pid_in_output, err_out);
}