#include "./algorithm/pid_leso.h"
#include "./algorithm/filter.h"

#define PI (3.14159265358979323846f)

// #define Z_MAX 30000.0f
// #define K_M 0.741f  // N*m/A
// #define K_E 13.33f  // rpm/V
// #define R_S 3*1.8f //Ω
// #define M6020_OMEGA 6.0f

typedef struct
{
  float K_M;   // 	转矩常数				N*m/A
  float K_E;   // 	转速常数				rpm/V
  float R_S;   //	三相电阻总值		Ω
  float OMEGA; //	直径						mm
  float Z_MAX; //	最大控制量			//
} leso_model_init_t;
const leso_model_init_t M3508 = {.K_M = 0.3f, .K_E = 24.48f, .R_S = 3 * 0.194f, .OMEGA = 3.5f, .Z_MAX = 16384};
const leso_model_init_t GM6020 = {.K_M = 0.741f, .K_E = 13.33f, .R_S = 3 * 1.8f, .OMEGA = 6.0f, .Z_MAX = 30000};

void update_leso(leso_para_t *leso, float y, float u);

/**
 * @brief  pid dual loop calculation with LESO
 * @param      pid     the inner and outter loop pid struct containing parameters
 * @param      leso   the leso struct containing parameters
 * @param[in]  err     error value
 * @param[in]  err_out outter loop feedback value
 * @return pid calculation result
 */
float pid_leso_dualloop(pid_struct_t pid[2], leso_para_t *leso, float err, float err_out, float feed)
{
  float pid_in_output, pid_out_output;

  // err -= leso->z1*(ANGLE_RANGE/(2.0f*PI))*leso->af_z1;
  err -= leso->z1 * leso->af_z1;

  pid_in_output = pid_calc(&pid[POS_LOOP], err, 0);
  pid_in_output -= leso->z2 * (30 / PI) * leso->af_z2;

  pid_out_output = pid_calc(&pid[RPM_LOOP], pid_in_output, err_out);
  pid_out_output -= leso->z3 * leso->b1;
  // TODO: iir_filter for output

  // update_leso(leso,err*(2.0f*PI/ANGLE_RANGE),pid_out_output);
  update_leso(leso, err, feed);
  return pid_out_output;
}
/**
 * @brief  init leso for gm6020 pid
 * @param  leso   the leso struct containing parameters
 * @param[in] h
 * @param[in] b1 		:   by default 0.1 ~ 2
 * @param[in] a_z1		:   by defalut 0.01
 * @param[in] _J		  :   momentum of inertia, kg.m^2
 * @param[in] omega0 :   bandwidth of disturb
 * @return none
 */
void leso_6020_init(leso_para_t *leso, float h, float b1, float _J)
{
  leso->h = h;
  leso->b1 = b1;
  // (s + w)^2 = s^2 + beta_1 * s + beta_2

  leso->beta1 = 3.0f * GM6020.OMEGA;
  leso->beta2 = 3.0f * GM6020.OMEGA * GM6020.OMEGA;
  leso->beta3 = GM6020.OMEGA * GM6020.OMEGA * GM6020.OMEGA;

  // b_0 = Km*K/(JR)
  leso->b0 = GM6020.K_M * (24.0f / 30000.0f) / (_J * GM6020.R_S);
  // a_0 = Km*Ke/(JR)
  leso->a0 = -GM6020.K_M / (_J * GM6020.R_S) / GM6020.K_E * (PI / 30);

  leso->af_z1 = 0.1f;
  leso->af_z2 = 1.0f;

  leso->z_max = GM6020.Z_MAX;
  // leso->z1 = leso->z2 = leso->z3= 0.0f;
}
/**
 * @brief  init leso for m3508 pid
 * @param  leso   the leso struct containing parameters
 * @param[in] h
 * @param[in] b1 		: by default 0.1 ~ 2
 * @param[in] a_z1		: by defalut 0.01
 * @param[in] _J		: momentum of inertia, kg.m^2
 * @param[in] omega0 	: bandwidth of disturb
 * @return none
 */
void leso_3508_init(leso_para_t *leso, float h, float b1, float _J)
{
  leso->h = h;
  leso->b1 = b1;
  // (s + w)^2 = s^2 + beta_1 * s + beta_2

  leso->beta1 = 3.0f * M3508.OMEGA;
  leso->beta2 = 3.0f * M3508.OMEGA * M3508.OMEGA;
  leso->beta3 = M3508.OMEGA * M3508.OMEGA * M3508.OMEGA;

  // b_0 = Km*K/(JR)
  leso->b0 = M3508.K_M * (24.0f / 30000.0f) / (_J * M3508.R_S);
  leso->a0 = -M3508.K_M / (_J * M3508.R_S) / M3508.K_E * (PI / 30);

  leso->af_z1 = 0.1f;
  leso->af_z2 = 1.0f;

  // leso->z1 = leso->z2 = leso->z3= 0.0f;

  leso->z_max = M3508.Z_MAX;
}

/**
 * @brief  update leso
 * @param  leso   the leso struct containing parameters
 * @param[in] y : feedback value(current error in rad)
 * @param[in] u : current output value
 * @return none
 */
void update_leso(leso_para_t *leso, float y, float u)
{
  float e = leso->z1 - y;

  leso->z1 += leso->h * (leso->z2 - leso->beta1 * e);
  if (leso->z1 > leso->z_max)
    leso->z1 = leso->z_max;
  else if (leso->z1 < -leso->z_max)
    leso->z1 = -leso->z_max;

  leso->z2 += leso->h * (leso->z3 - leso->beta2 * e + leso->a0 * leso->z2 + leso->b0 * u);
  if (leso->z2 > leso->z_max)
    leso->z2 = leso->z_max;
  else if (leso->z2 < -leso->z_max)
    leso->z2 = -leso->z_max;

  leso->z3 -= leso->h * leso->beta3 * e;
  if (leso->z3 > leso->z_max)
    leso->z3 = leso->z_max;
  else if (leso->z3 < -leso->z_max)
    leso->z3 = -leso->z_max;
}

/**
 * @brief  reset leso
 * @param  leso   the leso struct containing parameters
 * @return none
 */
void reset_leso(leso_para_t *leso)
{
  leso->z1 = leso->z2 = leso->z3 = 0.0f;
}
