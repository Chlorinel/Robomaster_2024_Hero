#include "./algorithm/filter.h"
#include "main.h"
#include "math.h"
#include "stdlib.h"

// 一阶低通滤波器
void LPF_init(LPF_t *filter, float ts, float fc) {
  filter->orig_val = 0;
  filter->ts = ts;
  filter->fc = fc;
  filter->a = filter->ts / (filter->ts + 1.0f / 2 / PI / filter->fc);
}

float LPF_update(LPF_t *filter, float new_value) {
  if (isnan(new_value) == 1)
    return filter->fltr_val;

  filter->a = filter->ts / (filter->ts + 1.0f / 2 / PI / filter->fc);
  filter->orig_val = new_value;
  float temp =
      filter->fltr_val + filter->a * (filter->orig_val - filter->fltr_val);
  if (isnan(temp) == 0) {
    filter->fltr_val = temp;
  }
  return filter->fltr_val;
}

//------------------------------------------
// mean filter parameter
const float m2_cof[3] = {0.2, 0.5, 0.3};

//------------------------------------------
// IIR filter parameter (2 or 3 refer to the order)

// IIR 2-Order Filter Fs=500Hz,Fc=30Hz Butterworth
const float NUM_2[3] = {0.02785976604, 0.05571953207, 0.02785976604};
const float DEN_2[3] = {1, -1.475480437, 0.5869194865};

// //IIR 3-Order Filter Fs=500Hz,Fc=20Hz Butterworth
const float NUM_3[4] = {0.001567010302, 0.004701030906, 0.004701030906,
                        0.001567010302};
const float DEN_3[4] = {1, -2.498608351, 2.115254164, -0.6041097045};

// IIR 3-Order Filter Fs=300Hz,Fc=90Hz Chebyshev Type II
//  const float NUM_3[4] = {0.009448255412,0.01762608625,0.01762608625,
//  0.009448255412}; const float DEN_3[4] =
//  {1,-2.147717714,1.622931719,-0.4210654497};

////FIR 5-Order Filter Fs=500Hz,Fc=20Hz, Windows(Chebyshev)
const float NUM_5[6] = {0.04964470491, 0.1659367979, 0.2844184935,
                        0.2844184935,  0.1659367979, 0.04964470491};
// const float NUM_5[6] =
// {0.09592749923,0.1705435663,0.233528927,0.233528927,0.1705435663,0.09592749923};
/*---------------------------------------------------------------------------------------------------------*/
#define D_CUTOFF 2
#define MIN_CUTOFF 0.1f
#define FILTER_BETA 0.05f

/**
 * @brief  3-order mean filter(average)
 * @param[in]  x_i input in fp32
 * @return output fp32
 */
float mean_filter_2(float x_i) {
  static float x_mean[2] = {0};
  float y = m2_cof[0] * x_i + m2_cof[1] * x_mean[0] + m2_cof[2] * x_mean[1];
  x_mean[1] = x_mean[0];
  x_mean[0] = x_i;
  return y;
}

/**
 * @brief  2-order IIR filter
 * @param[in]  x    input in fp32
 * @param[in]  ch   input channel, there are MAX_FILTER_CH_2 channels available
 * @return output fp32
 */
float iir_filter_2(float x, unsigned int ch) {
  static float y2, x2_n1[MAX_FILTER_CH_2], x2_n2[MAX_FILTER_CH_2],
      y2_n1[MAX_FILTER_CH_2], y2_n2[MAX_FILTER_CH_2];
  y2 = NUM_2[0] * x + NUM_2[1] * x2_n1[ch] + NUM_2[2] * x2_n2[ch] -
       DEN_2[1] * y2_n1[ch] - DEN_2[2] * y2_n2[ch];
  // if(y<EPS && y>-EPS) y = 0.0;
  y2_n2[ch] = y2_n1[ch];
  y2_n1[ch] = y2;
  x2_n2[ch] = x2_n1[ch];
  x2_n1[ch] = x;
  return y2;
}

/**
 * @brief  3-order IIR filter
 * @param[in]  x    input in fp32
 * @param[in]  ch   input channel, there are MAX_FILTER_CH_2 channels available
 * @return output fp32
 */

float iir_filter_3(float x, unsigned int ch) {
  static float y3, x3_n1[MAX_FILTER_CH_3], x3_n2[MAX_FILTER_CH_3],
      x3_n3[MAX_FILTER_CH_3], y3_n1[MAX_FILTER_CH_3], y3_n2[MAX_FILTER_CH_3],
      y3_n3[MAX_FILTER_CH_3];
  y3 = NUM_3[0] * x + NUM_3[1] * x3_n1[ch] + NUM_3[2] * x3_n2[ch] +
       NUM_3[3] * x3_n3[ch] - DEN_3[1] * y3_n1[ch] - DEN_3[2] * y3_n2[ch] -
       DEN_3[3] * y3_n3[ch];
  // if(y<EPS && y>-EPS) y = 0.0;
  y3_n3[ch] = y3_n2[ch];
  y3_n2[ch] = y3_n1[ch];
  y3_n1[ch] = y3;
  x3_n3[ch] = x3_n2[ch];
  x3_n2[ch] = x3_n1[ch];
  x3_n1[ch] = x;
  return y3;
}

/**
 * @brief  5-order FIR filter
 * @param[in]  x    input in fp32
 * @param[in]  ch   input channel, there are MAX_FILTER_CH_5 channels available
 * @return output fp32
 */
float fir_filter_5(float x, unsigned int ch) {
  static float y5, x5_n1[MAX_FILTER_CH_5], x5_n2[MAX_FILTER_CH_5],
      x5_n3[MAX_FILTER_CH_5], x5_n4[MAX_FILTER_CH_5], x5_n5[MAX_FILTER_CH_5];
  y5 = NUM_5[0] * x + NUM_5[1] * x5_n1[ch] + NUM_5[2] * x5_n2[ch] +
       NUM_5[3] * x5_n3[ch] + NUM_5[4] * x5_n4[ch] + NUM_5[5] * x5_n5[ch];

  x5_n5[ch] = x5_n4[ch];
  x5_n4[ch] = x5_n3[ch];
  x5_n3[ch] = x5_n2[ch];
  x5_n2[ch] = x5_n1[ch];
  x5_n1[ch] = x;
  return y5;
}

/**
 * @brief  one euro filter
 * @param[in]  x    input in fp32
 * @param[in]  dt   time duration between calls in seconds
 * @param[in]  ch   input channel, there are MAX_ONE_EURO_FILTER_CH channels
 * available
 * @return output fp32
 */
float one_euro_filter(float x, float dt, unsigned int ch) {
  static float x_prev[MAX_ONE_EURO_FILTER_CH], dx_prev[MAX_ONE_EURO_FILTER_CH];
  float dx, dx_hat, a_d, a, x_hat, cutoff;
  // The filtered derivative of the signal.
  a_d = 2 * PI * D_CUTOFF * dt / (1 + 2 * PI * D_CUTOFF * dt);
  dx = (x - x_prev[ch]) / dt;
  dx_hat = a_d * dx + (1 - a_d) * dx_prev[ch];

  // The filtered signal.
  if (dx_hat < 0) {
    cutoff = MIN_CUTOFF - FILTER_BETA * dx_hat;
  } else {
    cutoff = MIN_CUTOFF + FILTER_BETA * dx_hat;
  }

  a = 2 * PI * cutoff * dt / (1 + 2 * PI * cutoff * dt);
  x_hat = a * x + (1 - a) * x_prev[ch];
  // Memorize the previous values.
  x_prev[ch] = x_hat;
  dx_prev[ch] = dx_hat;
  return x_hat;
}

/*
float adaptive_lp_filter(float x, float deadband, float cnt_limit)
{
        static float trust= 0.2;
        static int cmp, cmp_last, cnt = 0;
        static float x_last, x_filtered;
        cmp = (x - x_last > 0);
        if (cmp == cmp_last){
                if (abs (x - x_last) > deadband)
                  cnt += 5;
                if (cnt >= cnt_limit)
                        trust += 0.1;
        }else{
                cnt = 0;
                trust  = 0.2;
                cmp_last = cmp;
        }
        if (trust  > 0.99) trust= 0.99;
        x_filtered = (1-trust) * x_last + trust * x;
        x_last = x;
        return x_filtered;
} */

/*
 * @brief  Init fields of structure kalman1_state.
 * @param[in]  filter pointer to the struct of 1-dimension kalman parameters
 * @param[in]  init_x initial value of sample
 * @param[in]  q initial predict noise covariance
 * @param[in]  r noise covariance of measurement((3σ)^2)
 * @return none
 */
void kalman1_init(kalman1_state *state, float init_x, float q, float r) {
  state->x = init_x;
  state->p = 0.0f;
  state->A = 1;
  state->H = 1;
  state->q = q; // 10e-6;  /* predict noise convariance */
  state->r = r; // 10e-5;  /* measure error convariance */
}

/*
 * @brief  1 Dimension Kalman filter
 * @param[in]  filter pointer to the struct of 1-dimension kalman parameters
 * @param[in]  z_measure measured value
 * @return estimated values
 */
float kalman1_filter(kalman1_state *state, float z_measure) {
  /* Predict */
  state->x = state->A * state->x;
  state->p = state->A * state->A * state->p + state->q;
  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

  /* Measurement */
  state->gain =
      state->p * state->H / (state->p * state->H * state->H + state->r);
  state->x = state->x + state->gain * (z_measure - state->H * state->x);
  state->p = (1 - state->gain * state->H) * state->p;

  return state->x;
}

/*
 * @brief  1 Dimension Kalman filter with estimation
 * @param[in]  filter pointer to the struct of 1-dimension kalman parameters
 * @param[in]  z_measure measured value
 * @return estimated values
 */
float kalman1_filter_est(kalman1_state *state, float z_measure, float est) {
  /* Predict */
  state->x = est;
  state->p = state->p + state->q;

  /* Measurement */
  state->gain =
      state->p * state->H / (state->p * state->H * state->H + state->r);
  state->x = state->x + state->gain * (z_measure - state->H * state->x);
  state->p = (1 - state->gain * state->H) * state->p;

  return state->x;
}

/**
 * @brief  kalman filter struct init
 * @param[in]  filter pointer to the struct of kalman parameters
 * @param[in]  q_cur noise covariance of current value
 * @param[in]  q_rate noise covariance of the rate
 * @param[in]  r noise covariance of measurement((3σ)^2)
 * @return none
 */
void kalman_init(kalman_filter_t *filter, float q_cur, float q_rate, float r) {
  filter->Q_cur = q_cur;
  filter->Q_bias = q_rate;
  filter->R_measure = r;

  filter->P[0][0] = 0.0f;
  filter->P[0][1] = 0.0f;
  filter->P[1][0] = 0.0f;
  filter->P[1][1] = 0.0f;
  filter->K[0] = 0.0f;
  filter->K[1] = 0.0f;
  filter->S = 0.0f;
  filter->y = 0.0f;
  filter->bias = 0.0f;
}

/**
 * @brief  kalman filter update process
 * @param[in]  filter pointer to the struct of kalman parameters
 * @param[in]  cur current value to be filtered
 * @param[in]  rate the rate of current value(cur per sec)
 * @param[in]  dt delta time in second
 * @return the filtered value
 */
float kalman_update(kalman_filter_t *filter, float cur, float rate, float dt) {
  // step 1
  filter->rate = rate - filter->bias;
  filter->result += dt * filter->rate;
  // step 2: Update estimation error covariance - Project the error covariance
  // ahead
  filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] -
                           filter->P[1][0] + filter->Q_cur);
  filter->P[0][1] -= dt * filter->P[1][1];
  filter->P[1][0] -= dt * filter->P[1][1];
  filter->P[1][1] += filter->Q_bias * dt;
  // Step 3
  filter->y = cur - filter->result;
  // Step 4: Calculate Kalman gain
  filter->S = filter->P[0][0] + filter->R_measure;

  // Step 5: Kk = Pk|k-1 * H^T * S^-1_k
  filter->K[0] = filter->P[0][0] / filter->S;
  filter->K[1] = filter->P[1][0] / filter->S;

  // Step 6
  filter->result += filter->K[0] * filter->y;
  filter->bias += filter->K[1] * filter->y;

  // Step 7
  filter->P[0][0] -= filter->K[0] * filter->P[0][0];
  filter->P[0][1] -= filter->K[0] * filter->P[0][1];
  filter->P[1][0] -= filter->K[1] * filter->P[0][0];
  filter->P[1][1] -= filter->K[1] * filter->P[0][1];

  return filter->result;
}

/**
 * @brief  kalman filter result value set
 * @param[in]  cur value to set
 * @return none
 */
void kalman_set(kalman_filter_t *filter, float cur) { filter->result = cur; }
