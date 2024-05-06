#ifndef _FILTER_H
#define _FILTER_H

#include "./base_drv/drv_conf.h"
// 低通滤波器
typedef struct
{
	float orig_val; // orignal value,原始值
	float fltr_val; // filter value,滤波值
	float ts;		// 采用间隔时间
	float fc;		// 截至频率
	float a;		// 比例系数,init会自动更新
} LPF_t;
void LPF_init(LPF_t *filter, float ts, float fc);
float LPF_update(LPF_t *filter, float new_value);

// IIR filter channel number (2 or 3 refer to the order)

#define MAX_FILTER_CH_2 4
#define MAX_FILTER_CH_3 4
#define MAX_FILTER_CH_5 3
#define MAX_ONE_EURO_FILTER_CH 3

#define CHASSIS_FILTER3_CH1 0
#define CHASSIS_FILTER3_CH2 1
#define CHASSIS_FILTER3_CH3 2

#define GIMBAL_PITCH_FILTER2_RPM_CH 0
#define GIMBAL_YAW_FILTER2_RPM_CH 1
#define SHOOT_LEFT_FILTER2_RPM_CH 2
#define SHOOT_RIGHT_FILTER2_RPM_CH 3

float mean_filter_2(float x_i);
float iir_filter_2(float x, unsigned int ch);
float iir_filter_3(float x, unsigned int ch);

float fir_filter_5(float x, unsigned int ch);
float one_euro_filter(float x, float dt, unsigned int ch);

// 卡尔曼滤波
typedef struct
{
	float Q_cur, Q_bias, R_measure;
	float P[2][2];
	float K[2];
	float S, y, bias;
	float rate, result;
} kalman_filter_t;

/* 1 Dimension */
typedef struct
{
	float x; /* state */
	float A; /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
	float H; /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
	float q; /* process(predict) noise convariance */
	float r; /* measure noise convariance */
	float p; /* estimated error convariance */
	float gain;
} kalman1_state;

void kalman_init(kalman_filter_t *filter, float q_cur, float q_rate, float r);
float kalman_update(kalman_filter_t *filter, float cur, float rate, float dt);
void kalman_set(kalman_filter_t *filter, float cur);

float kalman1_filter(kalman1_state *state, float z_measure);
float kalman1_filter_est(kalman1_state *state, float z_measure, float est);
void kalman1_init(kalman1_state *state, float init_x, float q, float r);

#endif
