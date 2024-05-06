#ifndef _cordic_h_
#define _cordic_h_
void cordic_atan_sqrtf(float yf, float xf, float* ret_atan, float* ret_sqrt);
void cordic_sin_cosf(float ang, float* ret_sin, float* ret_cos);
float cordic_logf(float e);

#endif
