#ifndef _UTIL_H
#define _UTIL_H
#include "math.h"
#define CLAMP(x, min, max)                                                     \
  (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#define pow2of(x) ((x) * (x))
#define pow3of(x) (pow2of(x) * (x))
#define pow4of(x) (pow3of(x) * (x))
#define pow5of(x) (pow4of(x) * (x))
static inline float CLAMP0(float input, float max) {
  if (input > max)
    return (max);
  else if (input < -max)
    return -(max);
  else
    return input;
}

#define rad2deg(_n) (180.f * (_n) / PI)
#define deg2rad(_n) (PI * (_n) / 180.f)
#define deg2scl(_n) (4096.f * (_n) / 180.f)
#define scl2deg(_n) (180.f * (_n) / 4096.f)
#define rad2scl(_n) (4096.f * (_n) / PI)
#define scl2rad(_n) (PI * (_n) / 4096.f)

#define DB_friction 75 // 后摩擦轮半径,毫米计量
#define DF_friction 75 // 前摩擦轮半径,毫米计量
#define rpm2mps(_rpm)                                                          \
  (2 * PI * (_rpm) / 60.f * D_friction * 0.001f / 2) // 电机RPM转米每秒
#define mps2rpm_back(_mps)                                                     \
  ((_mps) / (DB_friction * 0.001f / 2) / (2 * PI) * 60.f) // 后电机米每秒转RPM
#define mps2rpm_front(_mps)                                                    \
  ((_mps) / (DF_friction * 0.001f / 2) / (2 * PI) * 60.f) // 前电机米每秒转RPM

#define LIMIT_MIN_MAX(x, M1, M2)                                               \
  (x) = (M1 < M2) ? (((x) <= (M1)) ? (M1) : (((x) >= (M2)) ? (M2) : (x)))      \
                  : (((x) <= (M2)) ? (M2) : (((x) >= (M1)) ? (M1) : (x)))

#define sign(_n) (_n >= 0 ? 1.f : -1.f)
#define _pow2(_n) ((float)(_n) * (float)(_n))
#define max(x1, x2) (x1 >= x2 ? x1 : x2)
#define min(x1, x2) (x1 <= x2 ? x1 : x2)

static inline float fabs_max(float x1, float x2) {
  float abs_x1 = x1 > 0 ? x1 : -x1;
  float abs_x2 = x2 > 0 ? x2 : -x2;
  if (abs_x1 > abs_x2)
    return x1;
  else
    return x2;
}

#define fabs_min(_n1, _n2) (fabsf(_n1) > fabs(_n2) ? (_n2) : (_n1))
/**
 *	@brief 死区范围内,误差连续过渡
 */
static inline float NL_Deadband_ratio(float error, float deadband,
                                      float n_ratio) {
  float k = fabs(error / deadband);
  float kn = 1;
  if (k <= 1)
    for (int i = 0; i < n_ratio; i++)
      kn *= k;
  return kn;
}
/**
 *	@brief 正S形死区限制,小误差小增益,大误差大增益
 */
#define NL_Deadband(error, deadband, n_ratio)                                  \
  (NL_Deadband_ratio(error, deadband, n_ratio) * (error))

// #define ONE_SIDE(_input)
// (_input > 0 ? _input : 0)

/**
 *	@brief 限制绝对最大值
 */

static inline float CLAMP_MAX(float input, float max) {
  if (input > max)
    return (max);
  else if (input < -max)
    return -(max);
  else
    return input;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

// static float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5F375A86 - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
// }
/**
 *	@brief 快速平方根倒数
 */
static float fast_inv_sqrt(float number) {
  const float x2 = number * 0.5F;
  const float threehalfs = 1.5F;

  union {
    float f;
    unsigned int i;
  } conv = {.f = number};
  conv.i = 0x5F375A86 - (conv.i >> 1);
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  return conv.f;
}
/**
 *	@breif 摇杆优化函数,整出S型曲线
 *
 */
#define T_ACC_CNT 2
static float s_curve(float v_max, float cnt) {
  float cntcnt;
  if (cnt < 0.0f) {
    cnt = -cnt;
    v_max = -v_max;
  }
  cntcnt = cnt / ((float)T_ACC_CNT);
  if (cnt < T_ACC_CNT / 2.0f) {
    return 2.0f * v_max * (cntcnt * cntcnt);
  } else if (cnt < T_ACC_CNT) {
    cntcnt = cntcnt - 1.0f;
    return v_max * (1.0f - 2.0f * (cntcnt * cntcnt));
  } else
    return v_max;
}
/**
 *	@brief 不知道有啥用
 *
 */
#define ACC_FILTER_MAX_CH 3
static float get_acc(float current, int ch) {
  static float last_val[ACC_FILTER_MAX_CH][3];
  float result;
  if (ch >= ACC_FILTER_MAX_CH)
    return 0.0f;
  // refer to the Backward differentiation formula
  result = (11 * current - 18 * last_val[ch][2] + 9 * last_val[ch][1] -
            2 * last_val[ch][0]) /
           6.0f;
  last_val[ch][0] = last_val[ch][1];
  last_val[ch][1] = last_val[ch][2];
  last_val[ch][2] = current;
  return result;
}
/**
 *	@brief 获取差角ang1-ang2
 */
#if 1
#include "./base_drv/drv_conf.h"
// ang1->expect,ang2->feedback
static inline int _get_delta_ang(int ang1, int ang2, int angle_range) {
  return (ang1 - ang2 + angle_range * 3 / 2) % angle_range - angle_range / 2;
}
static inline float get_delta_ang(float ang1, float ang2, float angle_range) {
  return fmod((ang1 - ang2 + angle_range * 3.f / 2.f), angle_range) -
         angle_range / 2.f;
  // return (float)_get_delta_ang(ang1 * ANGLE_RANGE * 10, ang2 * ANGLE_RANGE *
  // 10, angle_range * ANGLE_RANGE * 10) / (ANGLE_RANGE * 10);
}

#else
// 会在min与max产生卡顿
static inline float get_delta_ang(float ang1, float ang2, float angle_range) {
  float error = ang1 - ang2;
  if (error > angle_range / 2) {
    return error - angle_range;
  }
  if (error < -angle_range / 2) {
    return error + angle_range;
  } else
    return error;
}

#endif
/**
 *	@brief	将数据的量程映射到指定范围
 */
static inline float range_map(float scale, float min, float max) {
  if (min >= max)
    return scale;

  float ret = scale;
  if (scale >= max)
    ret -= max - min;
  if (scale < min)
    ret += max - min;
  return ret;
}

#define clone(list1, list2)                                                    \
  (sizeof(list1) == sizeof(list2) ? memcpy(&list1, &list2, sizeof(list1))      \
                                  : "内存长度不对")
#define clone_datfptr(list1, ptr) (memcpy(&list1, ptr, sizeof(list1)))
#endif
