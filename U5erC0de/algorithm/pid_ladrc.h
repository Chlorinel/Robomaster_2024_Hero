
#ifndef _pid_LADRC_H
#define _pid_LADRC_H

#include "./algorithm/pid_leso.h"

typedef struct
{
	float v1;				//系统的控制量
	float v2;				//v1的导数
	float r0;				//调节系数,越大追踪效果越好,但噪声增益越高
	float h0;				//滤波因子,越大滤波效果越好,一般略大于h
	float h;				//步长,越小滤波效果越好
}td_para_t;

typedef struct
{
	leso_para_t leso_para;
	td_para_t td_para;
	float kp,kd;
	float u;
}ladrc_para_t;

void ladrc_td_init(td_para_t* td, float h, float r0, float h0);
void ladrc_init(ladrc_para_t* ladrc_para,float omega_c,float k);
float ladrc_cal(ladrc_para_t* ladrc_para ,float v);

#endif

