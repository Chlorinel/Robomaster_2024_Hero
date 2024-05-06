#ifndef _pid_LESO_H
#define _pid_LESO_H

#include "./algorithm/pid.h"
typedef struct
{
	float h;
	float beta1; // beta_N为观测器闭环的增益矩阵,相当于PID
	float beta2;
	float beta3;
	float a0;	 // 状态矩阵A
	float b0;	 // 控制矩阵B
	float af_z1; // 角度环误差补偿比例
	float af_z2; // 速度环误差补偿比例
	float b1;	 // 电流环设定补偿比例
	/* LESO */
	float z1; // 预测对象y
	float z2; // 预测对象y'
	float z3; // 预测扰动f
	float z_max;
} leso_para_t; /* Linear ESO */

typedef struct
{
	pid_struct_t pid[2];
	leso_para_t leso;
} pid_leso_t;

float pid_leso_dualloop(pid_struct_t pid[2], leso_para_t *leso, float err, float err_out, float feed);

void leso_6020_init(leso_para_t *leso, float h, float b1, float _J);
void leso_3508_init(leso_para_t *leso, float h, float b1, float _J);
void update_leso(leso_para_t *leso, float y, float u);
void reset_leso(leso_para_t *leso);

#endif
