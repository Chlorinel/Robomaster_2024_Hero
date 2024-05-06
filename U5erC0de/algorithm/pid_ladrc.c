#include "./algorithm/pid_ladrc.h"

#include <math.h>
/**
 *	adrc相关:
 *		1.https://blog.csdn.net/theysay_/article/details/106098820
*/

/**
 *	@brief	获得数字正负号
 */
static inline float ladrc_sign(float val)
{
	if(val >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}
/**
 *	@brief	快速求根号
 */
float fast_sqrtf(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = x* y * (1.5f - (halfx * y * y));
	
	return y;
}
/**
 * 	@brief 	最速离散控制函数
 * 	@param	x1->距离目标差值,用于追踪跟踪输入
 * 	@param	x2->输入的导数,追踪输入的导数
 *  @param	r->调节系数，r越大跟踪效果越好，但微分信号会增加高频噪声
 * 	@param	h0->差分时间
 *  @return	v2的控制量,有v2=v2+dt*fh,或v2'=fh
 * 	@ref		1.代码实现	https://blog.csdn.net/qq_42278205/article/details/114547808
*/
float ladrc_fhan(float v1, float v2, float r0, float h0)
{
	float d = h0 * h0 * r0;
	float a0 = h0 * v2;
	float y = v1 + a0;
	float a1 = fast_sqrtf(d*(d + 8.0f*fabsf(y)));
	float a2 = a0 + ladrc_sign(y)*(a1-d)*0.5f;
	float sy = (ladrc_sign(y+d) - ladrc_sign(y-d))*0.5f;
	float a = (a0 + y - a2)*sy + a2;
	float sa = (ladrc_sign(a+d) - ladrc_sign(a-d))*0.5f;
	float fh=-r0*(a/d - ladrc_sign(a))*sa - r0*ladrc_sign(a);
	return fh;
}
/**
 *	@brief	微分跟踪器初始化
 *	@ref		1.原理讲解:	https://blog.csdn.net/JeSuisDavid/article/details/119597487
 */
void ladrc_td_init(td_para_t* td, float h, float r0, float h0)
{
	td->h = h;
	td->r0 = r0;
	td->h0 = h0;
	//td->v1 = td->v2 = 0.0f;
}

/**
 *	@brief	微分跟踪器
 * 	@param	td->微分结构体
 *	@param	v->追踪目标
 *	@ref		 
*/
void ladrc_td(td_para_t* td, float v)
{
	float fh = ladrc_fhan(td->v1 - v, td->v2, td->r0, td->h0);
	td->v1 += td->h * td->v2;
	td->v2 += td->h * fh;
}

void ladrc_init(ladrc_para_t* ladrc_para,float omega_c,float k)
{
	ladrc_para->kp = omega_c*omega_c;
	ladrc_para->kd = 2*omega_c*k;
	//leso->a0 = 0.0f;
	//ladrc_para->u = 0.0f;
}

float ladrc_cal(ladrc_para_t* ladrc_para ,float v)
{
	ladrc_td(&ladrc_para->td_para, v);
	update_leso(&ladrc_para->leso_para,v, ladrc_para->u);
	
	float e1 = ladrc_para->td_para.v1 - ladrc_para->leso_para.z1;
	float e2 = ladrc_para->td_para.v2 - ladrc_para->leso_para.z2;
	float u0 = ladrc_para->kp * e1 + ladrc_para->kd * e2;

	ladrc_para->u = u0 - ladrc_para->leso_para.z3 / ladrc_para->leso_para.b1;
	return ladrc_para->u;
}

