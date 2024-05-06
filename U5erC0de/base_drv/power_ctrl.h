#ifndef _POW_CTRL_H
#define _POW_CTRL_H

#include "algorithm/pid.h"
// #include chassis_module_motor_ctrl_h
#include "algorithm/util.h"
#include "base_drv/drv_cap/super_cap.h"
#include "base_drv/drv_referee/referee.h"

#define CLEAN_FLAG 0

#define CLEAN_FLAG 0 // 铲屎宏
// 旧框架里有部分代码需要用到旧功率限制结构里的参数,这里先用宏注释掉先
#define ENABLE_MOTOR_POWER_PARAM_CALIBR_FLAG 0 // 使能标定模式
// 注意,在标定模式,函数chassis_power_distributor仅会使能0号电机的输出
#if ENABLE_MOTOR_POWER_PARAM_CALIBR_FLAG == 1
#warning "Be careful here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
/***
 * 标定模式下,需要开启串口(通常是usart1)调用vofa.c将电机的P、omega、I三参数发上上位机
 * 上位机需要运行https://scnu-pioneer.coding.net/p/ec/d/2023_super_caps/git/tree/V3.0(Lizi)_仿真及公式计算部分/matlab/log_motor_data
 * 中的log.py脚本将数据打包成.csv文件。发送过程中推荐使用电机在悬空(无负载)情况下的数据,期间要不断用遥控器或其他输入设备改变电机的期望速度。
 * 采集完成后需要打开matlab运行同仓库下的chassis_power_fit.m文件对电机P、omega、I三参数之间的关系进行拟合,输出窗口就能得到P_idle、Kn、MLC、
 * ESR四个参数
 * 这四个参数的量纲一般都比较抽象,甚至可能在转为用W、rad/s、A来表示的时候数值也跟数据手册有所区别,但没逝,在功率限制这里完全是可以做黑箱来使用的
 * 得到四参数后,继续开着标定模式并打开debug或使用vofa查看MPPCB.fitted_power与MPPCB.real_power是否相符,寄了那就重标或者抄旧参
 * 若参数正确但控制效果仍不理想,那就好好思考设置到chassis_pow_lim里的raw_expt_Iq与real_Iq的单位是否相符
 ***/
#endif

#define ENABLE_POWER_LIMIT_FLAG 1
// 使能功率限制,觉得功率限制有问题的可以置0关闭
/***
 * 功率限制原理:
 * 参考:https://www.bilibili.com/video/BV1tu41177wm,来自西交利物浦的分享~
 * 电机输入功率满足P=Kn*omega*Iq + ESR*Iq^2 + MLC*omega^2,
 * 其中Kn、ESR、MLC均为常值, 所以上述关系可以简化为P=F(omega,Iq)
 * 应用时首先可以根据expt_omega与expt_Iq得到expt_P,又已知咱们需要限制的最大功率max_P,
 * 那就可以按照各电机的expt_P之比分配max_P得到set_P
 * 接下来就是需要将set_P转为set_Iq了,观察功率模型,可以发现以Iq为未知量时式子其实就是一个一元二次方程,
 * omega为当前角速度,这样就可以得到每个电机在有最大功率限制时的每个时刻应该有的输出了
 ***/
#define CHASSIS_MOTOR_NUM 4 // 底盘电机总数
/**
 * @struct chassis_power_lim_t
 * @brief chassis power limit parameter
 */
/*功率限制参数结构体*/
typedef struct {
  /**需要在chassis.c中预先设定的值**/
  float power_limit; // 最大额定输入功率
  float referee_power_buffer; // 实际缓冲能量值,从裁判系统读取,亦可软件设定
  float expt_cap_energy_ratio;     // 期望电容剩余能量百分比
  float expt_referee_power_buffer; // 期望缓冲能量值

  /**需要在chassis_base.c中预先设定的值**/
  float raw_expt_Iq[CHASSIS_MOTOR_NUM]; // 原始设定输出电流
  float real_Iq[CHASSIS_MOTOR_NUM];     // 实际电调输出电流
  float expt_omega[CHASSIS_MOTOR_NUM];  // 期望电机转子角速度
  float real_omega[CHASSIS_MOTOR_NUM];  // 实际电机转子角速度

  /**功率限制处理值,直接按原本在raw_expt_Iq里对应的位置发出即可**/
  float set_Iq[CHASSIS_MOTOR_NUM]; // 最终设定输出电流

  /**中间处理值**/
  float raw_power[CHASSIS_MOTOR_NUM]; // 原闭环控制器所设定的功率
  float fitted_power[CHASSIS_MOTOR_NUM]; // 根据电机数据拟合出的实际功率
  float set_power[CHASSIS_MOTOR_NUM]; // 最终均分后所得的功率
  float set_total_power; // 最终设定电流值所对应的上限功率值,由电容组电压决定		
			
	float real_cap_ratio; // 电容实际储能量
	float chassis_real_total_power; // 功率板采集到的实际总功率
} chassis_power_lim_t;

typedef struct {
  float real_power; // 单电机消耗功率
  float set_ctrl_value; // 设定转子控制值,应为一个与切向电流(A)呈线性相关的值
  float real_ctrl_value; // 实际转子控制值,应为一个与切向电流(A)呈线性相关的值
  float real_omega; // 实际转子角速度,应为一个与转子角速度(rad/s)呈线性相关的值
  float fitted_power; // 功率模型拟合得到的电机功率
} motor_power_parameter_calibrate_bag_t;
///<<< 以下宏里的参数仅适用于M3508电机 >>>///
// 最大输出电流限制(16384进制->20A)
#define MAX_CURRENT_LIMIT (20.f)

#define ecd2iq(_ecd)                                                           \
  ((_ecd)*20.f / C620_OUTPUT_MAX) // 电调电流编码值转实际转矩电流
#define iq2ecd(_iq)                                                            \
  ((_iq)*C620_OUTPUT_MAX / 20.f) // 电调实际转矩电流转电流编码值

// #define REDUCTION_RATIO (3591.0f / 187.0f)        // 减速比
#define rpm2radps(_rpm) ((_rpm)*PI / 30.f) // rpm转rad/s

float get_M3508_Iq(float P, float omega, int8_t dirc);
float get_M3508_power(float omega, float Iq);
void chassis_power_distributor(chassis_power_lim_t *pow_lim,
                               uint8_t is_cap_enable);
void update_soft_chassis_energy_buffer(chassis_power_lim_t *pow_lim);

#endif
