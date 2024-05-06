#ifndef _BLDC_model_h_
#define _BLDC_model_h_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE

typedef struct BLDC_model
{
    struct motor_intrinsic_parameter // 电机固有参数
    {
        float R;  // 相电阻
        float L;  // 相电感
        float Kt; // 扭矩矩常数
        float Kv; // 转速常数(空载)                         //一般不用

        uint16_t MAX_output; // 发给电调的最大控制量        //用宏?
    } M_IP;

    struct intrinsic_parameter
    {
        float T0; // 固有力矩,对yaw轴基本为0,而pitch就为重力矩
        float J;  // 输出转动惯量
    };

    struct estimate_parameter // 预测变量
    {
        float Omega; // 角速度
        float Tq;    // 力矩
        float Um;    // 电机两端电压
        float Id;    // 电机电流

        float eta; // 电机输出效率(排除相电阻、电感)(堵转时为零)
        float TqE; // 外部干扰力矩,如人手故意去掰、云台加/减弹、底盘靠摩擦力矩带动运动等...

        float Kv;
    } est;

} motor_observer_t;

#endif
