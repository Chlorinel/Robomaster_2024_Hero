#include "./algorithm/util.h"
#include "./base_drv/drv_motor/motor.h"
#include "./robot_core/ctrl_core/move_ctrl.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include chassis_module_core_h
#include chassis_module_motor_ctrl_h

extern chassis_motors_t chassis_motors;
/**
 *  @brief  麦轮正向解算,通过更新底盘电机编码器进行车轮位置的采集
 *  @note   此部分功能类似云台的imu,我就放到2.5khz的高速定时器去了(
 */

chassis_state_t *mecanum_forward(void)
{
    static uint32_t last_tick = 0;
    if (last_tick == 0)
        last_tick = HAL_GetTick();

    static chassis_state_t wheel_state = {0};

    if (HAL_GetTick() - last_tick >= 1)
    {
        float delta_time = 0.001f * (HAL_GetTick() - last_tick);
        // 手动怼墙,自己手动计算出的比例
#define k_x 0.63553696351014821756773599807572f
#define k_y 0.6f
#define k_wz1 1.5776793600701299612416560660435f

        float _delta_ang[4] = {0}; // 车轮角变化
        float delta_l[4] = {0};    // 车轮线变化
        float delta_x, delta_y, delta_yaw;
        static float last_rel_angle[4] = {0}; // 减少内存占用
        float R_wheel = (9.549296f / R_McEnham) / 1000.f;
        for (uint8_t i = 0; i < 4; i++)
        {
            _delta_ang[i] = get_delta_ang(
                chassis_motors._all_chassis_motors[i].real.rel_angle,
                last_rel_angle[i],
                2 * PI);
            delta_l[i] = _delta_ang[i] * R_wheel;
            last_rel_angle[i] = chassis_motors._all_chassis_motors[i].real.rel_angle;
        }
        delta_x = (delta_l[0] - delta_l[1] + delta_l[2] - delta_l[3]) / 4.f * k_x;
        delta_y = (delta_l[0] - delta_l[2] + delta_l[1] - delta_l[3]) / 4.f * k_y;
        delta_yaw = (delta_l[0] + delta_l[1] + delta_l[2] + delta_l[3]) / 4.f;
        float theta = 0;//chassis_info.expt_delta_yaw; // <<屎

        // 计算累加位移
        wheel_state.vx = (delta_x * cos(theta) - delta_y * sin(theta)) / delta_time;
        wheel_state.x += wheel_state.vx * delta_time;
        wheel_state.vy = -(delta_y * cos(theta) + delta_x * sin(theta)) / delta_time;
        wheel_state.y += wheel_state.vy * delta_time;
        wheel_state.wz = -delta_yaw * k_wz1 / delta_time;

        wheel_state.c_yaw += wheel_state.wz * delta_time;
        last_tick = HAL_GetTick();
    }

    return &wheel_state;
}

/**
 * @brief   获得四轮模式下该转向的角速度
 */

float move_dirc_rad; // 移动速度的方向          	//右手半角
float deadband_deg = 10;
float k_four_wheeler = 1;
float get_four_wheeler_wz(chassis_state_t *gimbal_expt_state, chassis_state_t *chassis_real_state)
{
    move_dirc_rad = atan2(gimbal_expt_state->vy, gimbal_expt_state->vx);
    move_dirc_rad = range_map(move_dirc_rad, -PI, PI);

    // 获得底盘相对于云台的前进方向	//右手半角
    float chassis_dirc_rad = 0; // range_map(chassis_real_state->yaw, -PI, PI);
    // 获得底盘相对于云台的后退方向  //右手半角
    float opposite_chassis_dirc_rad = PI; // range_map(chassis_dirc_rad + PI, -PI, PI);

    // 判断哪边靠近期望移动的方向,并获得角度差用于位置控制
    float trace_dirc = 0; // 最终确定的要跟随的方向
    if (fabs(get_delta_ang(move_dirc_rad, chassis_dirc_rad, 2 * PI)) <
        fabs(get_delta_ang(move_dirc_rad, opposite_chassis_dirc_rad, 2 * PI)))
    {
        trace_dirc = chassis_dirc_rad;
    }
    else
        trace_dirc = opposite_chassis_dirc_rad;

    // 移动速度与跟随方向的夹角
    float delta_ang = get_delta_ang(move_dirc_rad, trace_dirc, 2 * PI);

    extern pid_struct_t chassis_follow_pid;
    float vx_theshold = 0.01;
    float vy_theshold = 0.01;

    bool is_expt_stop = (fabs(gimbal_expt_state->vx) < vx_theshold) && (fabs(gimbal_expt_state->vy) < vy_theshold);

    float expt_wz = 0;
    if (!is_expt_stop)
    {
        expt_wz =
            -pid_calc(&chassis_follow_pid, 0, NL_Deadband(delta_ang, deg2rad(deadband_deg), 3)) * k_four_wheeler;
    }
    else
        expt_wz = 0;
    return expt_wz;
}
