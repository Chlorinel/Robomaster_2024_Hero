#include "./algorithm/filter.h"
#include "./base_drv/drv_motor/motor.h"
#include "./base_drv/drv_imu/imu.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/ctrl_core/gimbal_motor_ctrl.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern const _Motor DJI_motor_lib_M2006;
extern const _Motor DJI_motor_lib_M3508;
extern const _Motor DJI_motor_lib_GM6020;

/**
 * @brief	电机型号初始化函数
 * @param	p_motor:要被初始化的电机
 * @param	motor_model:电机实际型号,DJI旗下的就填motor_DJI_M2006、motor_DJI_M3508、motor_DJI_GM6020
 * @param	kpid:pid设定值
 * @param	ID:电机ID号,直接读灯闪烁次数可知
 * @note	控制类型为leso的电机leso部分需额外初始化,ladrc则需要单独初始化
 */
void motor_init(
    motor_t *p_motor,
    DJI_motor_model_t motor_model,
    uint8_t ID,
    void *param_k_pos,
    void *param_k_vel)
{
    p_motor->is_offline = true;
    uint32_t output_max = 0;
    switch (motor_model)
    {
    case motor_DJI_M2006:
        if (ID > 4)
            p_motor->motor_type = DJI_motor_lib_M2006.H;
        else
            p_motor->motor_type = DJI_motor_lib_M2006.L;
        output_max = C610_OUTPUT_MAX;
        break;
    case motor_DJI_M3508:
        if (ID > 4)
            p_motor->motor_type = DJI_motor_lib_M3508.H;
        else
            p_motor->motor_type = DJI_motor_lib_M3508.L;
        output_max = C620_OUTPUT_MAX;
        break;
    case motor_DJI_GM6020:
        if (ID > 4)
            p_motor->motor_type = DJI_motor_lib_GM6020.H;
        else
            p_motor->motor_type = DJI_motor_lib_GM6020.L;
        output_max = GM6020_OUTPUT_MAX;
        break;
    default:
        return;
    }
    p_motor->motor_type.offset_ID = ID;
    p_motor->real.last_raw_scale = 0;
    p_motor->real.abs_angle = 0;

    switch (p_motor->ctrl_mode)
    {
    case single_pid_ctrl:
        if (param_k_pos != NULL)
            pid_init(&(p_motor->pid[POS_LOOP]), (float *)param_k_pos, output_max / 50, output_max, 0);
        if (param_k_vel != NULL)
            pid_init(&(p_motor->pid[RPM_LOOP]), (float *)param_k_vel, output_max / 50, output_max, 0);
        break;
    case dual_pid_ctrl:
        pid_init(&(p_motor->pid[POS_LOOP]), (float *)param_k_pos, output_max / 50, output_max, 0);
        pid_init(&(p_motor->pid[RPM_LOOP]), (float *)param_k_vel, output_max / 50, output_max, 0);
        break;
    case dual_pid_leso_ctrl:
        pid_init(&(p_motor->pid_leso.pid[POS_LOOP]), (float *)param_k_pos, output_max / 50, output_max, 0);
        pid_init(&(p_motor->pid_leso.pid[RPM_LOOP]), (float *)param_k_vel, output_max / 50, output_max, 0);
        break;
    case dual_ladrc_ctrl:
        break;
    case single_adrc_ctrl:
#if 0
			if(param_k_pos!=NULL)
			{
				p_motor->adrc[POS_LOOP]=create_ADRC4angle();
  				p_motor->adrc[POS_LOOP].param_init(&(p_motor->adrc[POS_LOOP]), *((ADRC_Init*)param_k_pos) );
			}
			
			if(param_k_vel!=NULL)
			{
				p_motor->adrc[RPM_LOOP]=create_ADRC();
  				p_motor->adrc[RPM_LOOP].param_init(&(p_motor->adrc[RPM_LOOP]), *((ADRC_Init*)param_k_vel) );
			}
#endif
        break;
    }
    LPF_init(&p_motor->lpf_fltr, (float)(1.f / fs_tim_freq), fs_tim_freq / 2);
}