#include "imu.h"
#include "tim.h"
#include "imu.h"
#include "./algorithm/filter.h"
#include "./robot_core/interface/interface_BTB.h"
#include gimbal_module_motor_ctrl_h
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/tool/utils/trice.h"
#define TRICE_FILE Id(0)

imu_data_fp_t imu_real_info = {0};
eular_t _imu_eular = {0};
#if 0
	顺时针yaw变小		逆时针yaw变大
	抬头roll变低		低头roll变大
	右倾pitch变低		左倾pitch变大
#endif

void imu_init(void)
{
	htim7.Instance->PSC = 400 - 1;
	htim7.Instance->ARR = APB1_freq / 1000 / 1000 - 1;

	extern float hs_tim_freq;
	hs_tim_freq = APB1_freq / ((htim7.Instance->ARR + 1) * (htim7.Instance->PSC + 1));

	int8_t rslt;
	if ((RCC->CSR & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)))
	{
		TRICE(Id(53643), "MSG: POR/BOR startup, wait for 50ms for settle\n");
		bmi08x_delay_us(50000, 0); // 50ms for POR/BOR startup
	}
	uint32_t imu_init_cnt = 0;
	do
	{
		bmi08x_delay_us(3000, 0); // 3ms startup
		imu_init_cnt++;
		TRICE(Id(58105), "ERR:Err[%d] init_bmi08x() failed\n", rslt);
		rslt = init_bmi08x();
	} while (rslt != BMI08X_OK);

	TRICE(Id(59642), "MSG: bmi088 init successfully\n");
	enable_bmi08x_interrupt();

	__enable_irq();

	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_Delay(200);
	HAL_TIM_Base_Start_IT(&htim7);
}

void imu_loop(void)
{
	static gimbal_state_t last_imu_rpy = {0};
	static gimbal_state_t last_motor_rpy = {0};

	int8_t imu_ret = get_imu_att(&imu_real_info, &_imu_eular);
	switch (imu_ret) // 为阻塞型的普通SPI读取
	{
	case BMI08X_OK:
		robot.IMU_status = online;
		break;
	case IMU_DATA_NOT_RDY:
		robot.IMU_status = online;
		break;
	case IMU_CONF_ERR:
		robot.IMU_status = offline;
		break;
	case IMU_ACCEL_ERR:
		robot.IMU_status = offline;
		break;
	default:
		robot.IMU_status = offline;
		break;
	}
	if (robot.IMU_status == online)
	{
		gimbal_state_t imu_rpy = {
			.roll = -_imu_eular.pitch,
			.pitch = _imu_eular.roll,
			.yaw = _imu_eular.yaw};
		clone(last_imu_rpy, imu_rpy);

		update_gimbal_real_state(&imu_rpy);
		// 获取最后一刻的电机系
		gimbal_state_t null_rpy = {0};
		clone_datfptr(last_motor_rpy, fetch_robot_coordinate_system(&null_rpy));

		
		// 发送云台imu数据,用于死后电机回正
	}
	else
	{
		// 电机解算惯性系	=	最后一刻惯性系+电机坐标系差值
		//				   =   电机坐标系+最后一刻惯性系-最后一刻电机系

		gimbal_state_t offset_state = {
			.roll = last_imu_rpy.roll,
			.pitch = range_map(last_imu_rpy.pitch - last_motor_rpy.pitch, -PI, PI),
			.yaw = range_map(last_imu_rpy.yaw - last_motor_rpy.yaw, -PI, PI)};

		update_gimbal_real_state(fetch_robot_coordinate_system(&offset_state));
	}
}