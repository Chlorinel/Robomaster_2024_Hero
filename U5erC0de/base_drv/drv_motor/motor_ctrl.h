#ifndef _motor_ctrl_h_
#define _motor_ctrl_h_

#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE
/**
 * @ struct motor_data_t
 * @brief the struct that stores motor data receive from CAN
 */
typedef struct
{
	uint16_t raw_scale;		 //< 本次电机磁编码器输出值,取值0~8192
	uint16_t last_raw_scale; //< 上一次电机磁编码器输出值,取值0~8192
	float rel_angle;		 //< 电机实际输出端距原点的绝对弧度值,
							 //< 原点默认为开机处的位置,调整需调用set_motor_mid_position函数
	float abs_angle;		 //< 电机实际输出端距原点的相对弧度值,取值0~2*PI
	int16_t speed_rpm;		 //< 电机旋转速度
	int16_t current;		 //< 电机输出电流
	uint8_t tempture;		 //< 电机内部温度
} motor_data_t;

/**
 * @struct motor_transmit_msg_t
 * @brief the struct stores motor data for transmission
 */
typedef struct
{
	int16_t D[4]; ///< the array of 4 16bit data
} motor_transmit_msg_t;

void update_gimbal_motor_status(void);
void update_chassis_motor_status(void);
void pitch_filter_init(void);
HAL_StatusTypeDef set_all_gimbal_motor_output(void);
HAL_StatusTypeDef set_all_chassis_motor_output(void);
HAL_StatusTypeDef gimbal_parse_motor_data(CAN_RxHeaderTypeDef *header, uint8_t *rx_buffer);
HAL_StatusTypeDef chassis_parse_motor_data(CAN_RxHeaderTypeDef *header, uint8_t *rx_buffer);
HAL_StatusTypeDef set_motor_output(CAN_HandleTypeDef *hcan, motor_transmit_msg_t *tx_msg, uint32_t id);

#endif
