#ifndef _DRV_CONF_H
#define _DRV_CONF_H

#define HAL_INCLUDE "stm32f4xx_hal.h" ///< define HAL header
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

#define gimbal_module_core_h "./robot_core/ctrl_core/gimbal.h"
#define gimbal_module_motor_ctrl_h "./robot_core/ctrl_core/gimbal_motor_ctrl.h"
#define chassis_module_core_h "./robot_core/ctrl_core/chassis.h"
#define chassis_module_motor_ctrl_h                                            \
  "./robot_core/ctrl_core/chassis_motor_ctrl.h"
#if defined(__CC_ARM)
#pragma anon_unions
#endif

#define enable_watch_dog false
/**
 * CAN与FIFO分配:
 * hcan1.FIFO0.filternum0->底盘四电机
 * hcan1.FIFO1.filternum12->超级电容
 * hcan1.->拨盘电机
 * hcan2.FIFO1.filternum0->fl、fr、fc、pr
 */

// 可删
#define THIS_IS_GIMBAL
// #define THIS_IS_CHASSIS

#include "can.h"

#define CHASSIS_MOTOR_CAN hcan1
#define WHEELS_CAN hcan1
#define WHEELS_FIFO CAN_FILTER_FIFO0
#define WHEELS_FLTRNUM 0

#define YAW_AND_DIAL_CAN hcan1
#define YAW_AND_DIAL_FIFO CAN_FILTER_FIFO0
#define YAW_AND_DIAL_FLTRNUM 8

#define CAP_CAN hcan1
#define CAP_FIFO CAN_FILTER_FIFO0
#define CAP_FLTRNUM 12

#define GIMBAL_MOTOR_CAN hcan2
#define MOTOR_FIFO CAN_FILTER_FIFO1
#define MOTOR_FLTRNUM 0
#define FIRL_MOTOR_FLTRNUM (MOTOR_FLTRNUM + 4)

#define MAX_V 4.f

#define MAX_MOVE_SPEED 3.8
#define MAX_SPIN_SPEED 1.53

#define SELF_POWER_CTRL 0
#define CAP_POWER_CTRL 1

#define APB1_freq 84000000.0f
#define PI (3.14159265358979323846f)
/*
RAM_PERSIST宏为变量内存持久化的修饰符
某些关键变量在非上电复位时需要保持，一般初始化代码内在复位后会把所有变量清空，该修饰符可阻止编译器清空变量
未在KEIL中的sct文件增加无初始化段，或者启动代码内没有判断复位类型而不处理变量初始化全部清空，该宏无效可注释掉
*/
// #define RAM_PERSIST __attribute__((section("NO_INIT"),zero_init))  ///< marco
// to avoid variable initialized at reset

#ifndef RAM_PERSIST
#define RAM_PERSIST ///< disabled RAM_PERSIST
#endif

#define USE_RC_UART 1 ///< select if we use remote control uart(0 or 1)

#if USE_RC_UART == 1
/** @name remote control config
 * @note Usage: \n
 *       1.add rc_uart_idle_handle(&huartx) to stm32fxx_it.c(before IRQ_Handler)
 * \n 2.MX_USARTX_UART_Init() \n 3.rc_recv_dma_init() \n
 * @{
 */
#define RC_UART USART3 ///< select which uart remote control dbus uses (USARTx)
#define RC_UART_HANDLE                                                         \
  huart3 ///< select which uart handle remote control dbus uses (huartx)
/** @} remote control config */
#define REFEREE_UART                                                           \
  USART6 ///< select which uart referee system feedback uses (USARTx)
#define REFEREE_UART_HANDLE                                                    \
  huart6 ///< select which uart handle referee system feedback uses (huartx)\
#define REFEREE_ASYNC_SEND 0        ///< select if referee send use uart_send_async(0 or 1)

#endif

#define USE_VT_RC_UART 1
/** @name VT remote control config
 * @note Usage: \n
 *       1.add rc_uart_idle_handle(&huartx) to stm32fxx_it.c(before IRQ_Handler)
 * \n 2.MX_USARTX_UART_Init() \n 3.vt_rc_recv_dma_init() \n
 * @{
 */
#if USE_VT_RC_UART == 1
#define VT_RC_UART USART1
#define VT_RC_UART_HANDLE huart1
#endif

/*CAN等待发送完成超时，单位毫秒，设为0即无阻塞的发送*/
#define CAN_TX_TIMEOUT 0      ///< timeout in ms, set to 0 to disable
#define C620_OUTPUT_MAX 16384 ///< C620 maximum output(range:-20~20A)
#define C610_OUTPUT_MAX 10000
#define GM6020_OUTPUT_MAX 30000 ///< GM6020 maximum output
#define ANGLE_RANGE 8192        ///< range of motor raw angle(13bit data)
#define CAN_DATA_LEN 8          ///< can frame length

#endif
