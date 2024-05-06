#include "usart.h"
#include "./base_drv/drv_uart.h"
#include "./base_drv/drv_rc/rc.h"
#include "./base_drv/drv_motor/motor.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/weapon/shooter.h"
#include "./robot_core/weapon/vision.h"
#include gimbal_module_core_h
#include gimbal_module_motor_ctrl_h
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/weapon/shooter.h"
#include "./robot_core/tool/vofa.h"

void show_data_to_vofa(void)
{
    extern gimbal_motors_t gimbal_motors;
    // 0
    _printf("%f,", fc_real_abs_angle);
		
    _printf("\r\n");
}

#define host_uart huart1

uint8_t rx_buffer[32] = {0};
void host_init(void)
{
    // uart_recv_dma_init(&host_uart, rx_buffer, 32);
}
// 原理：
// SendBuff数组做缓存，存放要打印的字符串，然后再由Uart或其他方式发送出去

#include "stdarg.h"
#include "stdio.h"
#include "string.h" //可以拿来调用memcpy或memset

#define buf_len 128 // 尽量大些,否则会溢出
uint8_t uart_sendbuff[buf_len] = {0};

int _printf(const char *format, ...)
{
    static uint16_t rv_len = 0;
    memset(uart_sendbuff, 0, rv_len);
    va_list arg;
    va_start(arg, format);
    rv_len = vsnprintf((char *)uart_sendbuff, sizeof(uart_sendbuff), (char *)format, arg);
    va_end(arg);

    if (rv_len > buf_len)
        rv_len = buf_len;
    if (host_uart.gState != HAL_UART_STATE_READY)
        return 0;
    HAL_UART_Transmit(&host_uart, uart_sendbuff, rv_len, 0xff);
    return rv_len;
}
