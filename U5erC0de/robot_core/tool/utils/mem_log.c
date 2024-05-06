#include "./robot_core/tool/utils/mem_log.h"
#include <string.h>

char log_buffer[LOG_BUFFER_LEN] __attribute__((at(LOG_MEM_ADDR)));

static unsigned int current_index = 0;
static unsigned int uart_log_index = 0;

void mem_log_print(const char *str, const char level)
{
	char tmp[TMP_BUFFER_LEN];
	unsigned int len = 0;
	while (str[len] != '\0')
	{
		len++;
		if (len > TMP_BUFFER_LEN - 3)
			return;
	}
	if (len + current_index + 4 > LOG_BUFFER_LEN)
	{
		memset(log_buffer, 0x0, LOG_BUFFER_LEN);
		current_index = 0;
	}
	tmp[0] = level;
	tmp[1] = ':';
	memcpy(&tmp[2], str, len);
	tmp[len + 2] = '\n';
	memcpy(&log_buffer[current_index], tmp, len + 3);
	current_index += (len + 3);
}

void clear_mem_log(unsigned int reset_all)
{
	if (reset_all)
	{
		memset(log_buffer, 0x0, LOG_BUFFER_LEN);
		current_index = 0;
	}
	else
	{
		log_buffer[LOG_BUFFER_LEN - 1] = '\0';
	}
}

void uart_sync_log(void)
{
	if (uart_log_index > current_index)
		uart_log_index = 0;
	while (uart_log_index < current_index && uart_log_index < LOG_BUFFER_LEN)
	{

		// TODO: implement uart send char process(with time out detect)
		// while((USARTx->SR & USART_SR_TXE) == 0);
		// UARTx->DR = log_buffer[uart_log_index];
		if (log_buffer[uart_log_index] == '\0')
		{
			uart_log_index = 0;
			return;
		}
		uart_log_index++;
	}
}
