#ifndef _MEM_LOG_H
#define _MEM_LOG_H

#include "./base_drv/drv_conf.h"

#define LOG_BUFFER_LEN 128
#define TMP_BUFFER_LEN 32

#define LOG_MEM_ADDR 0X20001000

void mem_log_print(const char *str, const char level);
void clear_mem_log(unsigned int reset_all);

#if _DEBUG == 1
#define LOG_PRINT(MSG, LEVEL) mem_log_print(MSG, LEVEL)
#define CLEAR_LOG(F) clear_mem_log(F)
#else
#define LOG_PRINT(MSG, LEVEL) ((void)0)
#define CLEAR_LOG(F) ((void)0)
#endif

#endif
