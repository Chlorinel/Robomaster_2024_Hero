#ifndef VOFA_H
#define VOFA_H
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE

void show_data_to_vofa(void);

void host_init(void);

int _printf(const char *format, ...);
#endif