#ifndef _BUZZER_H_
#define _BUZZER_H_
#include "./base_drv/drv_conf.h"

void buzzer_init(void);
void buzzer_proc(void);
void beep(uint32_t cycle, uint32_t cnt,float _volume);
#endif