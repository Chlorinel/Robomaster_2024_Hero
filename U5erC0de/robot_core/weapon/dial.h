#ifndef _dial_h_
#define _dial_h_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE

typedef enum {
  disable = 0,
  eatting,
  feeding,
  lock,
  no_bullet,
} dial_mode_t;

dial_mode_t get_get_dial_expt_mode(void);
void dial_init(void);
void dial_control_loop(void);
#endif
