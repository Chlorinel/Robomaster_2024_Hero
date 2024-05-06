#ifndef _BUS_DETECT_
#define _BUS_DETECT_
#include "main.h"
typedef struct
{
    float data_last;
    uint32_t tick_start;
    float refresh_rate;
} rfreq_t;

uint32_t update_date_refresh_freq(rfreq_t *s_rf, float data_curr);

#define get_data_refresh_freq(_DAT, freq)              \
    do                                                 \
    {                                                  \
        static rfreq_t rfreq = {0};                    \
        freq = update_date_refresh_freq(&rfreq, _DAT); \
    } while (0)
#define force_refresh_freq(p_DAT, freq)

#endif