#include "./robot_core/tool/bus_detect.h"

// 检查数据刷新速率的函数
uint32_t update_date_refresh_freq(rfreq_t *s_rf, float data_curr)
{
    if (data_curr != s_rf->data_last)
    {
        s_rf->data_last = data_curr;
        s_rf->refresh_rate = 1000.f / (HAL_GetTick() - s_rf->tick_start);
        s_rf->tick_start = HAL_GetTick();
    }
    return s_rf->refresh_rate;
}
