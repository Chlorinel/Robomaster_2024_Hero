#ifndef _zero_new_bee_filter_h_
#define _zero_new_bee_filter_h_
#include "./base_drv/drv_conf.h"
#include HAL_INCLUDE

#define DATA_SIZE 20
typedef struct
{
    float A;
    float omega;
    float phi;
    float learning_rate;
    struct
    {
        float raw_yaw;
        uint32_t tick;
    } list[DATA_SIZE];
} sin_data_t;
typedef struct
{
    float track_yaw;
    uint8_t track_armor_num; // 视野中的为0,逆时针排序为1、2
} outpost_armor_t;

void write_sensor_data(sin_data_t *data, float track_yaw, uint32_t tick);
void update_sin_data(sin_data_t *data);
void calculate_outpost_yaw(outpost_armor_t *outpost, bool is_armor_jump, float new_yaw_data);
#endif