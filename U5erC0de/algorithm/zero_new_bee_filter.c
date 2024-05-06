#include "./algorithm/util.h"
#include "./algorithm/filter.h"
#include "./algorithm/zero_new_bee_filter.h"

#include <math.h>

// Function to calculate the error between the original data and the fit
float calculate_error(sin_data_t *data)
{
    double error = 0.0;
    float A = data->A;
    float omega = data->omega;
    float phi = data->phi;
    for (uint16_t i = 0; i < DATA_SIZE; i++)
    {
        float x = data->list[i].tick / 1000.f;
        float y = data->list[i].raw_yaw;

        float y_fit = A * sin(omega * x + phi);
        float diff = y_fit - y;
        error += diff * diff;
    }

    return error;
}

void write_sensor_data(sin_data_t *data, float track_yaw, uint32_t tick)
{
    // 找最久远的数据来写
    uint16_t min_i = (uint16_t)-1;
    uint32_t min_tick = (uint32_t)-1;
    for (uint16_t i = 0; i < DATA_SIZE; i++)
    {
        if (data->list[i].tick < min_tick)
        {
            min_tick = data->list[i].tick;
            min_i = i;
        }
    }
    data->list[min_i].tick = tick;
    data->list[min_i].raw_yaw = track_yaw;
}

float grad_A = 0.0;
float grad_omega = 0.0;
float grad_phi = 0.0;

void update_sin_data(sin_data_t *data)
{

    float error = calculate_error(data);
    float learning_rate = data->learning_rate;

    float A = data->A;
    float omega = data->omega;
    float phi = data->phi;

    // Compute the gradients for each parameter

    for (int i = 0; i < DATA_SIZE; i++)
    {
        float x = data->list[i].tick / 1000.f;
        // float y = data->list[i].raw_yaw;

        grad_A += 2 * error * sin(omega * x + phi);
        grad_omega += 2 * error * A * x * cos(omega * x + phi);
        grad_phi += 2 * error * A * cos(omega * x + phi);
    }

    // Update parameters using gradient descent
    data->A -= learning_rate * grad_A;
    data->omega -= learning_rate * grad_omega;
    data->phi -= learning_rate * grad_phi;
}
/**
 *  @brief  输入一个装甲板的yaw,连续调用后即可持续追踪此块装甲板的yaw
 *  @param  new_yaw_data:原始yaw
 * */
LPF_t outpost_armor_filter = {.ts=0.001,.fc=1};
bool had_armor_jump = false;
void calculate_outpost_yaw(outpost_armor_t *outpost, bool is_armor_jump, float new_yaw_data)
{
    if (is_armor_jump == true &&
        had_armor_jump == false)
    {
        outpost->track_armor_num++;
        if (outpost->track_armor_num >= 3)
            outpost->track_armor_num = 0;
        had_armor_jump = true;
    }
    if (is_armor_jump == false)
    {
        had_armor_jump = false;

    }

    float curr_outpost_yaw = new_yaw_data + outpost->track_armor_num * 2 * PI / 3;
    curr_outpost_yaw = range_map(curr_outpost_yaw, -PI, PI);
    float yaw_diff = get_delta_ang(curr_outpost_yaw, outpost->track_yaw, 2 * PI);
    yaw_diff = LPF_update(&outpost_armor_filter, yaw_diff);
    outpost->track_yaw += yaw_diff;
    range_map(outpost->track_yaw, -PI, PI);
}