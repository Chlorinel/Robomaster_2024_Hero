#include "./base_drv/drv_buzzer/buzzer.h"
#include "tim.h"

extern TIM_HandleTypeDef htim4;

uint32_t beep_cycle = 1;
uint32_t beep_cnt_loaded = 114514;
void buzzer_init(void)
{
	htim4.Instance->ARR = 250;
	htim4.Instance->PSC = 420;
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
uint8_t volume = 1;
void buzzer_proc(void)
{
	if (beep_cycle > 0)
	{
		static uint32_t tick_start = 0;

		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, ((beep_cycle + 1) % 2) * volume);
		if (HAL_GetTick() - tick_start > beep_cnt_loaded)
		{
			tick_start = HAL_GetTick();
			beep_cycle--;
		}
	}
	else
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
/*
	@param	cycle:循环次数
	@param	cnt:单次循环周期(ms),越小代表事件越紧急
*/
uint16_t a = 240;
void beep(uint32_t cycle, uint32_t cnt, float _volume)
{
	if (beep_cycle == 0 || beep_cnt_loaded > cnt)
	{
		beep_cycle = cycle + 1;
		beep_cnt_loaded = cnt;
		volume = _volume * a;
	}
}
