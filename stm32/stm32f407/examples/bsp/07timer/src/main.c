/**
 * @file main.c
 * @brief Main function Population
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#include "main.h"

void key_handle(void);

void tim01_task(void *parameter) {
	switch_led_toggle(DEVICE_LED01);
}

int main(void)
{
	/*
	   STM32H407 HAL 库初始化，此时系统用的还是F407自带的16MHz，HSI时钟:
	   - 调用函数HAL_InitTick，初始化滴答时钟中断1ms。
	   - 设置NVIV优先级分组为4。
	 */
	HAL_Init();
	/*
	   配置系统时钟到168MHz
	   - 切换使用HSE。
	   - 此函数会更新全局变量SystemCoreClock，并重新配置HAL_InitTick。
	*/
	system_clock_config();

	system_init();

	soft_timer_set(ST_KEY_SCAN_TASK_INDEX, ST_CYCLE_10MS, ST_INFINIT_REPEAT, device_key_scan_10ms);
	soft_timer_set(ST_BEEP_TASK_INDEX, ST_CYCLE_10MS, ST_INFINIT_REPEAT, device_beep_handle);
	soft_timer_set(ST_KEY_HANDLE_TASK_INDEX, ST_CYCLE_1MS, ST_INFINIT_REPEAT, key_handle);
	drv_tim_config_t config = {
		.mode = MODE_PERIOD,
		.cycle = 1000,
		.time_uint = UNIT_1MS,
		.task = tim01_task,
	};
	drv_tim_base_config(DRV_TIM1, config);
	drv_tim_base_start(DRV_TIM1);

	drv_pwm_out_config_t pwm2_config = {
		.mode = PWM_MODE1,
		.frequency = 15000,
		.default_duty_cycle = 0,
		.channel_enable[0] = true,
		.channel_enable[1] = true,
		.channel_enable[2] = true,
		.channel_enable[3] = true,
	};
	drv_pwm_out_config(DRV_TIM2, pwm2_config);
	drv_pwm_out_duty_set(DRV_TIM2, PWM_CHANNEL_1, 0);
	drv_pwm_out_duty_set(DRV_TIM2, PWM_CHANNEL_2, 5000);
	drv_pwm_out_duty_set(DRV_TIM2, PWM_CHANNEL_3, 7500);
	drv_pwm_out_duty_set(DRV_TIM2, PWM_CHANNEL_4, 10000);
	drv_pwm_out_start(DRV_TIM2, PWM_CHANNEL_1);
	drv_pwm_out_start(DRV_TIM2, PWM_CHANNEL_2);
	drv_pwm_out_start(DRV_TIM2, PWM_CHANNEL_3);
	drv_pwm_out_start(DRV_TIM2, PWM_CHANNEL_4);
	while (1)
	{
		soft_timer_run();
	}
}

void key_handle(void) {
	uint8_t ukey_code;

	ukey_code = device_get_key();
	if (ukey_code != KEY_NONE) {
		switch (ukey_code)
		{
		case KEY_1_DOWN:
			device_beep_key_tone(0);
			break;
		case KEY_1_UP:
			switch_led_on(DEVICE_LED01);
			com_send_buf(COM1, "TEST", 4);
			break;
		case KEY_1_LONG:
			switch_led_off(DEVICE_LED01);
			break;
		case KEY_2_DOWN:
			device_beep_start(0,5,5,10);
			break;
		case KEY_2_UP:
			switch_led_toggle(DEVICE_LED01);
			break;
		case KEY_2_LONG:
			switch_led_toggle(DEVICE_LED01);
			break;
		case KEY_3_DOWN:
			device_beep_start(0,50,50,3);
			break;
		case KEY_3_UP:
			switch_led_toggle(DEVICE_LED02);
			break;
		case KEY_3_LONG:
			switch_led_toggle(DEVICE_LED02);
			break;
		case KEY_4_DOWN:
			switch_led_toggle(DEVICE_LED03);
			break;
		case KEY_4_UP:
			switch_led_toggle(DEVICE_LED03);
			break;
		case KEY_4_LONG:
			switch_led_toggle(DEVICE_LED03);
			break;

		default:
			break;
		}
	}
}
