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
	driver_tim_config_t config = {
		.mode = MODE_PERIOD,
		.cycle = 1000,
		.time_uint = UNIT_1MS,
		.task = tim01_task,
	};
	driver_tim_base_config(DRIVER_TIM1, config);
	driver_tim_base_start(DRIVER_TIM1);
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
