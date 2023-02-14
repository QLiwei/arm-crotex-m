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

int main(void)
{
	/*
	   STM32H407 HAL ���ʼ������ʱϵͳ�õĻ���F407�Դ���16MHz��HSIʱ��:
	   - ���ú���HAL_InitTick����ʼ���δ�ʱ���ж�1ms��
	   - ����NVIV���ȼ�����Ϊ4��
	 */
	HAL_Init();
	/*
	   ����ϵͳʱ�ӵ�168MHz
	   - �л�ʹ��HSE��
	   - �˺��������ȫ�ֱ���SystemCoreClock������������HAL_InitTick��
	*/
	system_clock_config();

	system_init();

	soft_timer_set(ST_KEY_SCAN_TASK_INDEX, ST_CYCLE_10MS, ST_INFINIT_REPEAT, device_key_scan_10ms);
	soft_timer_set(ST_BEEP_TASK_INDEX, ST_CYCLE_10MS, ST_INFINIT_REPEAT, device_beep_handle);
	soft_timer_set(ST_KEY_HANDLE_TASK_INDEX, ST_CYCLE_1MS, ST_INFINIT_REPEAT, key_handle);
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
			switch_led_on(&g_switch_led_devs[0]);
			break;
		case KEY_1_LONG:
			switch_led_off(&g_switch_led_devs[0]);
			break;
		case KEY_2_DOWN:
			device_beep_start(0,5,5,10);
			break;
		case KEY_2_UP:
			switch_led_toggle(&g_switch_led_devs[0]);
			break;
		case KEY_2_LONG:
			switch_led_toggle(&g_switch_led_devs[0]);
			break;
		case KEY_3_DOWN:
			device_beep_start(0,50,50,3);
			break;
		case KEY_3_UP:
			switch_led_toggle(&g_switch_led_devs[1]);
			break;
		case KEY_3_LONG:
			switch_led_toggle(&g_switch_led_devs[1]);
			break;
		case KEY_4_DOWN:
			switch_led_toggle(&g_switch_led_devs[2]);
			break;
		case KEY_4_UP:
			switch_led_toggle(&g_switch_led_devs[2]);
			break;
		case KEY_4_LONG:
			switch_led_toggle(&g_switch_led_devs[2]);
			break;

		default:
			break;
		}
	}
}