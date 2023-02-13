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

int main(void)
{
	uint32_t ms_count = 0;
	uint8_t ukey_code;
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

	while (1)
	{
		HAL_Delay(1);
		ms_count++;
		if(ms_count % 10 == 0) {
			device_key_scan_10ms();
		}

		ukey_code = device_get_key();
		if (ukey_code != KEY_NONE) {
			switch (ukey_code)
			{
			case KEY_1_DOWN:
				switch_led_on(&g_switch_led_devs[0]);
				break;
			case KEY_1_UP:
				switch_led_on(&g_switch_led_devs[0]);
				break;
			case KEY_1_LONG:
				switch_led_off(&g_switch_led_devs[0]);
				break;
			case KEY_2_DOWN:
				switch_led_toggle(&g_switch_led_devs[0]);
				break;
			case KEY_2_UP:
				switch_led_toggle(&g_switch_led_devs[0]);
				break;
			case KEY_2_LONG:
				switch_led_toggle(&g_switch_led_devs[0]);
				break;
			case KEY_3_DOWN:
				switch_led_toggle(&g_switch_led_devs[1]);
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
}
