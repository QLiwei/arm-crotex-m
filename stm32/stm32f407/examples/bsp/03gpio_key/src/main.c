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
		HAL_Delay(1000);
		switch_led_on(&g_switch_led_devs[0]);
		switch_led_off(&g_switch_led_devs[1]);
		switch_led_on(&g_switch_led_devs[2]);
		HAL_Delay(1000);
		switch_led_off(&g_switch_led_devs[0]);
		switch_led_on(&g_switch_led_devs[1]);
		switch_led_off(&g_switch_led_devs[2]);
	}
}
