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
