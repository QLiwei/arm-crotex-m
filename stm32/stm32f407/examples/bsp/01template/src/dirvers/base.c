/**
 * @file base.c
 * @brief base
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */

#include "base.h"

/**
 * @brief Initialize the system clock
 *
 * @note
 *  System Clock source            = PLL (HSE)
 *  SYSCLK(Hz)                     = 168000000 (CPU Clock)
 *  HCLK = SYSCLK / 1              = 168000000 (AHB1Periph)
 *  PCLK2 = HCLK / 2               = 84000000  (APB2Periph)
 *  PCLK1 = HCLK / 4               = 42000000  (APB1Periph)
 *  HSE Frequency(Hz)              = 8000000
 *  PLL_M                          = 8
 *  PLL_N                          = 336
 *  PLL_P                          = 2
 *  PLL_Q                          = 4
 *  VDD(V)                         = 3.3
 *  Flash Latency(WS)              = 5
 *
 */
void system_clock_config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;


	/* 芯片内部的LDO稳压器输出的电压范围，选用的PWR_REGULATOR_VOLTAGE_SCALE1 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* 使能HSE，并选择HSE作为PLL时钟源 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
        //Error_Handler(__FILE__, __LINE__);
	}

	/*
       选择PLL的输出作为系统时钟
		HCLK = SYSCLK / 1  (AHB1Periph)
		PCLK2 = HCLK / 2   (APB2Periph)
		PCLK1 = HCLK / 4   (APB1Periph)
    */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	/* 此函数会更新SystemCoreClock，并重新配置HAL_InitTick */
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
        //Error_Handler(__FILE__, __LINE__);
	}

    /* 使能SYS时钟和IO补偿 */
	__HAL_RCC_SYSCFG_CLK_ENABLE() ;

	HAL_EnableCompensationCell();
}
