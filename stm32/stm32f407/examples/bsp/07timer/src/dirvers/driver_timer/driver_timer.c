/**
 * @file driver_timer.c
 * @brief timer common
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-25       vector(vector.qiu@gmail.com)    first version
 */
#include "driver_timer.h"

#define APB2_TIMER_CLK  (168000000)
#define APB1_TIMER_CLK  (84000000)
uint32_t tim_clk_get(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) {
        return APB2_TIMER_CLK;
    } else {
        return APB1_TIMER_CLK;
    }
}

void rcc_tim_gpio_enable(GPIO_TypeDef* GPIOx) {
    if (GPIOx == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
	else if (GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
	else if (GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
	else if (GPIOx == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
	else if (GPIOx == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
	else if (GPIOx == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
	else if (GPIOx == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
	else if (GPIOx == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
	else if (GPIOx == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
    else
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

void rcc_tim_enable(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (TIMx == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (TIMx == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (TIMx == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (TIMx == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
    else if (TIMx == TIM6) __HAL_RCC_TIM6_CLK_ENABLE();
    else if (TIMx == TIM7) __HAL_RCC_TIM7_CLK_ENABLE();
    else if (TIMx == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
    else if (TIMx == TIM9) __HAL_RCC_TIM9_CLK_ENABLE();
    else if (TIMx == TIM10) __HAL_RCC_TIM10_CLK_ENABLE();
    else if (TIMx == TIM11) __HAL_RCC_TIM11_CLK_ENABLE();
    else if (TIMx == TIM12) __HAL_RCC_TIM12_CLK_ENABLE();
    else if (TIMx == TIM13) __HAL_RCC_TIM13_CLK_ENABLE();
    else if (TIMx == TIM14) __HAL_RCC_TIM14_CLK_ENABLE();
    else
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

void rcc_tim_disable(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) __HAL_RCC_TIM3_CLK_DISABLE();
	else if (TIMx == TIM2) __HAL_RCC_TIM2_CLK_DISABLE();
	else if (TIMx == TIM3) __HAL_RCC_TIM3_CLK_DISABLE();
	else if (TIMx == TIM4) __HAL_RCC_TIM4_CLK_DISABLE();
	else if (TIMx == TIM5) __HAL_RCC_TIM5_CLK_DISABLE();
	else if (TIMx == TIM6) __HAL_RCC_TIM6_CLK_DISABLE();
	else if (TIMx == TIM7) __HAL_RCC_TIM7_CLK_DISABLE();
	else if (TIMx == TIM8) __HAL_RCC_TIM8_CLK_DISABLE();
	else if (TIMx == TIM9) __HAL_RCC_TIM9_CLK_DISABLE();
	else if (TIMx == TIM10) __HAL_RCC_TIM10_CLK_DISABLE();
	else if (TIMx == TIM11) __HAL_RCC_TIM11_CLK_DISABLE();
	else if (TIMx == TIM12) __HAL_RCC_TIM12_CLK_DISABLE();
	else if (TIMx == TIM13) __HAL_RCC_TIM13_CLK_DISABLE();
	else if (TIMx == TIM14) __HAL_RCC_TIM14_CLK_DISABLE();
	else
	{
		Error_Handler(__FILE__, __LINE__);
	}
}

uint8_t tim_af_get(TIM_TypeDef* TIMx)
{
	uint8_t af = 0;

	if (TIMx == TIM1) af = GPIO_AF1_TIM1;
	else if (TIMx == TIM2) af = GPIO_AF1_TIM2;
	else if (TIMx == TIM3) af = GPIO_AF2_TIM3;
	else if (TIMx == TIM4) af = GPIO_AF2_TIM4;
	else if (TIMx == TIM5) af = GPIO_AF2_TIM5;
	else if (TIMx == TIM8) af = GPIO_AF3_TIM8;
	else if (TIMx == TIM9) af = GPIO_AF3_TIM9;
	else if (TIMx == TIM10) af = GPIO_AF3_TIM10;
	else if (TIMx == TIM11) af = GPIO_AF3_TIM11;
	else if (TIMx == TIM12) af = GPIO_AF9_TIM12;
	else if (TIMx == TIM13) af = GPIO_AF9_TIM13;
	else if (TIMx == TIM14) af = GPIO_AF9_TIM14;
	else
	{
		Error_Handler(__FILE__, __LINE__);
	}
	return af;
}

void tim_gpio_config(TIM_TypeDef* TIMx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX) {
    GPIO_InitTypeDef   GPIO_InitStruct;

	rcc_tim_gpio_enable(GPIOx);
	rcc_tim_enable(TIMx);

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = tim_af_get(TIMx);
	GPIO_InitStruct.Pin = GPIO_PinX;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
