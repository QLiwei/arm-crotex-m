/**
 * @file driver_timer.h
 * @brief timer driver definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-18       vector(vector.qiu@gmail.com)    first version
 *
 */
#ifndef __DRIVER_TIMER_H__
#define __DRIVER_TIMER_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APB2_TIMER_CLK  (168000000)
#define APB1_TIMER_CLK  (84000000)
inline uint32_t tim_clk_get(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) {
        return APB2_TIMER_CLK;
    } else {
        return APB1_TIMER_CLK;
    }
}

inline void rcc_tim_gpio_enable(GPIO_TypeDef* GPIOx) {
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
        Error_Handler();
    }
}

inline void rcc_tim_enable(TIM_TypeDef* TIMx) {
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
        Error_Handler();
    }
}

typedef enum {
    DRIVER_TIM1 = 0,
    DRIVER_TIM2,
    DRIVER_TIM3,
    DRIVER_TIM4,
    DRIVER_TIM5,
    DRIVER_TIM6,
    DRIVER_TIM7,
    DRIVER_TIM8,
    DRIVER_TIM9,
    DRIVER_TIM10,
    DRIVER_TIM11,
    DRIVER_TIM12,
    DRIVER_TIM13,
    DRIVER_TIM14,
    DRIVER_TIM_NONE,
}driver_tim_id_e;

typedef enum {
    MODE_ONESHOT = 0,
    MODE_PERIOD,
} tim_mode_e;

/* timer base */
typedef void (*time_task)(void);
typedef struct {
    tim_mode_e mode;
    uint32_t cycle; // us
    time_task task;
} driver_tim_config_t;
void driver_tim_config(driver_tim_id_e _id, driver_tim_config_t _config);
void driver_tim_start(driver_tim_id_e _id);
void driver_tim_stop(driver_tim_id_e _id);

/* timer encoder */


#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_TIMER_H__ */