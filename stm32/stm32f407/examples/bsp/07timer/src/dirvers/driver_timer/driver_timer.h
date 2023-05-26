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


void rcc_tim_gpio_enable(GPIO_TypeDef* GPIOx);
uint32_t tim_clk_get(TIM_TypeDef* TIMx);
void rcc_tim_enable(TIM_TypeDef* TIMx);

typedef enum {
    DRIVER_TIM1 = 0,
    DRIVER_TIM2,    // 32位计数值
    DRIVER_TIM3,
    DRIVER_TIM4,
    DRIVER_TIM5,    // 32位计数值
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

typedef enum {
    UNIT_1US = 0,
    UNIT_10US,
    UNIT_100US,
    UNIT_1MS,
} tim_unit_e;

/* timer base */
typedef void (*time_task)(void);
typedef struct {
    tim_mode_e mode;
    uint32_t cycle; // TIM2/5为32位计数器
    tim_unit_e time_uint;
    time_task task;
} driver_tim_config_t;
void driver_tim_base_config(driver_tim_id_e _id, driver_tim_config_t _config);
void driver_tim_base_start(driver_tim_id_e _id);
void driver_tim_base_stop(driver_tim_id_e _id);
void driver_tim_base_irq(driver_tim_id_e _id);
/* timer encoder */


#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_TIMER_H__ */

