/**
 * @file drv_timer.h
 * @brief timer driver definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-18       vector(vector.qiu@gmail.com)    first version
 *
 */
#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

void rcc_tim_gpio_enable(GPIO_TypeDef* GPIOx);
uint32_t tim_clk_get(TIM_TypeDef* TIMx);
void rcc_tim_enable(TIM_TypeDef* TIMx);
void rcc_tim_disable(TIM_TypeDef* TIMx);
uint8_t tim_af_get(TIM_TypeDef* TIMx);
void tim_gpio_config(TIM_TypeDef* TIMx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PinX);

typedef enum {
    DRV_TIM1 = 0,    // [Advanced]       Up,Dowm,Up/Down Capture compare channels:4 Complemen-tary output: yes   APB2:MAX-168MHz
    DRV_TIM2,        // [General] 32bits Up,Dowm,Up/Down Capture compare channels:4 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM3,        // [General]        Up,Dowm,Up/Down Capture compare channels:4 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM4,        // [General]        Up,Dowm,Up/Down Capture compare channels:4 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM5,        // [General] 32bits Up,Dowm,Up/Down Capture compare channels:4 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM6,        // [Basic]          Up              Capture compare channels:0                              APB1:MAX-84MHz
    DRV_TIM7,        // [Basic]          Up              Capture compare channels:0                              APB1:MAX-84MHz
    DRV_TIM8,        // [Advanced]       Up,Dowm,Up/Down Capture compare channels:4 Complemen-tary output: yes   APB2:MAX-168MHz
    DRV_TIM9,        // [General]        Up              Capture compare channels:2 Complemen-tary output: no    APB2:MAX-168MHz
    DRV_TIM10,       // [General]        Up              Capture compare channels:1 Complemen-tary output: no    APB2:MAX-168MHz
    DRV_TIM11,       // [General]        Up              Capture compare channels:1 Complemen-tary output: no    APB2:MAX-168MHz
    DRV_TIM12,       // [General]        Up              Capture compare channels:2 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM13,       // [General]        Up              Capture compare channels:1 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM14,       // [General]        Up              Capture compare channels:1 Complemen-tary output: no    APB1:MAX-84MHz
    DRV_TIM_NONE,
}drv_tim_id_e;

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
typedef void (*time_task)(void *parameter);
typedef struct {
    tim_mode_e mode;
    uint32_t cycle; // TIM2/5为32位计数器
    tim_unit_e time_uint;
    time_task task;
    void *parameter;
} drv_tim_config_t;
void drv_tim_base_config(drv_tim_id_e _id, drv_tim_config_t _config);
void drv_tim_base_start(drv_tim_id_e _id);
void drv_tim_base_stop(drv_tim_id_e _id);
void drv_tim_base_irq(drv_tim_id_e _id);

/* timer pwm out  */
typedef enum {
    PWM_MODE1 = 0,
    PWM_MODE2,
} pwm_mode_e;

typedef enum {
    PWM_CHANNEL_1 = 0,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
} pwm_channel_e;

typedef void (*brake_call_back)(void *parameter);
typedef struct {
    pwm_mode_e mode;
    uint32_t frequency;
    uint16_t default_duty_cycle; // 100%-10000 1%-100
    bool channel1_enable;
    bool channel2_enable;
    bool channel3_enable;
    bool channel4_enable;
    bool channel1_n_enable;
    bool channel2_n_enable;
    bool channel3_n_enable;
    bool brake_enable;
    uint8_t brake_time;
    brake_call_back call_back;
    void *parameter;
}drv_pwm_out_config_t;
void drv_pwm_out_config(drv_tim_id_e _id, drv_pwm_out_config_t _config);
void drv_pwm_out_duty_cycle(drv_tim_id_e _id, pwm_channel_e _channel, uint16_t _duty);
void drv_pwm_out_start(drv_tim_id_e _id, pwm_channel_e _channel);
void drv_pwm_out_stop(drv_tim_id_e _id, pwm_channel_e _channel);
void drv_pwm_out_brake_irq(drv_tim_id_e _id);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_TIMER_H__ */

