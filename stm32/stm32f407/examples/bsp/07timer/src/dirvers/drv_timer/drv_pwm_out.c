/**
 * @file drv_pwm_out.c
 * @brief pwm out driver
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-18       vector(vector.qiu@gmail.com)    first version
 * 2023-05-29       vector                          rename driver to drv
 * 2023-05-30       vector                          interface perfection,cutting and parameter inspection through macro definition
 *
 */
#include "drv_timer.h"

#if defined(USE_DRV_TIM1) || defined(USE_DRV_TIM2) || defined(USE_DRV_TIM3) ||      \
    defined(USE_DRV_TIM4) || defined(USE_DRV_TIM5) || defined(USE_DRV_TIM8) ||      \
    defined(USE_DRV_TIM9) || defined(USE_DRV_TIM10) || defined(USE_DRV_TIM11) ||    \
    defined(USE_DRV_TIM12) || defined(USE_DRV_TIM13) || defined(USE_DRV_TIM14)

#define IS_drv_pwm_out_ID(_id)  (((_id) == DRV_TIM1 )   ||    \
                             ((_id) == DRV_TIM2 )   ||    \
                             ((_id) == DRV_TIM3 )   ||    \
                             ((_id) == DRV_TIM4)    ||    \
                             ((_id) == DRV_TIM5 )   ||    \
                             ((_id) == DRV_TIM8 )   ||    \
                             ((_id) == DRV_TIM9 )   ||    \
                             ((_id) == DRV_TIM10 )  ||    \
                             ((_id) == DRV_TIM11 )  ||    \
                             ((_id) == DRV_TIM12 )  ||    \
                             ((_id) == DRV_TIM13 )  ||    \
                             ((_id) == DRV_TIM14 ))

#define IS_drv_pwm_out_CHANNELS(_channel)   (((_channel) == PWM_CHANNEL_1)  ||  \
                                         ((_channel) == PWM_CHANNEL_2)  ||  \
                                         ((_channel) == PWM_CHANNEL_3)  ||  \
                                         ((_channel) == PWM_CHANNEL_4))

#define IS_drv_pwm_out_DUTY_CYCLE(_duty)    ((_duty) >= 0 && (_duty) <= 10000)
#define IS_drv_pwm_out_FREQUENCY(_frequency)    ((_frequency) > 0 && (_frequency) <= 100000)
#define IS_drv_pwm_out_MODE(_mode)          (((_mode) == PWM_MODE1) ||  \
                                         ((_mode) == PWM_MODE2)).
typedef struct {
    TIM_HandleTypeDef htimx;
    TIM_TypeDef *timx;
    uint32_t clock_division;
    uint32_t period;
    GPIO_TypeDef* gpio_channel[4];
    uint16_t gpio_pin_channel[4];
    GPIO_TypeDef* gpio_channel_n[3];
    uint16_t gpio_pin_channel_n[3];
    GPIO_TypeDef* gpio_brake;
    uint16_t gpio_pin_brake;
    drv_pwm_out_config_t config;
} drv_pwm_out_put_dev_t;
static drv_pwm_out_put_dev_t dev[DRV_TIM_NONE] = {
#ifdef USE_DRV_TIM1
    {
        .timx = TIM1,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM1 GPIO Configuration
        PE9     ------> TIM1_CH1
        PE11     ------> TIM1_CH2
        PE13     ------> TIM1_CH3
        PE14     ------> TIM1_CH4
        PE8     ------> TIM1_CH1N
        PE10     ------> TIM1_CH2N
        PE12     ------> TIM1_CH3N
        PE15     ------> TIM1_BKIN
        */
        .gpio_channel[0] = GPIOE,
        .gpio_pin_channel[0] = GPIO_PIN_9,
        .gpio_channel[1] = GPIOE,
        .gpio_pin_channel[1] = GPIO_PIN_11,
        .gpio_channel[2] = GPIOE,
        .gpio_pin_channel[2] = GPIO_PIN_13,
        .gpio_channel[3] = GPIOE,
        .gpio_pin_channel[3] = GPIO_PIN_14,
        .gpio_channel_n[0] = GPIOE,
        .gpio_pin_channel_n[0] = GPIO_PIN_8,
        .gpio_channel_n[1] = GPIOE,
        .gpio_pin_channel_n[1] = GPIO_PIN_10,
        .gpio_channel_n[2] = GPIOE,
        .gpio_pin_channel_n[2] = GPIO_PIN_12,
        .gpio_brake = GPIOE,
        .gpio_pin_brake = GPIO_PIN_15,
    },
#endif
#ifdef USE_DRV_TIM2
    {
        .timx = TIM2,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM2 GPIO Configuration
        PA15     ------> TIM2_CH1
        PB3     ------> TIM2_CH2
        PB10     ------> TIM2_CH3
        PB11     ------> TIM2_CH4
        */
        .gpio_channel[0] = GPIOA,
        .gpio_pin_channel[0] = GPIO_PIN_15,
        .gpio_channel[1] = GPIOB,
        .gpio_pin_channel[1] = GPIO_PIN_3,
        .gpio_channel[2] = GPIOA,
        .gpio_pin_channel[2] = GPIO_PIN_10,
        .gpio_channel[3] = GPIOB,
        .gpio_pin_channel[3] = GPIO_PIN_11,
    },
#endif
#ifdef USE_DRV_TIM3
    {
        .timx = TIM3,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM3 GPIO Configuration
        PB4     ------> TIM3_CH1
        PA7     ------> TIM3_CH2
        PB0     ------> TIM3_CH3
        PB1     ------> TIM3_CH4
        */
        .gpio_channel[0] = GPIOB,
        .gpio_pin_channel[0] = GPIO_PIN_4,
        .gpio_channel[1] = GPIOA,
        .gpio_pin_channel[1] = GPIO_PIN_7,
        .gpio_channel[2] = GPIOB,
        .gpio_pin_channel[2] = GPIO_PIN_0,
        .gpio_channel[3] = GPIOB,
        .gpio_pin_channel[3] = GPIO_PIN_1,
    },
#endif
#ifdef USE_DRV_TIM4
    {
        .timx = TIM4,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM4 GPIO Configuration
        PD12     ------> TIM4_CH1
        PD13     ------> TIM4_CH2
        PD14     ------> TIM4_CH3
        PD15     ------> TIM4_CH4
        */
        .gpio_channel[0] = GPIOD,
        .gpio_pin_channel[0] = GPIO_PIN_12,
        .gpio_channel[1] = GPIOD,
        .gpio_pin_channel[1] = GPIO_PIN_13,
        .gpio_channel[2] = GPIOD,
        .gpio_pin_channel[2] = GPIO_PIN_14,
        .gpio_channel[3] = GPIOD,
        .gpio_pin_channel[3] = GPIO_PIN_15,
    },
#endif
#ifdef USE_DRV_TIM5
    {
        .timx = TIM5,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM5 GPIO Configuration
        PA0-WKUP     ------> TIM5_CH1
        PA1     ------> TIM5_CH2
        PA2     ------> TIM5_CH3
        PA3     ------> TIM5_CH4
        */
        .gpio_channel[0] = GPIOA,
        .gpio_pin_channel[0] = GPIO_PIN_0,
        .gpio_channel[1] = GPIOA,
        .gpio_pin_channel[1] = GPIO_PIN_1,
        .gpio_channel[2] = GPIOA,
        .gpio_pin_channel[2] = GPIO_PIN_2,
        .gpio_channel[3] = GPIOA,
        .gpio_pin_channel[3] = GPIO_PIN_3,
    },
#endif
#ifdef USE_DRV_TIM6
    {
        .timx = TIM6,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
#endif
#ifdef USE_DRV_TIM7
    {
        .timx = TIM7,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
#endif
#ifdef USE_DRV_TIM8
    {
        .timx = TIM8,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM8 GPIO Configuration
        PC6     ------> TIM8_CH1
        PC7     ------> TIM8_CH2
        PC8     ------> TIM8_CH3
        PC9     ------> TIM8_CH4
        PA5     ------> TIM8_CH1N
        PB14     ------> TIM8_CH2N
        PB15     ------> TIM8_CH3N
        PA6     ------> TIM8_BKIN
        */
        .gpio_channel[0] = GPIOC,
        .gpio_pin_channel[0] = GPIO_PIN_6,
        .gpio_channel[1] = GPIOC,
        .gpio_pin_channel[1] = GPIO_PIN_7,
        .gpio_channel[2] = GPIOC,
        .gpio_pin_channel[2] = GPIO_PIN_8,
        .gpio_channel[3] = GPIOC,
        .gpio_pin_channel[3] = GPIO_PIN_9,
        .gpio_channel_n[0] = GPIOA,
        .gpio_pin_channel_n[0] = GPIO_PIN_5,
        .gpio_channel_n[1] = GPIOB,
        .gpio_pin_channel_n[1] = GPIO_PIN_14,
        .gpio_channel_n[2] = GPIOB,
        .gpio_pin_channel_n[2] = GPIO_PIN_15,
        .gpio_brake = GPIOA,
        .gpio_pin_brake = GPIO_PIN_6,
    },
#endif
#ifdef USE_DRV_TIM9
    {
        .timx = TIM9,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM9 GPIO Configuration
        PE5     ------> TIM9_CH1
        PE6     ------> TIM9_CH2
        */
        .gpio_channel[0] = GPIOE,
        .gpio_pin_channel[0] = GPIO_PIN_5,
        .gpio_channel[1] = GPIOE,
        .gpio_pin_channel[1] = GPIO_PIN_6,
    },
#endif
#ifdef USE_DRV_TIM10
    {
        .timx = TIM10,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM10 GPIO Configuration
        PF6     ------> TIM10_CH1
        */
        .gpio_channel[0] = GPIOF,
        .gpio_pin_channel[0] = GPIO_PIN_6,
    },
#endif
#ifdef USE_DRV_TIM11
    {
        .timx = TIM11,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM11 GPIO Configuration
        PF7     ------> TIM11_CH1
        */
        .gpio_channel[0] = GPIOF,
        .gpio_pin_channel[0] = GPIO_PIN_7,
    },
#endif
#ifdef USE_DRV_TIM12
    {
        .timx = TIM12,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM12 GPIO Configuration
        PB14     ------> TIM12_CH1  (TIM8_CH2N)
        PB15     ------> TIM12_CH2  (TIM8_CH3N)
        */
    },
#endif
#ifdef USE_DRV_TIM13
    {
        .timx = TIM13,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM13 GPIO Configuration
        PF8     ------> TIM13_CH1
        */
        .gpio_channel[0] = GPIOF,
        .gpio_pin_channel[0] = GPIO_PIN_8,
    },
#endif
#ifdef USE_DRV_TIM14
    {
        .timx = TIM14,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
        /**TIM14 GPIO Configuration
        PF9     ------> TIM14_CH1
        */
        .gpio_channel[0] = GPIOF,
        .gpio_pin_channel[0] = GPIO_PIN_9,
    },
#endif
};

static void drv_pwm_out_gpio_config(drv_tim_id_e _id);
static uint32_t drv_pwm_out_get_channel(pwm_channel_e _channel);
static bool drv_pwm_out_get_channel_state(drv_tim_id_e _id, pwm_channel_e _channel);

static void drv_pwm_out_gpio_config(drv_tim_id_e _id) {
    assert_param(IS_drv_pwm_out_ID(_id));

    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_1) == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel[0], dev[_id].gpio_pin_channel[0]);
    }
    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_2) == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel[1], dev[_id].gpio_pin_channel[1]);
    }
    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_3) == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel[2], dev[_id].gpio_pin_channel[2]);
    }
    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_4) == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel[3], dev[_id].gpio_pin_channel[3]);
    }
    if (dev[_id].config.channel_n_enable[0] == true && dev[_id].gpio_channel_n[0] != NULL) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel_n[0], dev[_id].gpio_pin_channel_n[0]);
    }
    if (dev[_id].config.channel_n_enable[1] == true && dev[_id].gpio_channel_n[1] != NULL) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel_n[1], dev[_id].gpio_pin_channel_n[1]);
    }
    if (dev[_id].config.channel_n_enable[2] == true && dev[_id].gpio_channel_n[2] != NULL) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel_n[2], dev[_id].gpio_pin_channel_n[2]);
    }
    if (dev[_id].config.brake_enable == true && dev[_id].gpio_brake != NULL) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_brake, dev[_id].gpio_pin_brake);
    }
}

static uint32_t drv_pwm_out_get_channel(pwm_channel_e _channel) {
    assert_param(IS_drv_pwm_out_CHANNELS(_channel));

    uint32_t channel;
    switch (_channel)
    {
    case PWM_CHANNEL_1:
        channel = TIM_CHANNEL_1;
        break;

    case PWM_CHANNEL_2:
        channel = TIM_CHANNEL_2;
        break;

    case PWM_CHANNEL_3:
        channel = TIM_CHANNEL_3;
        break;

    case PWM_CHANNEL_4:
        channel = TIM_CHANNEL_4;
        break;

    default:
        break;
    }
    return channel;
}

static bool drv_pwm_out_get_channel_state(drv_tim_id_e _id, pwm_channel_e _channel) {
    assert_param(IS_drv_pwm_out_ID(_id));
    assert_param(IS_drv_pwm_out_CHANNELS(_channel));

    bool state = false;
    switch (_channel)
    {
    case PWM_CHANNEL_1:
        if (dev[_id].config.channel_enable[0] == true && dev[_id].gpio_channel[0] != NULL) {
            state = true;
        }
        break;

    case PWM_CHANNEL_2:
        if (dev[_id].config.channel_enable[1] == true && dev[_id].gpio_channel[1] != NULL) {
            state = true;
        }
        break;

    case PWM_CHANNEL_3:
        if (dev[_id].config.channel_enable[2] == true && dev[_id].gpio_channel[2] != NULL) {
            state = true;
        }
        break;

    case PWM_CHANNEL_4:
        if (dev[_id].config.channel_enable[3] == true && dev[_id].gpio_channel[3] != NULL) {
            state = true;
        }
        break;

    default:
        break;
    }
    return state;
}

void drv_pwm_out_config(drv_tim_id_e _id, drv_pwm_out_config_t _config) {
    assert_param(IS_drv_pwm_out_ID(_id));
    assert_param(IS_drv_pwm_out_MODE(_config.mode));
    assert_param(IS_drv_pwm_out_FREQUENCY(_config.frequency));
    assert_param(IS_drv_pwm_out_DUTY_CYCLE(_config.default_duty_cycle));

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    uint32_t tim_clk = tim_clk_get(dev[_id].timx) / (dev[_id].clock_division + 1);
    uint32_t prescaler; // 预分频
    uint32_t period;
    uint32_t pulse;

    if (_config.frequency >= 10000)
    {
        prescaler = 0;
    }
    else if (_config.frequency >= 1000)
    {
        prescaler = 9;
    }
    else if (_config.frequency >= 100)
    {
        prescaler = 99;
    }
    else if (_config.frequency >= 10)
    {
        prescaler = 999;
    }
    else
    {
        prescaler = 9999;
    }
    period = tim_clk / (_config.frequency * (prescaler + 1));
    pulse = period * (_config.default_duty_cycle / 10000);

    dev[_id].period = period;
    dev[_id].config = _config;
    dev[_id].htimx.Instance = dev[_id].timx;
    dev[_id].htimx.Init.Prescaler = prescaler;
    dev[_id].htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
    dev[_id].htimx.Init.Period = period;
    dev[_id].htimx.Init.ClockDivision = dev[_id].clock_division;
    dev[_id].htimx.Init.RepetitionCounter = 0;
    dev[_id].htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&dev[_id].htimx) != HAL_OK)
    {
         Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&dev[_id].htimx, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
    if (HAL_TIM_PWM_Init(&dev[_id].htimx) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&dev[_id].htimx, &sMasterConfig) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }

    if (_config.mode == PWM_MODE1)
    {
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
    }
    else
    {
        sConfigOC.OCMode = TIM_OCMODE_PWM2;
    }
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_1) == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }

    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_2) == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }

    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_3) == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }
    if (drv_pwm_out_get_channel_state(_id, PWM_CHANNEL_4) == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = _config.brake_time;
    if (_config.brake_enable == true)
    {
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    }
    else
    {
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    }
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&dev[_id].htimx, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }

    drv_pwm_out_gpio_config(_id);
}

error_t drv_pwm_out_duty_set(drv_tim_id_e _id, pwm_channel_e _channel, uint16_t _duty) {
    assert_param(IS_drv_pwm_out_ID(_id));
    assert_param(IS_drv_pwm_out_CHANNELS(_channel));
    assert_param(IS_drv_pwm_out_DUTY_CYCLE(_duty));

    uint32_t pulse;

    if (&dev[_id].htimx == NULL || drv_pwm_out_get_channel_state(_id, _channel) == false)
    {
        return -EEMPTY;
    }
    pulse = dev[_id].period * (_duty / 10000);
    __HAL_TIM_SET_COMPARE(&dev[_id].htimx, drv_pwm_out_get_channel(_channel), pulse);
    return EOK;
}

error_t drv_pwm_out_start(drv_tim_id_e _id, pwm_channel_e _channel) {
    assert_param(IS_drv_pwm_out_ID(_id));
    assert_param(IS_drv_pwm_out_CHANNELS(_channel));

    if (&dev[_id].htimx == NULL || drv_pwm_out_get_channel_state(_id, _channel) == false)
    {
        return -EEMPTY;
    }
    HAL_TIM_PWM_Start(&dev[_id].htimx, drv_pwm_out_get_channel(_channel));
    return EOK;
}

error_t drv_pwm_out_stop(drv_tim_id_e _id, pwm_channel_e _channel) {
    assert_param(IS_drv_pwm_out_ID(_id));
    assert_param(IS_drv_pwm_out_CHANNELS(_channel));

    if (&dev[_id].htimx == NULL || drv_pwm_out_get_channel_state(_id, _channel) == false)
    {
        return -EEMPTY;
    }
    HAL_TIM_PWM_Stop(&dev[_id].htimx, drv_pwm_out_get_channel(_channel));
    return EOK;
}

void drv_pwm_out_brake_irq(drv_tim_id_e _id) {
    if (__HAL_TIM_GET_FLAG(&dev[_id].htimx, TIM_FLAG_BREAK) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&dev[_id].htimx, TIM_IT_BREAK) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&dev[_id].htimx, TIM_IT_BREAK);
            dev[_id].config.call_back(dev[_id].config.parameter);
        }
    }
}

#endif
