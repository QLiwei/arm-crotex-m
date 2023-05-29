/**
 * @file drv_pwm_out.c
 * @brief pwm out driver
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-18       vector(vector.qiu@gmail.com)    first version
 * 2023-05-29       vector                          rename driver to drv
 *
 */
#include "drv_timer.h"


typedef struct {
    TIM_HandleTypeDef htimx;
    TIM_TypeDef *timx;
    uint32_t clock_division;
    GPIO_TypeDef* gpio_channel1;
    uint16_t gpio_pin_channel1;
    GPIO_TypeDef* gpio_channel2;
    uint16_t gpio_pin_channel2;
    GPIO_TypeDef* gpio_channel3;
    uint16_t gpio_pin_channel3;
    GPIO_TypeDef* gpio_channel4;
    uint16_t gpio_pin_channel4;
    GPIO_TypeDef* gpio_channel1_n;
    uint16_t gpio_pin_channel1_n;
    GPIO_TypeDef* gpio_channel2_n;
    uint16_t gpio_pin_channel2_n;
    GPIO_TypeDef* gpio_channel3_n;
    uint16_t gpio_pin_channel3_n;
    GPIO_TypeDef* gpio_brake;
    uint16_t gpio_pin_brake;
    drv_pwm_out_config_t config;
} drv_pwm_put_dev_t;
static drv_pwm_put_dev_t dev[DRV_TIM_NONE] = {
    {
        .timx = TIM1,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
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
        .gpio_channel1 = GPIOE,
        .gpio_pin_channel1 = GPIO_PIN_9,
        .gpio_channel2 = GPIOE,
        .gpio_pin_channel2 = GPIO_PIN_11,
        .gpio_channel3 = GPIOE,
        .gpio_pin_channel3 = GPIO_PIN_13,
        .gpio_channel4 = GPIOE,
        .gpio_pin_channel4 = GPIO_PIN_14,
        .gpio_channel1_n = GPIOE,
        .gpio_pin_channel1_n = GPIO_PIN_8,
        .gpio_channel2_n = GPIOE,
        .gpio_pin_channel2_n = GPIO_PIN_10,
        .gpio_channel3_n = GPIOE,
        .gpio_pin_channel3_n = GPIO_PIN_12,
        .gpio_brake = GPIOE,
        .gpio_pin_brake = GPIO_PIN_15,
    },
    {
        .timx = TIM2,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM2 GPIO Configuration
        PA15     ------> TIM2_CH1
        PB3     ------> TIM2_CH2
        PB10     ------> TIM2_CH3
        PB11     ------> TIM2_CH4
        */
        .gpio_channel1 = GPIOA,
        .gpio_pin_channel1 = GPIO_PIN_15,
        .gpio_channel2 = GPIOB,
        .gpio_pin_channel2 = GPIO_PIN_3,
        .gpio_channel3 = GPIOA,
        .gpio_pin_channel3 = GPIO_PIN_10,
        .gpio_channel4 = GPIOB,
        .gpio_pin_channel4 = GPIO_PIN_11,
    },
    {
        .timx = TIM3,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM3 GPIO Configuration
        PB4     ------> TIM3_CH1
        PA7     ------> TIM3_CH2
        PB0     ------> TIM3_CH3
        PB1     ------> TIM3_CH4
        */
        .gpio_channel1 = GPIOB,
        .gpio_pin_channel1 = GPIO_PIN_4,
        .gpio_channel2 = GPIOA,
        .gpio_pin_channel2 = GPIO_PIN_7,
        .gpio_channel3 = GPIOB,
        .gpio_pin_channel3 = GPIO_PIN_0,
        .gpio_channel4 = GPIOB,
        .gpio_pin_channel4 = GPIO_PIN_1,
    },
    {
        .timx = TIM4,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM4 GPIO Configuration
        PD12     ------> TIM4_CH1
        PD13     ------> TIM4_CH2
        PD14     ------> TIM4_CH3
        PD15     ------> TIM4_CH4
        */
        .gpio_channel1 = GPIOD,
        .gpio_pin_channel1 = GPIO_PIN_12,
        .gpio_channel2 = GPIOD,
        .gpio_pin_channel2 = GPIO_PIN_13,
        .gpio_channel3 = GPIOD,
        .gpio_pin_channel3 = GPIO_PIN_14,
        .gpio_channel4 = GPIOD,
        .gpio_pin_channel4 = GPIO_PIN_15,
    },
    {
        .timx = TIM5,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM5 GPIO Configuration
        PA0-WKUP     ------> TIM5_CH1
        PA1     ------> TIM5_CH2
        PA2     ------> TIM5_CH3
        PA3     ------> TIM5_CH4
        */
        .gpio_channel1 = GPIOA,
        .gpio_pin_channel1 = GPIO_PIN_0,
        .gpio_channel2 = GPIOA,
        .gpio_pin_channel2 = GPIO_PIN_1,
        .gpio_channel3 = GPIOA,
        .gpio_pin_channel3 = GPIO_PIN_2,
        .gpio_channel4 = GPIOA,
        .gpio_pin_channel4 = GPIO_PIN_3,
    },
    {
        .timx = TIM6,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
    {
        .timx = TIM7,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
    {
        .timx = TIM8,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
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
        .gpio_channel1 = GPIOC,
        .gpio_pin_channel1 = GPIO_PIN_6,
        .gpio_channel2 = GPIOC,
        .gpio_pin_channel2 = GPIO_PIN_7,
        .gpio_channel3 = GPIOC,
        .gpio_pin_channel3 = GPIO_PIN_8,
        .gpio_channel4 = GPIOC,
        .gpio_pin_channel4 = GPIO_PIN_9,
        .gpio_channel1_n = GPIOA,
        .gpio_pin_channel1_n = GPIO_PIN_5,
        .gpio_channel2_n = GPIOB,
        .gpio_pin_channel2_n = GPIO_PIN_14,
        .gpio_channel3_n = GPIOB,
        .gpio_pin_channel3_n = GPIO_PIN_15,
        .gpio_brake = GPIOA,
        .gpio_pin_brake = GPIO_PIN_6,
    },
    {
        .timx = TIM9,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM9 GPIO Configuration
        PE5     ------> TIM9_CH1
        PE6     ------> TIM9_CH2
        */
        .gpio_channel1 = GPIOE,
        .gpio_pin_channel1 = GPIO_PIN_5,
        .gpio_channel2 = GPIOE,
        .gpio_pin_channel2 = GPIO_PIN_6,
    },
    {
        .timx = TIM10,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM10 GPIO Configuration
        PF6     ------> TIM10_CH1
        */
        .gpio_channel1 = GPIOF,
        .gpio_pin_channel1 = GPIO_PIN_6,
    },
    {
        .timx = TIM11,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM11 GPIO Configuration
        PF7     ------> TIM11_CH1
        */
        .gpio_channel1 = GPIOF,
        .gpio_pin_channel1 = GPIO_PIN_7,
    },
    {
        .timx = TIM12,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM12 GPIO Configuration
        PB14     ------> TIM12_CH1  (TIM8_CH2N)
        PB15     ------> TIM12_CH2  (TIM8_CH3N)
        */
    },
    {
        .timx = TIM13,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM13 GPIO Configuration
        PF8     ------> TIM13_CH1
        */
        .gpio_channel1 = GPIOF,
        .gpio_pin_channel1 = GPIO_PIN_8,
    },
    {
        .timx = TIM14,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
        /**TIM14 GPIO Configuration
        PF9     ------> TIM14_CH1
        */
        .gpio_channel1 = GPIOF,
        .gpio_pin_channel1 = GPIO_PIN_9,
    },
};

static void drv_pwm_gpio_config(drv_tim_id_e _id);
static uint32_t drv_pwm_out_get_channel(pwm_channel_e _channel);

static void drv_pwm_gpio_config(drv_tim_id_e _id) {
    /* parameter */

    if (dev[_id].config.channel1_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel1, dev[_id].gpio_pin_channel1);
    }
    if (dev[_id].config.channel2_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel2, dev[_id].gpio_pin_channel2);
    }
    if (dev[_id].config.channel3_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel3, dev[_id].gpio_pin_channel3);
    }
    if (dev[_id].config.channel4_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel4, dev[_id].gpio_pin_channel4);
    }
    if (dev[_id].config.channel1_n_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel1_n, dev[_id].gpio_pin_channel1_n);
    }
    if (dev[_id].config.channel2_n_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel2_n, dev[_id].gpio_pin_channel2_n);
    }
    if (dev[_id].config.channel3_n_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_channel3_n, dev[_id].gpio_pin_channel3_n);
    }
    if (dev[_id].config.brake_enable == true) {
        tim_gpio_config(dev[_id].timx, dev[_id].gpio_brake, dev[_id].gpio_pin_brake);
    }
}

static uint32_t drv_pwm_out_get_channel(pwm_channel_e _channel) {
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

void drv_pwm_out_config(drv_tim_id_e _id, drv_pwm_out_config_t _config) {
    /* parameter */
    assert_param(_id < drv_TIM_NONE);
    assert_param(_id != drv_TIM6 && _id != drv_TIM7);
    assert_param(_config.default_duty_cycle >= 0 && _config.default_duty_cycle <= 10000);
    assert_param(_config.frequency > 0);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    uint32_t tim_clk = tim_clk_get(dev[_id].timx) / (dev[_id].clock_division + 1);
    uint32_t prescaler; // 预分频
    uint32_t period;
    uint32_t pulse;


    dev[_id].config = _config;

    dev[_id].htimx.Instance = dev[_id].timx;
    dev[_id].htimx.Init.Prescaler = prescaler;
    dev[_id].htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
    dev[_id].htimx.Init.Period = period;
    dev[_id].htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
    else if (_config.mode == PWM_MODE2)
    {
        sConfigOC.OCMode = TIM_OCMODE_PWM2;
    } else {
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
    }
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (_config.channel1_enable == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }

    if (_config.channel2_enable == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }

    if (_config.channel3_enable == true)
    {
        if (HAL_TIM_PWM_ConfigChannel(&dev[_id].htimx, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }
    if (_config.channel4_enable == true)
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

    drv_pwm_gpio_config(_id);
}

void drv_pwm_out_start(drv_tim_id_e _id, pwm_channel_e _channel) {

    HAL_TIM_PWM_Start(&dev[_id].htimx, drv_pwm_out_get_channel(_channel));
}

void drv_pwm_out_stop(drv_tim_id_e _id, pwm_channel_e _channel) {
    HAL_TIM_PWM_Stop(&dev[_id].htimx, drv_pwm_out_get_channel(_channel));
}


void drv_brake_irq(drv_tim_id_e _id) {

}
