/**
 * @file driver_timer_base.c
 * @brief timer base driver
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-18       vector(vector.qiu@gmail.com)    first version
 */
#include "driver_timer.h"


/*
 *  System Clock source            = PLL (HSE:8000000)
 *  SYSCLK(Hz)                     = 168000000 (CPU Clock)
 *  HCLK = SYSCLK / 1              = 168000000 (AHB1Periph)
 *  PCLK2 = HCLK / 2               = 84000000  (APB2Periph)
 *  PCLK1 = HCLK / 4               = 42000000  (APB1Periph)
 *  定时器时钟频率由硬件自动设置
 *      APB 预分频器为 1，定时器时钟频率等于 APB 域的频率。
 *      否则，等于 APB 域的频率的两倍 (×2)。
 *  APB2 Timer = PCLK2 * 2         = 168000000  (TIM1/8/9/10/11)
 *  PPB1 Timer = PCLK1 * 2         = 84000000   (TIM2/3/4/5/12/13/14)
 *
 *  TIMxCLK = Timer / ClockDivision(1/2/4) 168MHZ(5.952ns) 84MHZ(11.904ns) 42MHZ(23.809ns)
 *
 *  T = (Period * Prescaler) / TIMxCLK
 */
typedef struct {
    TIM_TypeDef *timx;
    IRQn_Type irqn_type;
    uint32_t preempt_priority;
    uint32_t sub_priority;
    uint32_t clock_division;
    driver_tim_config_t config;
}driver_timer_base_dev_t;
static driver_timer_base_dev_t dev[DRIVER_TIM_NONE] {
    {
        .timx = TIM1,
        .irqn_type = TIM1_UP_TIM10_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM2,
        .irqn_type = TIM2_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM3,
        .irqn_type = TIM3_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM4,
        .irqn_type = TIM4_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM5,
        .irqn_type = TIM5_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM6,
        .irqn_type = TIM6_DAC_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM7,
        .irqn_type = TIM7_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM8,
        .irqn_type = TIM8_UP_TIM13_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM9,
        .irqn_type = TIM1_BRK_TIM9_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM10,
        .irqn_type = TIM1_UP_TIM10_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM11,
        .irqn_type = TIM1_TRG_COM_TIM11_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM12,
        .irqn_type = TIM8_BRK_TIM12_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM13,
        .irqn_type = TIM8_UP_TIM13_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
    {
        .timx = TIM14,
        .irqn_type = TIM8_TRG_COM_TIM14_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV1,
    },
}

void driver_tim_config(driver_tim_id_e _id, driver_tim_config_t _config) {
    assert_param(_id < DRIVER_TIM_NONE);
    assert_param(_config.cycle > 0);
    assert_param(_config.task != NULL);

    TIM_HandleTypeDef htimx = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    uint32_t tim_clk = tim_clk_get(dev[_id].timx) / (dev[_id].clock_division + 1);
    uint32_t prescaler; // 预分频
    uint32_t period;

    prescaler = tim_clk / 1000000;
    if (dev[_id].timx != DRIVER_TIM2 && dev[_id].timx != DRIVER_TIM5) {
        if (_config.cycle > 0xFFFF) {
            _config.cycle = 0xFFFF;
        }
    }
    period = _config.cycle - 1;
    dev[_id].config = _config;
    rcc_tim_enable(dev[_id].timx);
    HAL_NVIC_SetPriority(dev[_id].irqn_type, dev[_id].preempt_priority, dev[_id].sub_priority);
    HAL_NVIC_EnableIRQ(dev[_id].irqn_type);

    htimx.Instance = dev[_id].timx;
    htimx.Init.Prescaler = prescaler;
    htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimx.Init.Period = period;
    htimx.Init.ClockDivision = dev[_id].clock_division;
    htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htimx) != HAL_OK)
    {
        Error_Handler();
    }

    if (((dev[_id].timx) == TIM1) || ((dev[_id].timx) == TIM5) || ((dev[_id].timx) == TIM6)) {
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }

}

