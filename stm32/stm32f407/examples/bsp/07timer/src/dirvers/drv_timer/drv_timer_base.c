/**
 * @file drv_timer_base.c
 * @brief timer base driver
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-18       vector(vector.qiu@gmail.com)    first version
 * 2023-05-29       vector                          rename driver to drv
 * 2023-05-30       vector                          cutting and parameter inspection through macro definition
 *
 */
#include "drv_timer.h"

#if defined(USE_DRV_TIM1) || defined(USE_DRV_TIM2) || defined(USE_DRV_TIM3) ||      \
    defined(USE_DRV_TIM4) || defined(USE_DRV_TIM5) || defined(USE_DRV_TIM6) ||      \
    defined(USE_DRV_TIM7) || defined(USE_DRV_TIM8) || defined(USE_DRV_TIM9) ||      \
    defined(USE_DRV_TIM10) || defined(USE_DRV_TIM11) || defined(USE_DRV_TIM12) ||   \
    defined(USE_DRV_TIM13) || defined(USE_DRV_TIM14)

#define IS_DRV_TIM_BASE_ID(_id)     (((_id) == DRV_TIM1 )   ||    \
                                    ((_id) == DRV_TIM2 )   ||    \
                                    ((_id) == DRV_TIM3 )   ||    \
                                    ((_id) == DRV_TIM4)    ||    \
                                    ((_id) == DRV_TIM5 )   ||    \
                                    ((_id) == DRV_TIM6 )   ||    \
                                    ((_id) == DRV_TIM7 )   ||    \
                                    ((_id) == DRV_TIM8 )   ||    \
                                    ((_id) == DRV_TIM9 )   ||    \
                                    ((_id) == DRV_TIM10 )  ||    \
                                    ((_id) == DRV_TIM11 )  ||    \
                                    ((_id) == DRV_TIM12 )  ||    \
                                    ((_id) == DRV_TIM13 )  ||    \
                                    ((_id) == DRV_TIM14 ))

#define IS_DRV_TIM_BASE_MODE(_mode)         (((_mode) == MODE_ONESHOT) ||  \
                                             ((_mode) == MODE_PERIOD))

#define IS_DRV_TIM_BASE_TIME_UINT(_uint)    (((_uint) == UNIT_1US)      ||  \
                                             ((_uint) == UNIT_10US)     ||  \
                                             ((_uint) == UNIT_100US)    ||  \
                                             ((_uint) == UNIT_1MS))

#define IS_DRV_TIM_BASE_CYCLE(_cycle)   ((_cycle) > 0)
#define IS_DRV_TIM_BASE_TASK(_tack)     ((_task != NULL)

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
    TIM_HandleTypeDef htimx;
    TIM_TypeDef *timx;
    IRQn_Type irqn_type;
    uint32_t preempt_priority;
    uint32_t sub_priority;
    uint32_t clock_division;
    drv_tim_config_t config;
}drv_timer_base_dev_t;
static drv_timer_base_dev_t dev[DRV_TIM_NONE] = {
#ifdef USE_DRV_TIM1
    {
        .timx = TIM1,
        .irqn_type = TIM1_UP_TIM10_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
#endif
#ifdef USE_DRV_TIM2
    {
        .timx = TIM2,
        .irqn_type = TIM2_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM3
    {
        .timx = TIM3,
        .irqn_type = TIM3_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM4
    {
        .timx = TIM4,
        .irqn_type = TIM4_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM5
    {
        .timx = TIM5,
        .irqn_type = TIM5_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM6
    {
        .timx = TIM6,
        .irqn_type = TIM6_DAC_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM7
    {
        .timx = TIM7,
        .irqn_type = TIM7_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM8
    {
        .timx = TIM8,
        .irqn_type = TIM8_UP_TIM13_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
#endif
#ifdef USE_DRV_TIM9
    {
        .timx = TIM9,
        .irqn_type = TIM1_BRK_TIM9_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
#endif
#ifdef USE_DRV_TIM10
    {
        .timx = TIM10,
        .irqn_type = TIM1_UP_TIM10_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
#endif
#ifdef USE_DRV_TIM11
    {
        .timx = TIM11,
        .irqn_type = TIM1_TRG_COM_TIM11_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV4,
    },
#endif
#ifdef USE_DRV_TIM12
    {
        .timx = TIM12,
        .irqn_type = TIM8_BRK_TIM12_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM13
    {
        .timx = TIM13,
        .irqn_type = TIM8_UP_TIM13_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
#ifdef USE_DRV_TIM14
    {
        .timx = TIM14,
        .irqn_type = TIM8_TRG_COM_TIM14_IRQn,
        .preempt_priority = 1,
        .sub_priority = 0,
        .clock_division = TIM_CLOCKDIVISION_DIV2,
    },
#endif
};

void drv_tim_base_config(drv_tim_id_e _id, drv_tim_config_t _config) {
    assert_param(IS_DRV_TIM_BASE_ID(_id));
    assert_param(IS_DRV_TIM_BASE_MODE(_config.mode));
    assert_param(IS_DRV_TIM_BASE_TIME_UINT(_config.time_uint));
    assert_param(IS_DRV_TIM_BASE_CYCLE(_config.cycle));
    assert_param(IS_DRV_TIM_BASE_TASK(_config.task));

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    uint32_t tim_clk = tim_clk_get(dev[_id].timx) / (dev[_id].clock_division + 1); // 42MHZ
    uint32_t prescaler; // 预分频
    uint32_t period;

    if (_config.time_uint == UNIT_1US) {
        prescaler = tim_clk / 1000000;
    } else if (_config.time_uint == UNIT_10US) {
        prescaler = tim_clk / 100000;
    } else if (_config.time_uint == UNIT_100US) {
        prescaler = tim_clk / 10000;
    } else if (_config.time_uint == UNIT_1MS) {
        prescaler = tim_clk / 1000;
    }

    if (dev[_id].timx != TIM2 && dev[_id].timx != TIM5) {
        if (_config.cycle > 0xFFFF) {
            _config.cycle = 0xFFFF;
        }
    }
    period = _config.cycle - 1;
    dev[_id].config = _config;
    rcc_tim_enable(dev[_id].timx);
    HAL_NVIC_SetPriority(dev[_id].irqn_type, dev[_id].preempt_priority, dev[_id].sub_priority);
    HAL_NVIC_EnableIRQ(dev[_id].irqn_type);

    dev[_id].htimx.Instance = dev[_id].timx;
    dev[_id].htimx.Init.Prescaler = prescaler - 1;
    dev[_id].htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
    dev[_id].htimx.Init.Period = period;
    dev[_id].htimx.Init.ClockDivision = dev[_id].clock_division;
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

    if (((dev[_id].timx) == TIM1) || ((dev[_id].timx) == TIM5) || ((dev[_id].timx) == TIM6)) {
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&dev[_id].htimx, &sMasterConfig) != HAL_OK)
        {
            Error_Handler(__FILE__, __LINE__);
        }
    }
}

void drv_tim_base_start(drv_tim_id_e _id) {
    assert_param(IS_DRV_TIM_BASE_ID(_id));
    HAL_TIM_Base_Start(&dev[_id].htimx);
}

void drv_tim_base_stop(drv_tim_id_e _id) {
    assert_param(IS_DRV_TIM_BASE_ID(_id));
    HAL_TIM_Base_Stop(&dev[_id].htimx);
}

void drv_tim_base_irq(drv_tim_id_e _id) {
    if (__HAL_TIM_GET_FLAG(&dev[_id].htimx, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&dev[_id].htimx, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&dev[_id].htimx, TIM_IT_UPDATE);
            dev[_id].config.task(dev[_id].config.parameter);
            if (dev[_id].config.mode == MODE_ONESHOT) {
                HAL_TIM_Base_Stop(&dev[_id].htimx);
            }
        }
    }
}

#endif
