/**
 * @file drv_timer_it.c
 * @brief timer interrupt
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-25       vector(vector.qiu@gmail.com)    first version
 * 2023-05-29       vector                          rename driver to drv
 *
 */
#include "drv_timer_it.h"

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
#ifdef USE_DRV_TIM1
    drv_pwm_out_brake_irq(DRV_TIM1);
#endif
#ifdef USE_DRV_TIM9
    drv_tim_base_irq(DRV_TIM9);
#endif
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
#ifdef USE_DRV_TIM1
    drv_tim_base_irq(DRV_TIM1);
#endif
#ifdef USE_DRV_TIM10
    drv_tim_base_irq(DRV_TIM10);
#endif
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
#ifdef USE_DRV_TIM11
    drv_tim_base_irq(DRV_TIM11);
#endif
}

/**
  * @brief This function handles TIM1 Capture Compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{

}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
#ifdef USE_DRV_TIM2
    drv_tim_base_irq(DRV_TIM2);
#endif
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
#ifdef USE_DRV_TIM3
    drv_tim_base_irq(DRV_TIM3);
#endif
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
#ifdef USE_DRV_TIM4
    drv_tim_base_irq(DRV_TIM4);
#endif
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
#ifdef USE_DRV_TIM8
    drv_pwm_out_brake_irq(DRV_TIM8);
#endif
#ifdef USE_DRV_TIM12
    drv_tim_base_irq(DRV_TIM12);
#endif
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
#ifdef USE_DRV_TIM8
    drv_tim_base_irq(DRV_TIM8);
#endif
#ifdef USE_DRV_TIM13
    drv_tim_base_irq(DRV_TIM13);
#endif
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
#ifdef USE_DRV_TIM14
    drv_tim_base_irq(DRV_TIM14);
#endif
}

/**
  * @brief This function handles TIM8 Capture Compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{

}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
#ifdef USE_DRV_TIM5
    drv_tim_base_irq(DRV_TIM5);
#endif
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
#ifdef USE_DRV_TIM6
    drv_tim_base_irq(DRV_TIM6);
#endif
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
#ifdef USE_DRV_TIM7
    drv_tim_base_irq(DRV_TIM7);
#endif
}
