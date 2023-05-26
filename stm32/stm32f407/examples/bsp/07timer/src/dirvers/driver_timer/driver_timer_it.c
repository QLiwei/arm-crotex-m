/**
 * @file driver_timer_it.c
 * @brief timer interrupt
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-05-25       vector(vector.qiu@gmail.com)    first version
 */
#include "driver_timer_it.h"

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM9);
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM1);
    driver_tim_base_irq(DRIVER_TIM10);
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM11);
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
    driver_tim_base_irq(DRIVER_TIM2);
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM3);
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM4);
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM12);
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM8);
    driver_tim_base_irq(DRIVER_TIM13);
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM14);
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
    driver_tim_base_irq(DRIVER_TIM5);
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM6);
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
    driver_tim_base_irq(DRIVER_TIM7);
}
