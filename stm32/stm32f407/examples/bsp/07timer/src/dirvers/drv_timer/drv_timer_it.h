#ifndef __DRV_TIMER_IT_H__
#define __DRV_TIMER_IT_H__

#include "drv_timer.h"
#ifdef __cplusplus
 extern "C" {
#endif

void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* __drv_TIMER_IT_H__ */