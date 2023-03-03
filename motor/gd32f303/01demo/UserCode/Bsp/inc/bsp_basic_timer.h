#ifndef __BSP_BASIC_TIMER_H__
#define __BSP_BASIC_TIMER_H__

#include "gd32f30x.h"

/* 累计 TIM_Period个后产生一个更新或者中断*/
	//当定时器从0计数到BASIC_PERIOD_COUNT-1，即为BASIC_PERIOD_COUNT次，为一个定时周期
#define BASIC_PERIOD_COUNT  (5*10) // 1ms

// T = BASIC_PERIOD_COUNT * BASIC_PRESCALER_COUNT / TIMxCLK
//TIMxCLK=60MHz / 60 = 1MHz = 1us
// 实际测试 TIMxCLK=120Mhz 100us
#define BASIC_PRESCALER_COUNT   (12000)


void bsp_basic_timer_init(void);

#endif /* __BSP_BASIC_TIMER_H__ */
