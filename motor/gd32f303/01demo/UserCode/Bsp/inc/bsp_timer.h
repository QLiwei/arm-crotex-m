#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "gd32f30x.h"

/* timer frequency 120MHz */
/* motor 15KHz */
/* fre = timer_clk / per * pre */
#define TIMER_PRESCALER     1
#define TIMER_PERIOD        8000

void bsp_timer_init(void);

#endif /* BSP_TIMER_H__ */

