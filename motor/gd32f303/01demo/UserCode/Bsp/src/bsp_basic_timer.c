#include "bsp_basic_timer.h"


/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void timer_config(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER6);
    timer_deinit(TIMER6);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = BASIC_PRESCALER_COUNT-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = BASIC_PERIOD_COUNT-1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER6,&timer_initpara);

    nvic_irq_enable(TIMER6_IRQn, 0, 1);			    /* TIMER1中断设置，抢占优先级0，子优先级3 */
	timer_interrupt_enable(TIMER6,TIMER_INT_UP);	/* 使能更新中断 */
    timer_interrupt_flag_clear(TIMER6,TIMER_INT_FLAG_UP);

    /* auto-reload preload enable */
    timer_enable(TIMER6);
}


void bsp_basic_timer_init(void) {
    timer_config();
}







