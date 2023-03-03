#include "bsp_encoder.h"

volatile int32_t over_under_flowcount = 0;
/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA6 PA7 (TIMER2 CH0 CH1) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
    gpio_init(GPIOA,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_7);
}

/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void timer_config(void)
{
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;
    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = ENCODER_TIM_PRESCALER;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = ENCODER_TIM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER1 CH0 CH1 input capture configuration */
    timer_icinitpara.icfilter    = 0x05;//配置滤波器
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_input_capture_config(TIMER2, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER2, TIMER_CH_1, &timer_icinitpara);

    timer_quadrature_decoder_mode_config(TIMER2, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING);
	timer_slave_mode_select(TIMER2,TIMER_ENCODER_MODE2);

    timer_flag_clear(TIMER2, TIMER_FLAG_UP);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);//使能中断并设置优先级

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);

    timer_enable(TIMER2);
}

void TIMER2_IRQHandler(void) {
    if (timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP) != RESET) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        if (TIMER_CTL0(TIMER2) & TIMER_CTL0_DIR) { // down
            over_under_flowcount--;
        } else {
            over_under_flowcount++;
        }
    }
}

int32_t bsp_encoder_get_current_count(void) {
    return timer_counter_read(TIMER2) + (over_under_flowcount * ENCODER_TIM_PERIOD);
}

void bsp_encoder_init(void) {
    gpio_config();
    timer_config();
}
