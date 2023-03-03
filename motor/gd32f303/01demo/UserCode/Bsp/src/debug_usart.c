#include "debug_usart.h"

void board_console_init(void) {
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    // rcu_periph_clock_enable(RCU_GPIOA);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    // UART Driver Version 1
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    nvic_irq_enable(USART0_IRQn, 0, 0);

    usart_enable(USART0);
}

void Usart_SendByte(uint8_t ch)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE)) {
        ;
    }
}

void Usart_SendString(uint8_t *str, uint16_t size)
{
	uint16_t count = 0;

	for(; count < size; count++)
	{
        Usart_SendByte(*str++);
	}
}

int fputc(int ch, FILE *f) {
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE)) {
        ;
    }
    return ch;
}
