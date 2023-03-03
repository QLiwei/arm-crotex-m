#ifndef __BSP_USART__
#define __BSP_USART__

#include "gd32f30x.h"
#include <stdio.h>

void board_console_init(void);
void Usart_SendByte(uint8_t ch);
void Usart_SendString(uint8_t *str, uint16_t size);

#endif

