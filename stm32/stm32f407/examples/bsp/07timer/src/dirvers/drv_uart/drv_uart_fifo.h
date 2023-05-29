/**
 * @file drv_uart_fifo.h
 * @brief urat fifo driver definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-17       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __DRV_UART_FIFO_H__
#define __DRV_UART_FIFO_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_FIFO_EN       1
#define DEBUG_COM           COM1
#define DEBUG_UART          USART1

#define UART4_FIFO_EN        1
#define UART4_FIFO_BAUDRATE  115200
#define UART4_RX_FIFO_SIZE   256
#define UART4_TX_FIFO_SIZE   256

#define UART5_FIFO_EN        1
#define UART5_FIFO_BAUDRATE  115200
#define UART5_RX_FIFO_SIZE   256
#define UART5_TX_FIFO_SIZE   256

#define UART1_FIFO_EN        1
#define UART1_FIFO_BAUDRATE  115200
#define UART1_RX_FIFO_SIZE   256
#define UART1_TX_FIFO_SIZE   256

#define UART2_FIFO_EN        1
#define UART2_FIFO_BAUDRATE  115200
#define UART2_RX_FIFO_SIZE   256
#define UART2_TX_FIFO_SIZE   256

#define UART3_FIFO_EN        1
#define UART3_FIFO_BAUDRATE  115200
#define UART3_RX_FIFO_SIZE   256
#define UART3_TX_FIFO_SIZE   256

#define UART6_FIFO_EN        1
#define UART6_FIFO_BAUDRATE  115200
#define UART6_RX_FIFO_SIZE   256
#define UART6_TX_FIFO_SIZE   256

typedef enum
{
	COM1 = 0,	/* USART1 */
	COM2 = 1,	/* USART2 */
	COM3 = 2,	/* USART3 */
	COM4 = 3,	/* UART4 */
	COM5 = 4,	/* UART5 */
	COM6 = 5,	/* USART6 */
}COM_PORT_E;

typedef struct drv_uart_fifo
{
	USART_TypeDef *uart;		        /* STM32内部串口设备指针 */
	uint8_t *p_tx_fifo;			        /* 发送缓冲区 */
	uint8_t *p_rx_fifo;			        /* 接收缓冲区 */
	uint16_t u16_tx_fifo_size;		    /* 发送缓冲区大小 */
	uint16_t u16_rx_fifo_size;		    /* 接收缓冲区大小 */
	__IO uint16_t u16_tx_write;	        /* 发送缓冲区写指针 */
	__IO uint16_t u16_tx_read;		    /* 发送缓冲区读指针 */
	__IO uint16_t u16_tx_count;	        /* 等待发送的数据个数 */

	__IO uint16_t u16_rx_write;	        /* 接收缓冲区写指针 */
	__IO uint16_t u16_rx_read;		    /* 接收缓冲区读指针 */
	__IO uint16_t u16_rx_count;	        /* 还未读取的新数据个数 */

	void (*send_befor)(void); 	        /* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
	void (*send_after)(void); 	        /* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
	void (*recive_new)(uint8_t _byte);	/* 串口收到数据的回调函数指针 */
	uint8_t sending;			        /* send state 1: senging */
}drv_uart_fifo_t;

void com_send_buf(COM_PORT_E _port, uint8_t *_buf, uint16_t _len);
void com_send_char(COM_PORT_E _port, uint8_t _byte);
error_t com_get_char(COM_PORT_E _port, uint8_t *_byte);
error_t com_clear_tx_fifo(COM_PORT_E _port);
error_t com_clear_rx_fifo(COM_PORT_E _port);
bool uart_tx_empty(COM_PORT_E _port);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_UART_FIFO_H__ */
