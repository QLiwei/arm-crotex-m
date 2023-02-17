/**
 * @file driver_uart_fifo.c
 * @brief urat driver
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-17       vector(vector_qiu@163.com)      first version
 *
 */

#include "driver_uart_fifo.h"
#include <stdio.h>

#if (UART4_FIFO_EN == 1) || ( UART5_FIFO_EN == 1) || (UART1_FIFO_EN == 1) \
    || (UART2_FIFO_EN == 1) || (UART3_FIFO_EN == 1) || (UART6_FIFO_EN == 1)

static void uart_fifo_irq(driver_uart_fifo_t *dev);

#if UART4_FIFO_EN == 1
static uint8_t s_uart4_tx_fifo[UART4_TX_FIFO_SIZE];		/* 发送缓冲区 */
static uint8_t s_uart4_rx_fifo[UART4_RX_FIFO_SIZE];		/* 接收缓冲区 */
static driver_uart_fifo_t fifo_uart4 = {
    .uart = UART4,
    .p_tx_fifo = s_uart4_tx_fifo,
    .p_rx_fifo = s_uart4_rx_fifo,
    .u16_tx_fifo_size = UART4_TX_FIFO_SIZE,
    .u16_rx_fifo_size = UART4_RX_FIFO_SIZE,
    .u16_tx_write = 0,
    .u16_tx_read = 0,
    .u16_tx_count = 0,
    .u16_rx_write = 0,
    .u16_rx_read = 0,
    .u16_rx_count= 0,
    .send_befor = NULL,
    .send_after = NULL,
    .recive_new = NULL,
    .sending = 0,
};

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
    uart_fifo_irq(&fifo_uart4);
}
#endif /* UART4_FIFO_EN */

#if UART5_FIFO_EN == 1
static uint8_t s_uart5_tx_fifo[UART5_TX_FIFO_SIZE];		/* 发送缓冲区 */
static uint8_t s_uart5_rx_fifo[UART5_RX_FIFO_SIZE];		/* 接收缓冲区 */
static driver_uart_fifo_t fifo_uart5 = {
    .uart = UART5,
    .p_tx_fifo = s_uart5_tx_fifo,
    .p_rx_fifo = s_uart5_rx_fifo,
    .u16_tx_fifo_size = UART5_TX_FIFO_SIZE,
    .u16_rx_fifo_size = UART5_RX_FIFO_SIZE,
    .u16_tx_write = 0,
    .u16_tx_read = 0,
    .u16_tx_count = 0,
    .u16_rx_write = 0,
    .u16_rx_read = 0,
    .u16_rx_count= 0,
    .send_befor = NULL,
    .send_after = NULL,
    .recive_new = NULL,
    .sending = 0,
};

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
    uart_fifo_irq(&fifo_uart5);
}
#endif /* UART5_FIFO_EN */

#if UART1_FIFO_EN == 1
static uint8_t s_uart1_tx_fifo[UART1_TX_FIFO_SIZE];		/* 发送缓冲区 */
static uint8_t s_uart1_rx_fifo[UART1_RX_FIFO_SIZE];		/* 接收缓冲区 */
static driver_uart_fifo_t fifo_uart1 = {
    .uart = USART1,
    .p_tx_fifo = s_uart1_tx_fifo,
    .p_rx_fifo = s_uart1_rx_fifo,
    .u16_tx_fifo_size = UART1_TX_FIFO_SIZE,
    .u16_rx_fifo_size = UART1_RX_FIFO_SIZE,
    .u16_tx_write = 0,
    .u16_tx_read = 0,
    .u16_tx_count = 0,
    .u16_rx_write = 0,
    .u16_rx_read = 0,
    .u16_rx_count= 0,
    .send_befor = NULL,
    .send_after = NULL,
    .recive_new = NULL,
    .sending = 0,
};

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    uart_fifo_irq(&fifo_uart1);
}
#endif /* UART1_FIFO_EN */

#if UART2_FIFO_EN == 1
static uint8_t s_uart2_tx_fifo[UART2_TX_FIFO_SIZE];		/* 发送缓冲区 */
static uint8_t s_uart2_rx_fifo[UART2_RX_FIFO_SIZE];		/* 接收缓冲区 */
static driver_uart_fifo_t fifo_uart2 = {
    .uart = USART2,
    .p_tx_fifo = s_uart2_tx_fifo,
    .p_rx_fifo = s_uart2_rx_fifo,
    .u16_tx_fifo_size = UART2_TX_FIFO_SIZE,
    .u16_rx_fifo_size = UART2_RX_FIFO_SIZE,
    .u16_tx_write = 0,
    .u16_tx_read = 0,
    .u16_tx_count = 0,
    .u16_rx_write = 0,
    .u16_rx_read = 0,
    .u16_rx_count= 0,
    .send_befor = NULL,
    .send_after = NULL,
    .recive_new = NULL,
    .sending = 0,
};
/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    uart_fifo_irq(&fifo_uart2);
}
#endif /* UART2_FIFO_EN */

#if UART3_FIFO_EN == 1
static uint8_t s_uart3_tx_fifo[UART3_TX_FIFO_SIZE];		/* 发送缓冲区 */
static uint8_t s_uart3_rx_fifo[UART3_RX_FIFO_SIZE];		/* 接收缓冲区 */
static driver_uart_fifo_t fifo_uart3 = {
    .uart = USART3,
    .p_tx_fifo = s_uart3_tx_fifo,
    .p_rx_fifo = s_uart3_rx_fifo,
    .u16_tx_fifo_size = UART3_TX_FIFO_SIZE,
    .u16_rx_fifo_size = UART3_RX_FIFO_SIZE,
    .u16_tx_write = 0,
    .u16_tx_read = 0,
    .u16_tx_count = 0,
    .u16_rx_write = 0,
    .u16_rx_read = 0,
    .u16_rx_count= 0,
    .send_befor = NULL,
    .send_after = NULL,
    .recive_new = NULL,
    .sending = 0,
};
/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    uart_fifo_irq(&fifo_uart3);
}
#endif /* UART3_FIFO_EN */

#if UART6_FIFO_EN == 1
static uint8_t s_uart6_tx_fifo[UART6_TX_FIFO_SIZE];		/* 发送缓冲区 */
static uint8_t s_uart6_rx_fifo[UART6_RX_FIFO_SIZE];		/* 接收缓冲区 */
static driver_uart_fifo_t fifo_uart6 = {
    .uart = USART6,
    .p_tx_fifo = s_uart6_tx_fifo,
    .p_rx_fifo = s_uart6_rx_fifo,
    .u16_tx_fifo_size = UART6_TX_FIFO_SIZE,
    .u16_rx_fifo_size = UART6_RX_FIFO_SIZE,
    .u16_tx_write = 0,
    .u16_tx_read = 0,
    .u16_tx_count = 0,
    .u16_rx_write = 0,
    .u16_rx_read = 0,
    .u16_rx_count= 0,
    .send_befor = NULL,
    .send_after = NULL,
    .recive_new = NULL,
    .sending = 0,
};

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
    uart_fifo_irq(&fifo_uart6);
}
#endif /* UART6_FIFO_EN */

static void uart_send(driver_uart_fifo_t *_uart_fifo, uint8_t *_buf, uint16_t _len);
static error_t uart_get_char(driver_uart_fifo_t *_uart_fifo, uint8_t *_byte);
static driver_uart_fifo_t *com2fifo_uart(COM_PORT_E _port);

/**
 * @brief Serial interrupt handling function
 * @callgraph UARTx_IRQHandler()
 * @verbatim
 *
 * @param dev urat fifo driver struct
 */
static void uart_fifo_irq(driver_uart_fifo_t *dev) {
    uint32_t isrflags   = READ_REG(dev->uart->SR);
	uint32_t cr1its     = READ_REG(dev->uart->CR1);
	uint32_t cr3its     = READ_REG(dev->uart->CR3);

	/* Handling receive interrupts  */
    /* RXNE: Read data register not empty 1:You are ready to read the received data */
	if ((isrflags & USART_SR_RXNE) != RESET)
	{
		uint8_t ch;
        /* Read data from the serial port receive data register and store it to the receive FIFO */
		ch = READ_REG(dev->uart->DR);
		dev->p_rx_fifo[dev->u16_rx_write] = ch;
		if (++dev->u16_rx_write >= dev->u16_rx_fifo_size)
		{
			dev->u16_rx_write = 0;
		}
		if (dev->u16_rx_count < dev->u16_rx_fifo_size)
		{
			dev->u16_rx_count++;
		}

		/* 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记 */
		//if (dev->u16_rx_write == dev->u16_rx_read)
		//if (dev->u16_rx_count == 1)
		{
			if (dev->recive_new != NULL)
			{
				dev->recive_new(ch); /* 比如，交给MODBUS解码程序处理字节流 */
			}
		}
	}

	/* Handle send buffer over the empty */
    /* TXE:Transmit data register empty */
    /* TXEIE: TXE interrupt enable */
	if ( ((isrflags & USART_SR_TXE) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		//if (dev->u16_tx_read == dev->usTxWrite)
		if (dev->u16_tx_count == 0)
		{
			/* When the data of the send buffer has been fetched, it is forbidden to interrupt the send buffer over the empty */
            /* (note: the last data has not actually been sent at this time).*/
			CLEAR_BIT(dev->uart->CR1, USART_CR1_TXEIE);

			/* Interrupt when data has been sent */
			SET_BIT(dev->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			dev->sending = 1;

			/* One byte is taken from the send FIFO and written to the serial port send data register */
			dev->uart->DR = dev->p_tx_fifo[dev->u16_tx_read];
			if (++dev->u16_tx_read >= dev->u16_rx_fifo_size)
			{
				dev->u16_tx_read = 0;
			}
			dev->u16_tx_count--;
		}

	}
	/* Serial port send buffer data send completion interrupt */
    /* Transmission complete interrupt enable */
    /* TC: Transmission complete  */
	if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//if (dev->u16_tx_read == dev->u16_tx_write)
		if (dev->u16_tx_count == 0)
		{
			/* Serial fifo data transmission is completed，disable interrupt after data has been sent */
			CLEAR_BIT(dev->uart->CR1, USART_CR1_TCIE);

			/* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
			if (dev->send_after)
			{
				dev->send_after();
			}

			dev->sending = 0;
		}
		else
		{
			/* 正常情况下，不会进入此分支 */

			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
			//USART_SendData(dev->uart, dev->p_tx_fifo[dev->u16_tx_read]);
			dev->uart->DR = dev->p_tx_fifo[dev->u16_tx_read];
			if (++dev->u16_tx_read >= dev->u16_rx_fifo_size)
			{
				dev->u16_tx_read = 0;
			}
			dev->u16_tx_count--;
		}
	}
}

/**
 * @brief The urat fifo driver struct is found by the COM_PORT_E enumeration variable
 *
 * @param _port COMx
 * @return driver_uart_fifo_t* COMx corresponding urat fifo driver struct
 * @retval 0:urat fifo driver struct is not defined
 */
static driver_uart_fifo_t *com2fifo_uart(COM_PORT_E _port) {
    if (_port == COM1)
	{
		#if UART1_FIFO_EN == 1
			return &fifo_uart1;
		#else
			return 0;
		#endif
	}
	else if (_port == COM2)
	{
		#if UART2_FIFO_EN == 1
			return &fifo_uart2;
		#else
			return 0;
		#endif
	}
	else if (_port == COM3)
	{
		#if UART3_FIFO_EN == 1
			return &fifo_uart3;
		#else
			return 0;
		#endif
	}
	else if (_port == COM4)
	{
		#if UART4_FIFO_EN == 1
			return &fifo_uart4;
		#else
			return 0;
		#endif
	}
	else if (_port == COM5)
	{
		#if UART5_FIFO_EN == 1
			return &fifo_uart5;
		#else
			return 0;
		#endif
	}
	else if (_port == COM6)
	{
		#if UART6_FIFO_EN == 1
			return &fifo_uart6;
		#else
			return 0;
		#endif
	}
	else
	{
		Error_Handler(__FILE__, __LINE__);
		return 0;
	}
}

/**
 * @brief Send data of specified length from com port
 *
 * @param _port COMx
 * @param _buf Data buffer
 * @param _len Data length
 */
void com_send_buf(COM_PORT_E _port, uint8_t *_buf, uint16_t _len)
{
	driver_uart_fifo_t *uart_fifo;

	uart_fifo = com2fifo_uart(_port);
	if (uart_fifo == 0)
	{
		return;
	}

    /* The pre-send callback */
	if (uart_fifo->send_befor != NULL)
	{
		uart_fifo->send_befor();		/* 如果是RS485通信，可以在这个函数中将RS485设置为发送模式 */
	}

	uart_send(uart_fifo, _buf, _len);
}

/**
 * @brief Send a character through the com port
 *
 * @param _port COMx
 * @param _byte Character to send
 */
void com_send_char(COM_PORT_E _port, uint8_t _byte)
{
	com_send_buf(_port, &_byte, 1);
}

/**
 * @brief Receive a character through the com port
 *
 * @param _port COMx
 * @param _byte Character to be received
 * @return error_t EOK:Get data successful
 *              -EINVAL: Urat fifo driver struct is not defined
 *              -EEMPTY:There is no data to fetch
 */
error_t com_get_char(COM_PORT_E _port, uint8_t *_byte)
{
	driver_uart_fifo_t *uart_fifo;

	uart_fifo = com2fifo_uart(_port);
	if (uart_fifo == 0)
	{
		return -EINVAL;
	}

	return uart_get_char(uart_fifo, _byte);
}

/**
 * @brief Clear Tx FIFO
 *
 * @param _port COMx
 * @return error_t EOK:Clear Tx FIFO successful -EINVAL:Urat fifo driver struct is not defined
 */
error_t com_clear_tx_fifo(COM_PORT_E _port)
{
	driver_uart_fifo_t *uart_fifo;

	uart_fifo = com2fifo_uart(_port);
	if (uart_fifo == 0)
	{
		return -EINVAL;
	}

	uart_fifo->u16_tx_write = 0;
	uart_fifo->u16_tx_read = 0;
	uart_fifo->u16_tx_count = 0;
    return EOK;
}

/**
 * @brief Clear Rx FIFO
 *
 * @param _port COMx
 * @return error_t EOK:Clear Rx FIFO successful -EINVAL:Urat fifo driver struct is not defined
 */
error_t com_clear_rx_fifo(COM_PORT_E _port)
{
	driver_uart_fifo_t *uart_fifo;

	uart_fifo = com2fifo_uart(_port);
	if (uart_fifo == 0)
	{
		return -EINVAL;
	}

	uart_fifo->u16_rx_write = 0;
	uart_fifo->u16_rx_read = 0;
	uart_fifo->u16_rx_count = 0;
	return EOK;
}

/**
 * @brief Send the specified length data through the serial port
 *
 * @param _uart_fifo urat fifo struct
 * @param _buf Data buffer
 * @param _len Data length
 */
static void uart_send(driver_uart_fifo_t *_uart_fifo, uint8_t *_buf, uint16_t _len)
{
	uint16_t i;

	for (i = 0; i < _len; i++)
	{
		/* 如果发送缓冲区已经满了，则等待缓冲区空 */
		while (1)
		{
			__IO uint16_t count;

			DISABLE_INT();
			count = _uart_fifo->u16_tx_count;
			ENABLE_INT();

			if (count < _uart_fifo->u16_tx_fifo_size)
			{
				break;
			}
			else if(count == _uart_fifo->u16_tx_fifo_size)/* 数据已填满缓冲区 */
			{
                /* Enable transmit data register empty interrupt */
				if((_uart_fifo->uart->CR1 & USART_CR1_TXEIE) == 0)
				{
					SET_BIT(_uart_fifo->uart->CR1, USART_CR1_TXEIE);
				}
			}
		}

		/* Fill the send buffer with new data */
		_uart_fifo->p_tx_fifo[_uart_fifo->u16_tx_write] = _buf[i];

		DISABLE_INT();
		if (++_uart_fifo->u16_tx_write >= _uart_fifo->u16_tx_fifo_size)
		{
			_uart_fifo->u16_tx_write = 0;
		}
		_uart_fifo->u16_tx_count++;
		ENABLE_INT();
	}
    /* Enable transmit data register empty interrupt */
	SET_BIT(_uart_fifo->uart->CR1, USART_CR1_TXEIE);	/* 使能发送中断（缓冲区空） */
}

/**
 * @brief Get a character through the serial port
 *
 * @param _uart_fifo Urat fifo struct
 * @param _byte Save the received data
 * @return error_t EOK:Get the character successfully. -EEMPTY:No characters were received
 */
static error_t uart_get_char(driver_uart_fifo_t *_uart_fifo, uint8_t *_byte)
{
	uint16_t count;

	/* u16_rx_write 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 */
	DISABLE_INT();
	count = _uart_fifo->u16_rx_count;
	ENABLE_INT();

	/* 如果读和写索引相同，则返回0 */
	//if (_uart_fifo->u16_rx_read == u16_rx_write)
	if (count == 0)	/* 已经没有数据 */
	{
		return -EEMPTY;
	}
	else
	{
		*_byte = _uart_fifo->p_rx_fifo[_uart_fifo->u16_rx_read];		/* 从串口接收FIFO取1个数据 */

		/* 改写FIFO读索引 */
		DISABLE_INT();
		if (++_uart_fifo->u16_rx_read >= _uart_fifo->u16_rx_fifo_size)
		{
			_uart_fifo->u16_rx_read = 0;
		}
		_uart_fifo->u16_rx_count--;
		ENABLE_INT();
		return EOK;
	}
}

/**
 * @brief Get the status of data sent by com port, determine whether fifo is empty,
 *
 * @param _port COMx
 * @return true Uart Tx buffer is empty
 * @return false Senging
 */
bool uart_tx_empty(COM_PORT_E _port)
{
   uint8_t _sending;

   driver_uart_fifo_t *uart_fifo;

	uart_fifo = com2fifo_uart(_port);
	if (uart_fifo == 0)
	{
		return false;
	}

   _sending = uart_fifo->sending;

   if (_sending != 0)
   {
      return false;
   }
   return true;
}

/**
 * @brief Uart driver initialize
 *
 * @param dev Uart fifo struct
 */
static void driver_uart_init(driver_uart_fifo_t dev) {
    UART_HandleTypeDef huart;

    huart.Instance = dev.uart;
    huart.Init.BaudRate = UART4_FIFO_BAUDRATE;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
    CLEAR_BIT(dev.uart->SR, USART_SR_TC);       /* 清除TC发送完成标志 */
    CLEAR_BIT(dev.uart->SR, USART_SR_RXNE);     /* 清除RXNE接收标志 */
	SET_BIT(dev.uart->CR1, USART_CR1_RXNEIE);	/* 使能PE. RX接受中断 */
}

int fputc(int ch, FILE *f)
{
#if DEBUG_FIFO_EN == 1	/* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
	com_send_char(DEBUG_COM, ch);

	return ch;
#else	/* 采用阻塞方式发送每个字符,等待数据发送完毕 */
	/* 写一个字节到USART1 */
	DEBUG_UART->DR = ch;

	/* 等待发送结束 */
	while((DEBUG_UART->SR & USART_SR_TC) == 0)
	{}

	return ch;
#endif
}

int fgetc(FILE *f)
{

#if DEBUG_FIFO_EN == 1	/* 从串口接收FIFO中取1个数据, 只有取到数据才返回 */
	uint8_t ucData;

	while(com_get_char(DEBUG_COM, &ucData) == 0);

	return ucData;
#else
	/* 等待接收到数据 */
	while((DEBUG_UART->SR & USART_SR_RXNE) == 0)
	{}

	return (int)DEBUG_UART->DR;
#endif
}

/**
 * @brief  Initialize the UART low level resources by implementing the HAL_UART_MspInit()
 *
 * @verbatim
 *  (#) Enable the USARTx interface clock.
 *  (#) UART pins configuration:
 *      (+++) Enable the clock for the UART GPIOs.
 *      (+++) Configure the UART TX/RX pins as alternate function pull-up.
 *  (#) NVIC configuration if you need to use interrupt process (HAL_UART_Transmit_IT()
 *      and HAL_UART_Receive_IT() APIs):
 *      (+++)  Configure the USARTx interrupt priority.
 *      (+++)  Enable the NVIC USART IRQ handle.
 *  (#) DMA Configuration if you need to use DMA process (HAL_UART_Transmit_DMA()
 *      and HAL_UART_Receive_DMA() APIs):
 *      (+++) Declare a DMA handle structure for the Tx/Rx stream.
 *      (+++) Enable the DMAx interface clock.
 *      (+++) Configure the declared DMA handle structure with the required
 *              Tx/Rx parameters.
 *      (+++) Configure the DMA Tx/Rx stream.
 *      (+++) Associate the initialized DMA handle to the UART DMA Tx/Rx handle.
 *      (+++) Configure the priority and enable the NVIC for the transfer complete
 *              interrupt on the DMA Tx/Rx stream.
 *      (+++) Configure the USARTx interrupt priority and enable the NVIC USART IRQ handle
 *              (used for last byte sending completion detection in DMA non circular mode)
 *   (#) Program the Baud Rate, Word Length, Stop Bit, Parity, Hardware
 *       flow control and Mode(Receiver/Transmitter) in the huart Init structure.
 *
 * @param dev driver_uart_fifo_t handle structure
 */
void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance==UART4)
    {
        __HAL_RCC_UART4_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**UART4 GPIO Configuration
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART4 interrupt Init */
        HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
    else if(uartHandle->Instance==UART5)
    {
        /* UART5 clock enable */
        __HAL_RCC_UART5_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**UART5 GPIO Configuration
        PC12     ------> UART5_TX
        PD2     ------> UART5_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* UART5 interrupt Init */
        HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    else if(uartHandle->Instance==USART1)
    {
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if(uartHandle->Instance==USART2)
    {
        /* USART2 clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if(uartHandle->Instance==USART3)
    {
        /* USART3 clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if(uartHandle->Instance==USART6)
    {
        /* USART6 clock enable */
        __HAL_RCC_USART6_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART6 GPIO Configuration
        PC6     ------> USART6_TX
        PC7     ------> USART6_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART6 interrupt Init */
        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

/**
 * @brief UART Deinit
 *
 * @param dev
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if(uartHandle->Instance==UART4)
    {
        /* Peripheral clock disable */
        __HAL_RCC_UART4_CLK_DISABLE();

        /**UART4 GPIO Configuration
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

        /* UART4 interrupt Deinit */
        HAL_NVIC_DisableIRQ(UART4_IRQn);
    }
    else if(uartHandle->Instance==UART5)
    {
        /* Peripheral clock disable */
        __HAL_RCC_UART5_CLK_DISABLE();

        /**UART5 GPIO Configuration
        PC12     ------> UART5_TX
        PD2     ------> UART5_RX
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

        /* UART5 interrupt Deinit */
        HAL_NVIC_DisableIRQ(UART5_IRQn);
    }
    else if(uartHandle->Instance==USART1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

        /* USART1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
    else if(uartHandle->Instance==USART2)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

        /* USART2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
    else if(uartHandle->Instance==USART3)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

        /* USART3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
    else if(uartHandle->Instance==USART6)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART6_CLK_DISABLE();

        /**USART6 GPIO Configuration
        PC6     ------> USART6_TX
        PC7     ------> USART6_RX
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

        /* USART6 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART6_IRQn);
    }
}

/**
 * @brief The uart driver is initialized automatically
 *
 * @return int
 */
int uart_fifo_driver_init(void) {
#if UART4_FIFO_EN == 1
    driver_uart_init(fifo_uart4);
#endif /* UART4_FIFO_EN */

#if UART5_FIFO_EN == 1
    driver_uart_init(fifo_uart5);
#endif /* UART5_FIFO_EN */

#if UART1_FIFO_EN == 1
    driver_uart_init(fifo_uart1);
#endif /* UART1_FIFO_EN */

#if UART2_FIFO_EN == 1
    driver_uart_init(fifo_uart2);
#endif /* UART2_FIFO_EN */

#if UART3_FIFO_EN == 1
    driver_uart_init(fifo_uart3);
#endif /* UART3_FIFO_EN */

#if UART6_FIFO_EN == 1
    driver_uart_init(fifo_uart6);
#endif /* UART6_FIFO_EN */
	return 0;
}
INIT_BOARD_EXPORT(uart_fifo_driver_init);

#endif /* UARTx_FIFO_EN*/
