/**
 * @file driver_uart.c
 * @brief urat driver
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-16       vector(vector_qiu@163.com)      first version
 *
 */

#include "driver_uart.h"

#if UART4_EN == 1
UART_HandleTypeDef huart4;
static void driver_uart4_init(void);

static void driver_uart4_init(void) {
    huart4.Instance = UART4;
    huart4.Init.BaudRate = UART4_BAUDRATE;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}
#endif /* UART4_EN */

#if UART5_EN == 1
UART_HandleTypeDef huart5;
static void driver_uart5_init(void);

static void driver_uart5_init(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = UART5_BAUDRATE;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}
#endif /* UART5_EN */

#if UART1_EN == 1
UART_HandleTypeDef huart1;
static void driver_usart1_init(void);

static void driver_usart1_init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART1_BAUDRATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
#endif /* UART1_EN */

#if UART2_EN
UART_HandleTypeDef huart2;
static void driver_uasrt2_init(void);

static void driver_usart2_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = UART2_BAUDRATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
#endif /* UART2_EN */

#if UART3_EN == 1
UART_HandleTypeDef huart3;
static void driver_uasrt3_init(void);

static void driver_uasrt3_init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = UART3_BAUDRATE;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}
#endif /* UART3_EN */

#if UART6_EN == 1
UART_HandleTypeDef huart6;
static void driver_uasrt6_init(void);

static void driver_usart6_init(void)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = UART6_BAUDRATE;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}
#endif /* UART6_EN */

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
 * @param uartHandle UART_HandleTypeDef handle structure
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
 * @param uartHandle
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
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
int driver_uart(void) {
#if UART4_EN == 1
    driver_uart4_init();
#endif /* UART4_EN */

#if UART5_EN == 1
    driver_uart5_init();
#endif /* UART5_EN */

#if UART1_EN == 1
    driver_usart1_init();
#endif /* UART1_EN */

#if UART2_EN == 1
    driver_usart2_init();
#endif /* UART2_EN */

#if UART3_EN == 1
    driver_uasrt3_init();
#endif /* UART3_EN */

#if UART6_EN == 1
    driver_usart6_init();
#endif
	return 0;
}
INIT_BOARD_EXPORT(driver_uart);
