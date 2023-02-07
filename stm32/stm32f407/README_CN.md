# 1.软件包下载

[STM32CubeF4](https://github.com/STMicroelectronics/STM32CubeF4) 

1. www.st.com 检索 STM32CubeF4

2. 选择需要下载的版本

3. 填写邮箱，打开邮箱中的下载链接

   

# 2.CMSIS软件包下载

1. [CMSIS_5](https://github.com/ARM-software/CMSIS_5) 
2. STM32CubeF4软件包 CMSIS路径下
3. MDK C:\Users\HP\AppData\Local\Arm\Packs\ARM\CMSIS\5.7.0\CMSIS路径下

# 3.HAL库框架设计学习

## 3.1 配置文件

```c
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf.h
  * @author  MCD Application Team
  * @brief   HAL configuration file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver 
  */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
/* #define HAL_CAN_LEGACY_MODULE_ENABLED */
#define HAL_CRC_MODULE_ENABLED
#define HAL_CRYP_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_DCMI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
/* #define HAL_DMA2D_MODULE_ENABLED */
/* #define HAL_ETH_MODULE_ENABLED */
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_NAND_MODULE_ENABLED
#define HAL_NOR_MODULE_ENABLED
#define HAL_PCCARD_MODULE_ENABLED
#define HAL_SRAM_MODULE_ENABLED
/* #define HAL_SDRAM_MODULE_ENABLED */
#define HAL_HASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_I2S_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
/* #define HAL_LTDC_MODULE_ENABLED */
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
/* #define HAL_SAI_MODULE_ENABLED */
#define HAL_SD_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_USART_MODULE_ENABLED
#define HAL_IRDA_MODULE_ENABLED
#define HAL_SMARTCARD_MODULE_ENABLED
#define HAL_WWDG_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_HCD_MODULE_ENABLED


/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).  
  */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    (8000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    (100U)   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL). 
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    (16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE) 
 #define LSI_VALUE  (32000U)    
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  (32768U)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    (5000U)   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source 
  *        frequency, this source is inserted directly through I2S_CKIN pad. 
  */
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    (12288000U) /*!< Value of the External oscillator in Hz*/
#endif /* EXTERNAL_CLOCK_VALUE */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */     
#define  VDD_VALUE                    (3300U) /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            (0x0FU) /*!< tick interrupt priority */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              0U /* The prefetch will be enabled in SystemClock_Config(), depending on the used 
                                            STM32F405/415/07/417 device: RevA (prefetch must be off) or RevZ (prefetch can be on/off) */
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

#define  USE_HAL_ADC_REGISTER_CALLBACKS         0U /* ADC register callback disabled       */
#define  USE_HAL_CAN_REGISTER_CALLBACKS         0U /* CAN register callback disabled       */
#define  USE_HAL_CEC_REGISTER_CALLBACKS         0U /* CEC register callback disabled       */
#define  USE_HAL_CRYP_REGISTER_CALLBACKS        0U /* CRYP register callback disabled      */
#define  USE_HAL_DAC_REGISTER_CALLBACKS         0U /* DAC register callback disabled       */
#define  USE_HAL_DCMI_REGISTER_CALLBACKS        0U /* DCMI register callback disabled      */
#define  USE_HAL_DFSDM_REGISTER_CALLBACKS       0U /* DFSDM register callback disabled     */
#define  USE_HAL_DMA2D_REGISTER_CALLBACKS       0U /* DMA2D register callback disabled     */
#define  USE_HAL_DSI_REGISTER_CALLBACKS         0U /* DSI register callback disabled       */
#define  USE_HAL_ETH_REGISTER_CALLBACKS         0U /* ETH register callback disabled       */
#define  USE_HAL_HASH_REGISTER_CALLBACKS        0U /* HASH register callback disabled      */
#define  USE_HAL_HCD_REGISTER_CALLBACKS         0U /* HCD register callback disabled       */
#define  USE_HAL_I2C_REGISTER_CALLBACKS         0U /* I2C register callback disabled       */
#define  USE_HAL_FMPI2C_REGISTER_CALLBACKS      0U /* FMPI2C register callback disabled    */
#define  USE_HAL_I2S_REGISTER_CALLBACKS         0U /* I2S register callback disabled       */
#define  USE_HAL_IRDA_REGISTER_CALLBACKS        0U /* IRDA register callback disabled      */
#define  USE_HAL_LPTIM_REGISTER_CALLBACKS       0U /* LPTIM register callback disabled     */
#define  USE_HAL_LTDC_REGISTER_CALLBACKS        0U /* LTDC register callback disabled      */
#define  USE_HAL_MMC_REGISTER_CALLBACKS         0U /* MMC register callback disabled       */
#define  USE_HAL_NAND_REGISTER_CALLBACKS        0U /* NAND register callback disabled      */
#define  USE_HAL_NOR_REGISTER_CALLBACKS         0U /* NOR register callback disabled       */
#define  USE_HAL_PCCARD_REGISTER_CALLBACKS      0U /* PCCARD register callback disabled    */
#define  USE_HAL_PCD_REGISTER_CALLBACKS         0U /* PCD register callback disabled       */
#define  USE_HAL_QSPI_REGISTER_CALLBACKS        0U /* QSPI register callback disabled      */
#define  USE_HAL_RNG_REGISTER_CALLBACKS         0U /* RNG register callback disabled       */
#define  USE_HAL_RTC_REGISTER_CALLBACKS         0U /* RTC register callback disabled       */
#define  USE_HAL_SAI_REGISTER_CALLBACKS         0U /* SAI register callback disabled       */
#define  USE_HAL_SD_REGISTER_CALLBACKS          0U /* SD register callback disabled        */
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U /* SMARTCARD register callback disabled */
#define  USE_HAL_SDRAM_REGISTER_CALLBACKS       0U /* SDRAM register callback disabled     */
#define  USE_HAL_SRAM_REGISTER_CALLBACKS        0U /* SRAM register callback disabled      */
#define  USE_HAL_SPDIFRX_REGISTER_CALLBACKS     0U /* SPDIFRX register callback disabled   */
#define  USE_HAL_SMBUS_REGISTER_CALLBACKS       0U /* SMBUS register callback disabled     */
#define  USE_HAL_SPI_REGISTER_CALLBACKS         0U /* SPI register callback disabled       */
#define  USE_HAL_TIM_REGISTER_CALLBACKS         0U /* TIM register callback disabled       */
#define  USE_HAL_UART_REGISTER_CALLBACKS        0U /* UART register callback disabled      */
#define  USE_HAL_USART_REGISTER_CALLBACKS       0U /* USART register callback disabled     */
#define  USE_HAL_WWDG_REGISTER_CALLBACKS        0U /* WWDG register callback disabled      */

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

#define USE_SPI_CRC                     1U

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */
   
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_can_legacy.h"
#endif /* HAL_CAN_LEGACY_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f4xx_hal_cryp.h" 
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* HAL_DMA2D_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* HAL_ETH_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */
 
#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* HAL_NAND_MODULE_ENABLED */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* HAL_PCCARD_MODULE_ENABLED */ 
  
#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* HAL_SDRAM_MODULE_ENABLED */

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* HAL_HASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* HAL_LTDC_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* HAL_SAI_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* HAL_HCD_MODULE_ENABLED */
   
/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */


#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */

```

1. 外部震荡器配置 **HSE_VALUE** 配置值为实际晶振的频率

2. **TICK_INT_PRIORITY** 系统滴答定时器的优先级设置

   滴答定时器的优先级设置。这个优先级的设置至关重要。因为 HAL 库中各个外设驱动里面

   的延迟实现是基于此文件提供的时间基准。

   如果在中断服务程序里面调用基于此时间基准的延迟函数 HAL_Delay 要特别注意，因为这个函数的时间基准是基于滴答定时器或者其他通用定时器实现，实现方式是滴答定时器或者其他通用定时器里面对变量计数。如此以来，结果是显而易见的，如果其他中断服务程序调用了此函数，且中断优先级高于滴答定时器，会导致滴答定时器中断服务程序一直得不到执行，从而卡死在里面。所以滴答定时器的中断优先级一定要比他们高。

   另外这个时间基准既可以使用滴答定时器实现也可以使用通用的定时器实现，默认情况下是用的滴答定时器。

3. **USE_RTOS**是否使用RTOS

4. 断言功能，定义USE_FULL_ASSERT宏打开断言检测，需要自己实现assert_failed函数

   ```c
   /*
   *********************************************************************************************************
   * 函 数 名: assert_failed
   * 形 参：file : 源代码文件名称。关键字__FILE__表示源代码文件名。
   * line ：代码行号。关键字 __LINE__ 表示源代码行号
   * 返 回 值: 无
   *********************************************************************************************************
   */
   void assert_failed(uint8_t* file, uint32_t line)
   { 
       /* 
           用户可以添加自己的代码报告源代码文件名和代码行号，比如将错误文件和行号打印到串口
           printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       */
       /* 这是一个死循环，断言失败时程序会在此处死机，以便于用户查错 */
       while (1)
       {
       }
   }
   ```

   

## 3.2 时间基准

HAL 库专门搞了一个时间基准，默认来源是滴答定时器，也可以通过重定向使用其他定时器实现。相关函数全部集中在 stm32f4xx_hal.c 文件里面实现

```c

/**
  * @brief This function configures the source of the time base.
  *        The time source is configured  to have 1ms time base with a dedicated 
  *        Tick interrupt priority.
  * @note This function is called  automatically at the beginning of program after
  *       reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig().
  * @note In the default implementation, SysTick timer is the source of time base. 
  *       It is used to generate interrupts at regular time intervals. 
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process, 
  *       The SysTick interrupt must have higher priority (numerically lower)
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority Tick interrupt priority.
  * @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Return function status */
  return HAL_OK;
}
```

## 3.3 启动流程

```c
/**
  * @brief  This function is used to initialize the HAL Library; it must be the first 
  *         instruction to be executed in the main program (before to call any other
  *         HAL function), it performs the following:
  *           Configure the Flash prefetch, instruction and Data caches.
  *           Configures the SysTick to generate an interrupt each 1 millisecond,
  *           which is clocked by the HSI (at this stage, the clock is not yet
  *           configured and thus the system is running from the internal HSI at 16 MHz).
  *           Set NVIC Group Priority to 4.
  *           Calls the HAL_MspInit() callback function defined in user file 
  *           "stm32f4xx_hal_msp.c" to do the global low level hardware initialization 
  *            
  * @note   SysTick is used as time base for the HAL_Delay() function, the application
  *         need to ensure that the SysTick time base is always set to 1 millisecond
  *         to have correct HAL operation.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_Init(void)
{
  /* Configure Flash prefetch, Instruction cache, Data cache */ 
#if (INSTRUCTION_CACHE_ENABLE != 0U)
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif /* INSTRUCTION_CACHE_ENABLE */

#if (DATA_CACHE_ENABLE != 0U)
  __HAL_FLASH_DATA_CACHE_ENABLE();
#endif /* DATA_CACHE_ENABLE */

#if (PREFETCH_ENABLE != 0U)
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  HAL_MspInit();

  /* Return function status */
  return HAL_OK;
}
```

**用户也要使用滴答定时器中断**

## 3.4 初始化外设

HAL 库为外设初始化提供了一套框架，在调用HAL_UART_Init初始化串口，此函数就会调用HAL_UART_MspInit

如果要初始化，直接将此函数在其它源文件里面实现

```c
/**
  * @brief  UART MSP Init.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_MspInit could be implemented in the user file
   */
}

/**
  * @brief  UART MSP DeInit.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_MspDeInit could be implemented in the user file
   */
}

```

```c
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance==USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(huart->Instance==USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}
```

## 3.5 中断处理

加强对中断的管理，HAL 库也为各种外设中断也配套一个函数

```c
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    /* USER CODE END USART1_IRQn 0 */
    /* 参数是串口句柄 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */
    /* USER CODE END USART1_IRQn 1 */
}
```

用户可以直接在函数HAL_UART_IRQHandler 的前面或者后面添加新代码，也可以直接在 HAL_UART_IRQHandler 调用的各种回调函数里面执行，这些回调都是弱定义的，方便用户直接在其它文件里面重定义

## 3.6 DMA处理

为了方便各种外设直接启动 DMA，HAL 库专门为支持 DMA 操作的外设都提供了对应的 DMA 函数

```c
HAL_UART_Transmit_DMA()
HAL_UART_Receive_DMA()
HAL_UART_DMAPause()
HAL_UART_DMAResume()
HAL_UART_DMAStop()
```

针对外设的 DMA 函数基本都有开启中断，如果用户使能此外设的 NVIC，使用中务必别忘了写 DMA 的中断服务程序

```c
void DMA1_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    /* 参数是 DMA 句柄 */
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}
```

在 DMA 传输完成，半传输完成等中断里面执行功能，也是通过 HAL_DMA_IRQHandler 调用的各种回调函数里面实现

```c
HAL_UART_TxHalfCpltCallback()
HAL_UART_TxCpltCallback()
HAL_UART_RxHalfCpltCallback()
HAL_UART_RxCpltCallback()
```

# 4.STM32F4启动过程

启动过程是指从 CPU 上电复位执行第 1 条指令开始（汇编文件）到进入 C 程序 main()函数入口之间的部分

**小技能**:遇到不认识的指令或者关键词可以检索。**启动 MDK 软件，在 Help 菜单点击 uVision Help**

**startup_stmf407xx.s**

```s
;*******************************************************************************
;* File Name          : startup_stm32f407xx.s
;* Author             : MCD Application Team
;* Description        : STM32F407xx devices vector table for MDK-ARM toolchain. 
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the CortexM4 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;*******************************************************************************
;* @attention
;*
;* Copyright (c) 2017 STMicroelectronics.
;* All rights reserved.
;*
;* This software is licensed under terms that can be found in the LICENSE file
;* in the root directory of this software component.
;* If no LICENSE file comes with this software, it is provided AS-IS.
;*
;*******************************************************************************
;* <<< Use Configuration Wizard in Context Menu >>>
;
; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     PendSV_Handler             ; PendSV Handler
                DCD     SysTick_Handler            ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler                   ; Window WatchDog                                        
                DCD     PVD_IRQHandler                    ; PVD through EXTI Line detection                        
                DCD     TAMP_STAMP_IRQHandler             ; Tamper and TimeStamps through the EXTI line            
                DCD     RTC_WKUP_IRQHandler               ; RTC Wakeup through the EXTI line                       
                DCD     FLASH_IRQHandler                  ; FLASH                                           
                DCD     RCC_IRQHandler                    ; RCC                                             
                DCD     EXTI0_IRQHandler                  ; EXTI Line0                                             
                DCD     EXTI1_IRQHandler                  ; EXTI Line1                                             
                DCD     EXTI2_IRQHandler                  ; EXTI Line2                                             
                DCD     EXTI3_IRQHandler                  ; EXTI Line3                                             
                DCD     EXTI4_IRQHandler                  ; EXTI Line4                                             
                DCD     DMA1_Stream0_IRQHandler           ; DMA1 Stream 0                                   
                DCD     DMA1_Stream1_IRQHandler           ; DMA1 Stream 1                                   
                DCD     DMA1_Stream2_IRQHandler           ; DMA1 Stream 2                                   
                DCD     DMA1_Stream3_IRQHandler           ; DMA1 Stream 3                                   
                DCD     DMA1_Stream4_IRQHandler           ; DMA1 Stream 4                                   
                DCD     DMA1_Stream5_IRQHandler           ; DMA1 Stream 5                                   
                DCD     DMA1_Stream6_IRQHandler           ; DMA1 Stream 6                                   
                DCD     ADC_IRQHandler                    ; ADC1, ADC2 and ADC3s                            
                DCD     CAN1_TX_IRQHandler                ; CAN1 TX                                                
                DCD     CAN1_RX0_IRQHandler               ; CAN1 RX0                                               
                DCD     CAN1_RX1_IRQHandler               ; CAN1 RX1                                               
                DCD     CAN1_SCE_IRQHandler               ; CAN1 SCE                                               
                DCD     EXTI9_5_IRQHandler                ; External Line[9:5]s                                    
                DCD     TIM1_BRK_TIM9_IRQHandler          ; TIM1 Break and TIM9                   
                DCD     TIM1_UP_TIM10_IRQHandler          ; TIM1 Update and TIM10                 
                DCD     TIM1_TRG_COM_TIM11_IRQHandler     ; TIM1 Trigger and Commutation and TIM11
                DCD     TIM1_CC_IRQHandler                ; TIM1 Capture Compare                                   
                DCD     TIM2_IRQHandler                   ; TIM2                                            
                DCD     TIM3_IRQHandler                   ; TIM3                                            
                DCD     TIM4_IRQHandler                   ; TIM4                                            
                DCD     I2C1_EV_IRQHandler                ; I2C1 Event                                             
                DCD     I2C1_ER_IRQHandler                ; I2C1 Error                                             
                DCD     I2C2_EV_IRQHandler                ; I2C2 Event                                             
                DCD     I2C2_ER_IRQHandler                ; I2C2 Error                                               
                DCD     SPI1_IRQHandler                   ; SPI1                                            
                DCD     SPI2_IRQHandler                   ; SPI2                                            
                DCD     USART1_IRQHandler                 ; USART1                                          
                DCD     USART2_IRQHandler                 ; USART2                                          
                DCD     USART3_IRQHandler                 ; USART3                                          
                DCD     EXTI15_10_IRQHandler              ; External Line[15:10]s                                  
                DCD     RTC_Alarm_IRQHandler              ; RTC Alarm (A and B) through EXTI Line                  
                DCD     OTG_FS_WKUP_IRQHandler            ; USB OTG FS Wakeup through EXTI line                        
                DCD     TIM8_BRK_TIM12_IRQHandler         ; TIM8 Break and TIM12                  
                DCD     TIM8_UP_TIM13_IRQHandler          ; TIM8 Update and TIM13                 
                DCD     TIM8_TRG_COM_TIM14_IRQHandler     ; TIM8 Trigger and Commutation and TIM14
                DCD     TIM8_CC_IRQHandler                ; TIM8 Capture Compare                                   
                DCD     DMA1_Stream7_IRQHandler           ; DMA1 Stream7                                           
                DCD     FMC_IRQHandler                    ; FMC                                             
                DCD     SDIO_IRQHandler                   ; SDIO                                            
                DCD     TIM5_IRQHandler                   ; TIM5                                            
                DCD     SPI3_IRQHandler                   ; SPI3                                            
                DCD     UART4_IRQHandler                  ; UART4                                           
                DCD     UART5_IRQHandler                  ; UART5                                           
                DCD     TIM6_DAC_IRQHandler               ; TIM6 and DAC1&2 underrun errors                   
                DCD     TIM7_IRQHandler                   ; TIM7                   
                DCD     DMA2_Stream0_IRQHandler           ; DMA2 Stream 0                                   
                DCD     DMA2_Stream1_IRQHandler           ; DMA2 Stream 1                                   
                DCD     DMA2_Stream2_IRQHandler           ; DMA2 Stream 2                                   
                DCD     DMA2_Stream3_IRQHandler           ; DMA2 Stream 3                                   
                DCD     DMA2_Stream4_IRQHandler           ; DMA2 Stream 4                                   
                DCD     ETH_IRQHandler                    ; Ethernet                                        
                DCD     ETH_WKUP_IRQHandler               ; Ethernet Wakeup through EXTI line                      
                DCD     CAN2_TX_IRQHandler                ; CAN2 TX                                                
                DCD     CAN2_RX0_IRQHandler               ; CAN2 RX0                                               
                DCD     CAN2_RX1_IRQHandler               ; CAN2 RX1                                               
                DCD     CAN2_SCE_IRQHandler               ; CAN2 SCE                                               
                DCD     OTG_FS_IRQHandler                 ; USB OTG FS                                      
                DCD     DMA2_Stream5_IRQHandler           ; DMA2 Stream 5                                   
                DCD     DMA2_Stream6_IRQHandler           ; DMA2 Stream 6                                   
                DCD     DMA2_Stream7_IRQHandler           ; DMA2 Stream 7                                   
                DCD     USART6_IRQHandler                 ; USART6                                           
                DCD     I2C3_EV_IRQHandler                ; I2C3 event                                             
                DCD     I2C3_ER_IRQHandler                ; I2C3 error                                             
                DCD     OTG_HS_EP1_OUT_IRQHandler         ; USB OTG HS End Point 1 Out                      
                DCD     OTG_HS_EP1_IN_IRQHandler          ; USB OTG HS End Point 1 In                       
                DCD     OTG_HS_WKUP_IRQHandler            ; USB OTG HS Wakeup through EXTI                         
                DCD     OTG_HS_IRQHandler                 ; USB OTG HS                                      
                DCD     DCMI_IRQHandler                   ; DCMI  
                DCD     0                                 ; Reserved				                              
                DCD     HASH_RNG_IRQHandler               ; Hash and Rng
                DCD     FPU_IRQHandler                    ; FPU
                
                                         
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
        IMPORT  SystemInit
        IMPORT  __main

                 LDR     R0, =SystemInit
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler                   [WEAK]                                        
                EXPORT  PVD_IRQHandler                    [WEAK]                      
                EXPORT  TAMP_STAMP_IRQHandler             [WEAK]         
                EXPORT  RTC_WKUP_IRQHandler               [WEAK]                     
                EXPORT  FLASH_IRQHandler                  [WEAK]                                         
                EXPORT  RCC_IRQHandler                    [WEAK]                                            
                EXPORT  EXTI0_IRQHandler                  [WEAK]                                            
                EXPORT  EXTI1_IRQHandler                  [WEAK]                                             
                EXPORT  EXTI2_IRQHandler                  [WEAK]                                            
                EXPORT  EXTI3_IRQHandler                  [WEAK]                                           
                EXPORT  EXTI4_IRQHandler                  [WEAK]                                            
                EXPORT  DMA1_Stream0_IRQHandler           [WEAK]                                
                EXPORT  DMA1_Stream1_IRQHandler           [WEAK]                                   
                EXPORT  DMA1_Stream2_IRQHandler           [WEAK]                                   
                EXPORT  DMA1_Stream3_IRQHandler           [WEAK]                                   
                EXPORT  DMA1_Stream4_IRQHandler           [WEAK]                                   
                EXPORT  DMA1_Stream5_IRQHandler           [WEAK]                                   
                EXPORT  DMA1_Stream6_IRQHandler           [WEAK]                                   
                EXPORT  ADC_IRQHandler                    [WEAK]                         
                EXPORT  CAN1_TX_IRQHandler                [WEAK]                                                
                EXPORT  CAN1_RX0_IRQHandler               [WEAK]                                               
                EXPORT  CAN1_RX1_IRQHandler               [WEAK]                                                
                EXPORT  CAN1_SCE_IRQHandler               [WEAK]                                                
                EXPORT  EXTI9_5_IRQHandler                [WEAK]                                    
                EXPORT  TIM1_BRK_TIM9_IRQHandler          [WEAK]                  
                EXPORT  TIM1_UP_TIM10_IRQHandler          [WEAK]                
                EXPORT  TIM1_TRG_COM_TIM11_IRQHandler     [WEAK] 
                EXPORT  TIM1_CC_IRQHandler                [WEAK]                                   
                EXPORT  TIM2_IRQHandler                   [WEAK]                                            
                EXPORT  TIM3_IRQHandler                   [WEAK]                                            
                EXPORT  TIM4_IRQHandler                   [WEAK]                                            
                EXPORT  I2C1_EV_IRQHandler                [WEAK]                                             
                EXPORT  I2C1_ER_IRQHandler                [WEAK]                                             
                EXPORT  I2C2_EV_IRQHandler                [WEAK]                                            
                EXPORT  I2C2_ER_IRQHandler                [WEAK]                                               
                EXPORT  SPI1_IRQHandler                   [WEAK]                                           
                EXPORT  SPI2_IRQHandler                   [WEAK]                                            
                EXPORT  USART1_IRQHandler                 [WEAK]                                          
                EXPORT  USART2_IRQHandler                 [WEAK]                                          
                EXPORT  USART3_IRQHandler                 [WEAK]                                         
                EXPORT  EXTI15_10_IRQHandler              [WEAK]                                  
                EXPORT  RTC_Alarm_IRQHandler              [WEAK]                  
                EXPORT  OTG_FS_WKUP_IRQHandler            [WEAK]                        
                EXPORT  TIM8_BRK_TIM12_IRQHandler         [WEAK]                 
                EXPORT  TIM8_UP_TIM13_IRQHandler          [WEAK]                 
                EXPORT  TIM8_TRG_COM_TIM14_IRQHandler     [WEAK] 
                EXPORT  TIM8_CC_IRQHandler                [WEAK]                                   
                EXPORT  DMA1_Stream7_IRQHandler           [WEAK]                                          
                EXPORT  FMC_IRQHandler                    [WEAK]                                             
                EXPORT  SDIO_IRQHandler                   [WEAK]                                             
                EXPORT  TIM5_IRQHandler                   [WEAK]                                             
                EXPORT  SPI3_IRQHandler                   [WEAK]                                             
                EXPORT  UART4_IRQHandler                  [WEAK]                                            
                EXPORT  UART5_IRQHandler                  [WEAK]                                            
                EXPORT  TIM6_DAC_IRQHandler               [WEAK]                   
                EXPORT  TIM7_IRQHandler                   [WEAK]                    
                EXPORT  DMA2_Stream0_IRQHandler           [WEAK]                                  
                EXPORT  DMA2_Stream1_IRQHandler           [WEAK]                                   
                EXPORT  DMA2_Stream2_IRQHandler           [WEAK]                                    
                EXPORT  DMA2_Stream3_IRQHandler           [WEAK]                                    
                EXPORT  DMA2_Stream4_IRQHandler           [WEAK]                                 
                EXPORT  ETH_IRQHandler                    [WEAK]                                         
                EXPORT  ETH_WKUP_IRQHandler               [WEAK]                     
                EXPORT  CAN2_TX_IRQHandler                [WEAK]                                               
                EXPORT  CAN2_RX0_IRQHandler               [WEAK]                                               
                EXPORT  CAN2_RX1_IRQHandler               [WEAK]                                               
                EXPORT  CAN2_SCE_IRQHandler               [WEAK]                                               
                EXPORT  OTG_FS_IRQHandler                 [WEAK]                                       
                EXPORT  DMA2_Stream5_IRQHandler           [WEAK]                                   
                EXPORT  DMA2_Stream6_IRQHandler           [WEAK]                                   
                EXPORT  DMA2_Stream7_IRQHandler           [WEAK]                                   
                EXPORT  USART6_IRQHandler                 [WEAK]                                           
                EXPORT  I2C3_EV_IRQHandler                [WEAK]                                              
                EXPORT  I2C3_ER_IRQHandler                [WEAK]                                              
                EXPORT  OTG_HS_EP1_OUT_IRQHandler         [WEAK]                      
                EXPORT  OTG_HS_EP1_IN_IRQHandler          [WEAK]                      
                EXPORT  OTG_HS_WKUP_IRQHandler            [WEAK]                        
                EXPORT  OTG_HS_IRQHandler                 [WEAK]                                      
                EXPORT  DCMI_IRQHandler                   [WEAK]                                                                                 
                EXPORT  HASH_RNG_IRQHandler               [WEAK]
                EXPORT  FPU_IRQHandler                    [WEAK]
                
WWDG_IRQHandler                                                       
PVD_IRQHandler                                      
TAMP_STAMP_IRQHandler                  
RTC_WKUP_IRQHandler                                
FLASH_IRQHandler                                                       
RCC_IRQHandler                                                            
EXTI0_IRQHandler                                                          
EXTI1_IRQHandler                                                           
EXTI2_IRQHandler                                                          
EXTI3_IRQHandler                                                         
EXTI4_IRQHandler                                                          
DMA1_Stream0_IRQHandler                                       
DMA1_Stream1_IRQHandler                                          
DMA1_Stream2_IRQHandler                                          
DMA1_Stream3_IRQHandler                                          
DMA1_Stream4_IRQHandler                                          
DMA1_Stream5_IRQHandler                                          
DMA1_Stream6_IRQHandler                                          
ADC_IRQHandler                                         
CAN1_TX_IRQHandler                                                            
CAN1_RX0_IRQHandler                                                          
CAN1_RX1_IRQHandler                                                           
CAN1_SCE_IRQHandler                                                           
EXTI9_5_IRQHandler                                                
TIM1_BRK_TIM9_IRQHandler                        
TIM1_UP_TIM10_IRQHandler                      
TIM1_TRG_COM_TIM11_IRQHandler  
TIM1_CC_IRQHandler                                               
TIM2_IRQHandler                                                           
TIM3_IRQHandler                                                           
TIM4_IRQHandler                                                           
I2C1_EV_IRQHandler                                                         
I2C1_ER_IRQHandler                                                         
I2C2_EV_IRQHandler                                                        
I2C2_ER_IRQHandler                                                           
SPI1_IRQHandler                                                          
SPI2_IRQHandler                                                           
USART1_IRQHandler                                                       
USART2_IRQHandler                                                       
USART3_IRQHandler                                                      
EXTI15_10_IRQHandler                                            
RTC_Alarm_IRQHandler                            
OTG_FS_WKUP_IRQHandler                                
TIM8_BRK_TIM12_IRQHandler                      
TIM8_UP_TIM13_IRQHandler                       
TIM8_TRG_COM_TIM14_IRQHandler  
TIM8_CC_IRQHandler                                               
DMA1_Stream7_IRQHandler                                                 
FMC_IRQHandler                                                            
SDIO_IRQHandler                                                            
TIM5_IRQHandler                                                            
SPI3_IRQHandler                                                            
UART4_IRQHandler                                                          
UART5_IRQHandler                                                          
TIM6_DAC_IRQHandler                            
TIM7_IRQHandler                              
DMA2_Stream0_IRQHandler                                         
DMA2_Stream1_IRQHandler                                          
DMA2_Stream2_IRQHandler                                           
DMA2_Stream3_IRQHandler                                           
DMA2_Stream4_IRQHandler                                        
ETH_IRQHandler                                                         
ETH_WKUP_IRQHandler                                
CAN2_TX_IRQHandler                                                           
CAN2_RX0_IRQHandler                                                          
CAN2_RX1_IRQHandler                                                          
CAN2_SCE_IRQHandler                                                          
OTG_FS_IRQHandler                                                    
DMA2_Stream5_IRQHandler                                          
DMA2_Stream6_IRQHandler                                          
DMA2_Stream7_IRQHandler                                          
USART6_IRQHandler                                                        
I2C3_EV_IRQHandler                                                          
I2C3_ER_IRQHandler                                                          
OTG_HS_EP1_OUT_IRQHandler                           
OTG_HS_EP1_IN_IRQHandler                            
OTG_HS_WKUP_IRQHandler                                
OTG_HS_IRQHandler                                                   
DCMI_IRQHandler                                                                                                             
HASH_RNG_IRQHandler
FPU_IRQHandler  
           
                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END

```

1. 设置堆栈指针 SP=__initial_sp
2. 设置PC指针 = Reset_Handler
3. 设置中断向量表
4. 配置系统时钟
5. 配置外部SRAM/SDRAM用于程序变量等数据存储【可选】
6. 跳转到C库中的__main,最终调用用户程序的main()函数

Cortex-M 内核处理器复位后，处于线程模式，指令权限是特权级别（最高级别），堆栈设置为使用主堆栈 MSP

## 4.1 复位序列

硬件复位之后，CPU内的时序逻辑：（程序下载到内部flash，flash首地址为：0x0800 0000）

1. 将0x0800 0000位置存放的堆栈栈顶地址存放SP（MSP）中

2. 将0x0800 0004位置存放的向量地址装入PC程序计数器

   CPU从PC寄存器指向的物理地址取出第一条指令执行程序，Reset_Handler.

   复位中断服务程序会调用SystemInit()函数来配置系统时钟、配置FMC总线上的外部SRAM/SDRAM，然后跳转到 C 库中__main 函数。由 C 库中的__main 函数完成用户程序的初始化工作（比如：变量赋初值等），最后由__main 函数调用用户写的 main()函数开始执行 C 程序。

## 4.2 代码分析

1. **开辟栈(stack)空间，用于局部变量、函数调用、函数的参数**

   **EQU**：表示宏定义的伪指令，类似#define，。伪指令的意思是指这个“指令”并

   不会生成二进制程序代码，也不会引起变量空间分配。

   以字节为单位

   **ARER 伪指令表示下面将开始定义一个代码段或者数据段。此处是定义数据段**

   ARER 后面的关键字表示这个段的属性

   STACK ：表示这个段的名字，可以任意命名。

   NOINIT：表示此数据段不需要填入初始数据。

   READWRITE：表示此段可读可写。

   ALIGN=3 ：表示首地址按照 2 的 3 次方对齐，也就是按照 8 字节对齐(地址对 8 求余数等于 0)。

   开辟一段数据空间可读可写，段名 STACK，按照 8 字节对齐

   **SPACE 这行指令告诉汇编器给 STACK 段分配 多少字节的连续内存空间**

   **__initial_sp 紧接着 SPACE 语句放置，表示了栈顶地址。**

   __initial_sp 只是一个标号，标号主要用于表示一片内存空间的某个位置，等价于 C 语言中的“地址”概念。地址仅仅表示存储空间的一个位置，从 C 语言的角度来看，变量的地址，数组的地址或是函数的入口地址在本质上并无区别。

   ```c
   ; Amount of memory (in bytes) allocated for Stack
   ; Tailor this value to your application needs
   ; <h> Stack Configuration
   ;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
   ; </h>
   
   Stack_Size      EQU     0x00000400
   
                   AREA    STACK, NOINIT, READWRITE, ALIGN=3
   Stack_Mem       SPACE   Stack_Size
   __initial_sp
   ```

2. **开辟堆(heap)空间，主要用于动态内存分配**，也就是说用 malloc，calloc, realloc 等函数分配的变量空间是在堆上

   __heap_base 表示堆的开始地址。

   __heap_limit 表示堆的结束地址。

   ```c
   
   ; <h> Heap Configuration
   ;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
   ; </h>
   
   Heap_Size       EQU     0x00000200
   
                   AREA    HEAP, NOINIT, READWRITE, ALIGN=3
   __heap_base
   Heap_Mem        SPACE   Heap_Size
   __heap_limit
   ```

3. 

   **PRESERVE8** 指定当前文件保持堆栈八字节对齐。

   **THUMB** 表示后面的指令是 THUMB 指令集 ，CM4 采用的是 THUMB - 2 指令集。

   **AREA** 定义一块代码段，只读，段名字是 RESET。READONLY 表示只读，缺省就表示代码段了。

   **EXPORT** 语句将 3 个标号申明为可被外部引用， 主要提供给链接器用于连接库文件或其他文件

   ```c
                   PRESERVE8
                   THUMB
   
   
   ; Vector Table Mapped to Address 0 at Reset
                   AREA    RESET, DATA, READONLY
                   EXPORT  __Vectors
                   EXPORT  __Vectors_End
                   EXPORT  __Vectors_Size
   ```

4. **建立中断向量表**

   中断向量表定位在代码段的最前面。具体的物理地址由链接器的配置参数（IROM1 的地址）决定。如果程序在 Flash 运行，则中断向量表的起始地址是 0x08000000。

   **DCD 表示分配 1 个 4 字节的空间**。每行 DCD 都会生成一个 4 字节的二进制代码。中断向量表

   存放的实际上是中断服务程序的入口地址。当异常（也即是中断事件）发生时，CPU 的中断系统会将相

   应的入口地址赋值给 PC 程序计数器，之后就开始执行中断服务程序。

   ```c
   __Vectors       DCD     __initial_sp               ; Top of Stack
                   DCD     Reset_Handler              ; Reset Handler
                   DCD     NMI_Handler                ; NMI Handler
                   DCD     HardFault_Handler          ; Hard Fault Handler
                   DCD     MemManage_Handler          ; MPU Fault Handler
                   DCD     BusFault_Handler           ; Bus Fault Handler
                   DCD     UsageFault_Handler         ; Usage Fault Handler
                   DCD     0                          ; Reserved
                   DCD     0                          ; Reserved
                   DCD     0                          ; Reserved
                   DCD     0                          ; Reserved
                   DCD     SVC_Handler                ; SVCall Handler
                   DCD     DebugMon_Handler           ; Debug Monitor Handler
                   DCD     0                          ; Reserved
                   DCD     PendSV_Handler             ; PendSV Handler
                   DCD     SysTick_Handler            ; SysTick Handler
   
                   ; External Interrupts
                   DCD     WWDG_IRQHandler                   ; Window WatchDog           ...		                              
                   DCD     HASH_RNG_IRQHandler               ; Hash and Rng
                   DCD     FPU_IRQHandler                    ; FPU
                   
                                            
   __Vectors_End
   
   __Vectors_Size  EQU  __Vectors_End - __Vectors
   ```

5. **Reset_Handler**

   AREA 定义一块代码段，只读，段名字是 .text 。READONLY 表示只读。

   利用 **PROC、ENDP** 这一对伪指令把程序段分为若干个过程，使程序的结构加清晰

   **WEAK 声明**其他的同名标号优先于该标号被引用,就是说如果外面声明了的话会调用外面的。 这个声明很重要，它让我们可以在 C 文件中任意地方放置中断服务程序，只要保证 C 函数的名字和向量表中的名字一致即可。

   **IMPORT**：伪指令用于通知编译器要使用的标号在其他的源文件中定义。但要在当前源文件中引用，而且无论当前源文件是否引用该标号，该标号均会被加入到当前源文件的符号表中。

   **SystemInit** 函数在文件 system_stm32f4xx.c 里面，主要实现 RCC 相关寄存器复位和中断向量

   表位置设置。

   __main 标号表示 C/C++标准实时库函数里的一个初始化子程序__main 的入口地址。该程序

   的一个主要作用是初始化堆栈(跳转__user_initial_stackheap 标号进行初始化堆栈的，下面会讲到这个标

   号)，并初始化映像文件，最后跳转到 C 程序中的 main 函数。这就解释了为何所有的 C 程序必须有一个 main 函数作为程序的起点。因为这是由 C/C++标准实时库所规，并且不能更改。

   ```c
                   AREA    |.text|, CODE, READONLY
   
   ; Reset handler
   Reset_Handler    PROC
                    EXPORT  Reset_Handler             [WEAK]
           IMPORT  SystemInit
           IMPORT  __main
   
                    LDR     R0, =SystemInit
                    BLX     R0
                    LDR     R0, =__main
                    BX      R0
                    ENDP
   ```

6. **虚拟异常处理程序(可以修改的无限循环)**

   ​	死循环，用户可以在此实现自己的中断服务程序。不过很少在这里实现中断服务程序，一般多是在其它的 C 文件里面重新写一个同样名字的中断服务程序，因为这里是 WEEK 弱定义的。如果没有在其它文件中写中断服务器程序，且使能了此中断，进入到这里后，会让程序卡在这个地方。

   

   缺省中断服务程序

   死循环，如果用户使能中断服务程序，而没有在 C 文件里面写中断服务程序的话，都会进入到这里

   缺省中断服务程序结束

   ```c
   ; Dummy Exception Handlers (infinite loops which can be modified)
   
   NMI_Handler     PROC
                   EXPORT  NMI_Handler                [WEAK]
                   B       .
                   ENDP
   HardFault_Handler\
                   PROC
                   EXPORT  HardFault_Handler          [WEAK]
                   B       .
                   ENDP
   MemManage_Handler\
                   PROC
                   EXPORT  MemManage_Handler          [WEAK]
                   B       .
                   ENDP
   BusFault_Handler\
                   PROC
                   EXPORT  BusFault_Handler           [WEAK]
                   B       .
                   ENDP
   UsageFault_Handler\
                   PROC
                   EXPORT  UsageFault_Handler         [WEAK]
                   B       .
                   ENDP
   SVC_Handler     PROC
                   EXPORT  SVC_Handler                [WEAK]
                   B       .
                   ENDP
   DebugMon_Handler\
                   PROC
                   EXPORT  DebugMon_Handler           [WEAK]
                   B       .
                   ENDP
   PendSV_Handler  PROC
                   EXPORT  PendSV_Handler             [WEAK]
                   B       .
                   ENDP
   SysTick_Handler PROC
                   EXPORT  SysTick_Handler            [WEAK]
                   B       .
                   ENDP
   
   Default_Handler PROC
   
                   EXPORT  WWDG_IRQHandler                   [WEAK]                                        
                   EXPORT  PVD_IRQHandler                    [WEAK]                      
                   EXPORT  TAMP_STAMP_IRQHandler             [WEAK]         
                   EXPORT  RTC_WKUP_IRQHandler               [WEAK]                     
                   EXPORT  FLASH_IRQHandler                  [WEAK]                                         
                   EXPORT  RCC_IRQHandler                    [WEAK]                                            
                   EXPORT  EXTI0_IRQHandler                  [WEAK]                                            
                   EXPORT  EXTI1_IRQHandler                  [WEAK]                                             
                   EXPORT  EXTI2_IRQHandler                  [WEAK]                                            
                   EXPORT  EXTI3_IRQHandler                  [WEAK]                                           
                   EXPORT  EXTI4_IRQHandler                  [WEAK]                                            
                   EXPORT  DMA1_Stream0_IRQHandler           [WEAK]                                
                   EXPORT  DMA1_Stream1_IRQHandler           [WEAK]                                   
                   EXPORT  DMA1_Stream2_IRQHandler           [WEAK]                                   
                   EXPORT  DMA1_Stream3_IRQHandler           [WEAK]                                   
                   EXPORT  DMA1_Stream4_IRQHandler           [WEAK]                                   
                   EXPORT  DMA1_Stream5_IRQHandler           [WEAK]                                   
                   EXPORT  DMA1_Stream6_IRQHandler           [WEAK]                                   
                   EXPORT  ADC_IRQHandler                    [WEAK]                         
                   EXPORT  CAN1_TX_IRQHandler                [WEAK]                                                
                   EXPORT  CAN1_RX0_IRQHandler               [WEAK]                                               
                   EXPORT  CAN1_RX1_IRQHandler               [WEAK]                                                
                   EXPORT  CAN1_SCE_IRQHandler               [WEAK]                                                
                   EXPORT  EXTI9_5_IRQHandler                [WEAK]                                    
                   EXPORT  TIM1_BRK_TIM9_IRQHandler          [WEAK]                  
                   EXPORT  TIM1_UP_TIM10_IRQHandler          [WEAK]                
                   EXPORT  TIM1_TRG_COM_TIM11_IRQHandler     [WEAK] 
                   EXPORT  TIM1_CC_IRQHandler                [WEAK]                                   
                   EXPORT  TIM2_IRQHandler                   [WEAK]                                            
                   EXPORT  TIM3_IRQHandler                   [WEAK]                                            
                   EXPORT  TIM4_IRQHandler                   [WEAK]                                            
                   EXPORT  I2C1_EV_IRQHandler                [WEAK]                                             
                   EXPORT  I2C1_ER_IRQHandler                [WEAK]                                             
                   EXPORT  I2C2_EV_IRQHandler                [WEAK]                                            
                   EXPORT  I2C2_ER_IRQHandler                [WEAK]                                               
                   EXPORT  SPI1_IRQHandler                   [WEAK]                                           
                   EXPORT  SPI2_IRQHandler                   [WEAK]                                            
                   EXPORT  USART1_IRQHandler                 [WEAK]                                          
                   EXPORT  USART2_IRQHandler                 [WEAK]                                          
                   EXPORT  USART3_IRQHandler                 [WEAK]                                         
                   EXPORT  EXTI15_10_IRQHandler              [WEAK]                                  
                   EXPORT  RTC_Alarm_IRQHandler              [WEAK]                  
                   EXPORT  OTG_FS_WKUP_IRQHandler            [WEAK]                        
                   EXPORT  TIM8_BRK_TIM12_IRQHandler         [WEAK]                 
                   EXPORT  TIM8_UP_TIM13_IRQHandler          [WEAK]                 
                   EXPORT  TIM8_TRG_COM_TIM14_IRQHandler     [WEAK] 
                   EXPORT  TIM8_CC_IRQHandler                [WEAK]                                   
                   EXPORT  DMA1_Stream7_IRQHandler           [WEAK]                                          
                   EXPORT  FMC_IRQHandler                    [WEAK]                                             
                   EXPORT  SDIO_IRQHandler                   [WEAK]                                             
                   EXPORT  TIM5_IRQHandler                   [WEAK]                                             
                   EXPORT  SPI3_IRQHandler                   [WEAK]                                             
                   EXPORT  UART4_IRQHandler                  [WEAK]                                            
                   EXPORT  UART5_IRQHandler                  [WEAK]                                            
                   EXPORT  TIM6_DAC_IRQHandler               [WEAK]                   
                   EXPORT  TIM7_IRQHandler                   [WEAK]                    
                   EXPORT  DMA2_Stream0_IRQHandler           [WEAK]                                  
                   EXPORT  DMA2_Stream1_IRQHandler           [WEAK]                                   
                   EXPORT  DMA2_Stream2_IRQHandler           [WEAK]                                    
                   EXPORT  DMA2_Stream3_IRQHandler           [WEAK]                                    
                   EXPORT  DMA2_Stream4_IRQHandler           [WEAK]                                 
                   EXPORT  ETH_IRQHandler                    [WEAK]                                         
                   EXPORT  ETH_WKUP_IRQHandler               [WEAK]                     
                   EXPORT  CAN2_TX_IRQHandler                [WEAK]                                               
                   EXPORT  CAN2_RX0_IRQHandler               [WEAK]                                               
                   EXPORT  CAN2_RX1_IRQHandler               [WEAK]                                               
                   EXPORT  CAN2_SCE_IRQHandler               [WEAK]                                               
                   EXPORT  OTG_FS_IRQHandler                 [WEAK]                                       
                   EXPORT  DMA2_Stream5_IRQHandler           [WEAK]                                   
                   EXPORT  DMA2_Stream6_IRQHandler           [WEAK]                                   
                   EXPORT  DMA2_Stream7_IRQHandler           [WEAK]                                   
                   EXPORT  USART6_IRQHandler                 [WEAK]                                           
                   EXPORT  I2C3_EV_IRQHandler                [WEAK]                                              
                   EXPORT  I2C3_ER_IRQHandler                [WEAK]                                              
                   EXPORT  OTG_HS_EP1_OUT_IRQHandler         [WEAK]                      
                   EXPORT  OTG_HS_EP1_IN_IRQHandler          [WEAK]                      
                   EXPORT  OTG_HS_WKUP_IRQHandler            [WEAK]                        
                   EXPORT  OTG_HS_IRQHandler                 [WEAK]                                      
                   EXPORT  DCMI_IRQHandler                   [WEAK]                                                                                 
                   EXPORT  HASH_RNG_IRQHandler               [WEAK]
                   EXPORT  FPU_IRQHandler                    [WEAK]
                   
   WWDG_IRQHandler                                                       
   PVD_IRQHandler                                      
   TAMP_STAMP_IRQHandler                  
   RTC_WKUP_IRQHandler                                
   FLASH_IRQHandler                                                       
   RCC_IRQHandler                                                            
   EXTI0_IRQHandler                                                          
   EXTI1_IRQHandler                                                           
   EXTI2_IRQHandler                                                          
   EXTI3_IRQHandler                                                         
   EXTI4_IRQHandler                                                          
   DMA1_Stream0_IRQHandler                                       
   DMA1_Stream1_IRQHandler                                          
   DMA1_Stream2_IRQHandler                                          
   DMA1_Stream3_IRQHandler                                          
   DMA1_Stream4_IRQHandler                                          
   DMA1_Stream5_IRQHandler                                          
   DMA1_Stream6_IRQHandler                                          
   ADC_IRQHandler                                         
   CAN1_TX_IRQHandler                                                            
   CAN1_RX0_IRQHandler                                                          
   CAN1_RX1_IRQHandler                                                           
   CAN1_SCE_IRQHandler                                                           
   EXTI9_5_IRQHandler                                                
   TIM1_BRK_TIM9_IRQHandler                        
   TIM1_UP_TIM10_IRQHandler                      
   TIM1_TRG_COM_TIM11_IRQHandler  
   TIM1_CC_IRQHandler                                               
   TIM2_IRQHandler                                                           
   TIM3_IRQHandler                                                           
   TIM4_IRQHandler                                                           
   I2C1_EV_IRQHandler                                                         
   I2C1_ER_IRQHandler                                                         
   I2C2_EV_IRQHandler                                                        
   I2C2_ER_IRQHandler                                                           
   SPI1_IRQHandler                                                          
   SPI2_IRQHandler                                                           
   USART1_IRQHandler                                                       
   USART2_IRQHandler                                                       
   USART3_IRQHandler                                                      
   EXTI15_10_IRQHandler                                            
   RTC_Alarm_IRQHandler                            
   OTG_FS_WKUP_IRQHandler                                
   TIM8_BRK_TIM12_IRQHandler                      
   TIM8_UP_TIM13_IRQHandler                       
   TIM8_TRG_COM_TIM14_IRQHandler  
   TIM8_CC_IRQHandler                                               
   DMA1_Stream7_IRQHandler                                                 
   FMC_IRQHandler                                                            
   SDIO_IRQHandler                                                            
   TIM5_IRQHandler                                                            
   SPI3_IRQHandler                                                            
   UART4_IRQHandler                                                          
   UART5_IRQHandler                                                          
   TIM6_DAC_IRQHandler                            
   TIM7_IRQHandler                              
   DMA2_Stream0_IRQHandler                                         
   DMA2_Stream1_IRQHandler                                          
   DMA2_Stream2_IRQHandler                                           
   DMA2_Stream3_IRQHandler                                           
   DMA2_Stream4_IRQHandler                                        
   ETH_IRQHandler                                                         
   ETH_WKUP_IRQHandler                                
   CAN2_TX_IRQHandler                                                           
   CAN2_RX0_IRQHandler                                                          
   CAN2_RX1_IRQHandler                                                          
   CAN2_SCE_IRQHandler                                                          
   OTG_FS_IRQHandler                                                    
   DMA2_Stream5_IRQHandler                                          
   DMA2_Stream6_IRQHandler                                          
   DMA2_Stream7_IRQHandler                                          
   USART6_IRQHandler                                                        
   I2C3_EV_IRQHandler                                                          
   I2C3_ER_IRQHandler                                                          
   OTG_HS_EP1_OUT_IRQHandler                           
   OTG_HS_EP1_IN_IRQHandler                            
   OTG_HS_WKUP_IRQHandler                                
   OTG_HS_IRQHandler                                                   
   DCMI_IRQHandler                                                                                                             
   HASH_RNG_IRQHandler
   FPU_IRQHandler  
              
                   B       .
   
                   ENDP
   
                   ALIGN
   ```

7. 

   __user_initial_stackheap 将由__main 函数进行调用

   [MicroLib](https://www.keil.com/arm/microlib.asp):是 MDK 里面带的微库，针对嵌入式应用，MicroLIB 做了深度优化，比使用 C 标准库所需的 RAM 和 FLASH 空间都大大减小比如调用：<math.h>，<stdlib.h>，<stdio.h>，<string.h>

   ```c
   
   ;*******************************************************************************
   ; User Stack and Heap initialization
   ;*******************************************************************************
                    IF      :DEF:__MICROLIB
                   
                    EXPORT  __initial_sp
                    EXPORT  __heap_base
                    EXPORT  __heap_limit
                   
                    ELSE
                   
                    IMPORT  __use_two_region_memory
                    EXPORT  __user_initial_stackheap
                    
   __user_initial_stackheap
   
                    LDR     R0, =  Heap_Mem
                    LDR     R1, =(Stack_Mem + Stack_Size)
                    LDR     R2, = (Heap_Mem +  Heap_Size)
                    LDR     R3, = Stack_Mem
                    BX      LR
   
                    ALIGN
   
                    ENDIF
   
                    END
   
   ```

## 4.3 BOOT启动模式

| BOOT0 | BOOT1 | 启动模式         |
| ----- | ----- | ---------------- |
| 0     | x     | 从Flash启动      |
| 1     | 0     | 从系统存储器启动 |
| 1     | 1     | 从内嵌的SRAM启动 |

- 从 Flash 启动（正常运行时选择这种模式）。
-  从系统存储器启动（做 ISP 下载时用）。
- 从内嵌 SRAM 启动（调试用，一般很少使用）

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-152236.png)

- 如果仅作从 Flash 驱动，可以将 BOOT0 和 BOOT1 直接接地，不需要电阻。

- 从 SRAM 启动，BOOT1 固定取低电平，BOOT0 可以取高电平或者低电平。注意硬件上不支持从SRAM 启动。因为掉电后，SRAM 中的数据消失。

- 电阻 R55 和按键配合，实现高低电平的切换。

- 电阻 R46 起到 GPIO 复用的功能，系统上电复位后可以检测 BOOT1 引脚转态，开始工作后可以用来做 RS485 的发送使能控制。

- ISP（In-System Programming）

  在系统可编程，就是说在 PCB 板子上面直接烧录，不需要将单片机取下来用烧录器烧写。ISP 功

  能可以通过非常简单廉价的下载线直接在电路板上给单片机下载程序或者擦除程序，免去插来插

  去的麻烦。

# 5.电源/复位/系统时钟

## 5.1 电源

电源是系统稳定运行的根本，主要分为以下几个知识点，电源供电、供电监控、电源管理和低功耗。当前阶段主要了解电源供电和硬件上电时序。

### **5.1.1 电源供电**

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-153513.png)

这些常用标识的解释如下：

| 引脚名（标识） | 描述                                                         |
| -------------- | ------------------------------------------------------------ |
| VDD            | 用于 I/O 引脚和系统模拟部分供电，比如复位、电源管理、振荡器等。 |
| VDDA           | 用于 ADC、DAC、运放、比较器和电压基准供电，这部分供电是独立的。 |
| Vref+ Vref-    | 用于 ADC 和 DAC 的基准电压，当使能了 STM32F4 内部的电压基准，将使用内部基准供 VREF+，VREF-。如果没有使能的话，通过外置电压基准提供。 |
| Vbat           | 当 VDD 不供电的时候，由 VBAT 为备份域供电。                  |
| Vddldo         | 电压稳压器供电。                                             |
| Vcap           | 数字核心域供电。                                             |
| Vss            | 所有电源和模拟稳压器的地端。                                 |

### 5.1.2 **电源去耦电容的选择**

每个电源对 （VDD/VSS， VDDA/VSSA ...）必须使用下述的滤波陶瓷电容去耦。这些电容必须尽量靠近芯片引脚，以确保器件正常工作。不建议去掉滤波电容来降低 PCB 尺寸或成本，这可能导致器件工作不正常。 

## 5.2 硬件复位

所有数字计算机系统都是由某种形式的震荡时钟电路驱动的。这种电路被称为系统的“脉搏”，是系统正确运行的关键。如果振荡器失灵，系统将完全无法运行，如果振荡器运行不规律，系统执行的所有与时间有关的计算都会有误差。

所有微控制器的启动流程都不通用。由于硬件的复杂性，必须运行一段由厂家定义的短小的“复位程序”来使硬件处于一种正确的状态，然后再开始执行用户程序。运行这个复位程序需要时间并且要求微控制器的振荡器已经运行。

当系统由可靠的电源供电时，一旦通电，电源迅速地达到额定输出电压，一旦断电，电源迅速地下降到 0V，并且在接通的时候，电压不会降低。这时能够可靠地使用基于一个电容和一个电阻的低成本硬件复位。这种形式的复位电路称为阻容复位。

如果电源不够可靠，而涉及安全性，这种简单的阻容解决方案就不合适了。

### 5.2.1**上电复位和手动复位**

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-154642.png)

- STM32 这款 CPU 的复位引脚是低电平有效，即 NRST 为低电平时，CPU 处于复位状态。

- R84 和 C53 组成简单的 RC 复位电路。当系统上电瞬间，C114 电容两端电压可以认为是 0，CPU 处于复位状态。3.3V 电源通过 R84 给 C53 充电，当 C53 的电压升到 CPU 的高电平门槛电压时，CPU退出复位状态转入运行状态。

- 在设计电路时，需要选择适当的 R 值和 C 值，以保证 NRST 低电平持续时间满足 CPU 复位最小脉宽的要求。

- 当按下 S4 轻触开关时，C53 两端被短路接地，可实现手动复位 CPU。

  注，根据需要，大家也可以使用 STM32F407 NRST 引脚的内部上拉：

### 5.2.2 复位序列

### 5.2.3 软件复位

程序设计设置中还经常要用到软件复位，即调用一条函数就可以实现复位功能。此函数已经由 CMSIS 软件包中的 core_cm4.h 文件提供，函数如下：

```c
/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
__NO_RETURN __STATIC_INLINE void __NVIC_SystemReset(void)
{
  __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                       buffered write are completed before reset */
  SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                           (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                            SCB_AIRCR_SYSRESETREQ_Msk    );         /* Keep priority group unchanged */
  __DSB();                                                          /* Ensure completion of memory access */

  for(;;)                                                           /* wait until reset */
  {
    __NOP();
  }
}
```

软件复位反映到实际硬件上，就是给硬件复位部分发一个复位信号：

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-155443.png)

## 5.3 RCC时钟控制

- HSI (High-speed internal oscillator) ：

  HSI 是内部的高速 RC 振荡器，频率 16MHz，可被用于系统时钟。优势是低成本，无需外部时钟，快速启动（仅需几个微秒），缺点是精度差，即使经过校准。

- HSE (High-speed external oscillator): 

  HSE 是外部的高速振荡器，通过外接时钟源，有源或者无源晶振驱动，时钟范围 4-26MHz。优势是精度高，缺点是增加成本。

- LSE (Low-speed external oscillator) 

  LSE 是外部的低速振荡器，通过外接时钟源，有源或者无源晶振驱动，一般接 32.768KHz，主要用于RTC 实时时钟。

- LSI (Low-speed internal oscillator) 

  LSI 是内部的低速 RC 振荡器，频率约是 32KHz，主要用于独立看门狗和自动唤醒，也可以用于 RTC实时时钟。

[clock tree]

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-155951.png)

### 5.3.1 HSE和LSE硬件设计

- **HSE时钟**
  - 晶振和负载电容需要尽可能近地靠近 F4 的晶振引脚，以减小输出失真和启动稳定时间。负载电容值必须根据选定的晶振进行调节。
  - 对于 C46 和 C47，我们推荐使用高质量陶瓷电容，这种电容是设计用于需要高频率的场合，并且可以满足晶体或谐振器的需求。C46 和 C47 通常具有相同的值。
  - 这里再额外补充一个知识点，HSE 旁路时钟和外置晶振区别：当前 V5 板子是采用的外置晶振模式，高速外部 (HSE) 时钟可以使用一个 4 到 26MHz 的晶振 / 陶瓷谐振振荡器产生：而 bypass 旁路的意思就是不使用它，绕过它。具体到 HSE 旁路的话，用户直接提供 4-26MHz 的时钟源即可，可以使用有源晶振或者 FPGA 提供时钟等方式：
- **LSE 时钟**
  - STM32 的 LSE 晶振起振难（又称 RTC 起振）是老毛病了，选取晶振和配套电容比较讲究，最好按照 ST提供的厂家和配套电容选取

### 5.3.2 时钟配置

1. 在 stm32f4xx_hal_conf.h 文件配置 HSE_VALUE

   ```c
   #if !defined  (HSE_VALUE) 
     #define HSE_VALUE    (8000000U) /*!< Value of the External oscillator in Hz */
   #endif /* HSE_VALUE */
   ```

2. 系统上电后，在启动文件 startup_stm32f429xx.s 的复位中断服务程序里面会调用函数SystemInit

   ```c
   ; Reset handler
   Reset_Handler    PROC
                    EXPORT  Reset_Handler             [WEAK]
           IMPORT  SystemInit
           IMPORT  __main
   
                    LDR     R0, =SystemInit
                    BLX     R0
                    LDR     R0, =__main
                    BX      R0
                    ENDP
   ```

   以往 STM32F1 和 STM32F4 系列都会在函数 SystemInit 里面配置 PLL 锁相环，使用了 HAL 后，需要在 main 函数里面配置。当前 SystemInit 函数实现的功能如下：

   使能 FPU 单元

   复位 RCC 相关寄存器

   设置中断向量表的位置

   ```c
   /**
     * @brief  Setup the microcontroller system
     *         Initialize the FPU setting, vector table location and External memory 
     *         configuration.
     * @param  None
     * @retval None
     */
   void SystemInit(void)
   {
     /* FPU settings ------------------------------------------------------------*/
     #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
       SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
     #endif
     /* Reset the RCC clock configuration to the default reset state ------------*/
     /* Set HSION bit */
     RCC->CR |= (uint32_t)0x00000001;
   
     /* Reset CFGR register */
     RCC->CFGR = 0x00000000;
   
     /* Reset HSEON, CSSON and PLLON bits */
     RCC->CR &= (uint32_t)0xFEF6FFFF;
   
     /* Reset PLLCFGR register */
     RCC->PLLCFGR = 0x24003010;
   
     /* Reset HSEBYP bit */
     RCC->CR &= (uint32_t)0xFFFBFFFF;
   
     /* Disable all interrupts */
     RCC->CIR = 0x00000000;
   
   #if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
     SystemInit_ExtMemCtl(); 
   #endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */
   
     /* Configure the Vector Table location add offset address ------------------*/
   #ifdef VECT_TAB_SRAM
     SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
   #else
     SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
   #endif
   }
   ```

3. 在 main 函数的外设驱动初始化函数里面完成时钟初始化，主要是 PLL 锁相环，让芯片最终工作到 168MHz。

   ```c
   /*
   *********************************************************************************************************
   *	函 数 名: SystemClock_Config
   *	功能说明: 初始化系统时钟
   *            	System Clock source            = PLL (HSE)
   *            	SYSCLK(Hz)                     = 168000000 (CPU Clock)
   *            	HCLK = SYSCLK / 1              = 168000000 (AHB1Periph)
   *            	PCLK2 = HCLK / 2               = 84000000  (APB2Periph)
   *            	PCLK1 = HCLK / 4               = 42000000  (APB1Periph)
   *            	HSE Frequency(Hz)              = 25000000
   *           	PLL_M                          = 25
   *            	PLL_N                          = 336
   *            	PLL_P                          = 2
   *            	PLL_Q                          = 4
   *            	VDD(V)                         = 3.3
   *            	Flash Latency(WS)              = 5
   *	形    参: 无
   *	返 回 值: 无
   *********************************************************************************************************
   */
   static void SystemClock_Config(void)
   {
   	RCC_ClkInitTypeDef RCC_ClkInitStruct;
   	RCC_OscInitTypeDef RCC_OscInitStruct;
   
   	
   	/* 芯片内部的LDO稳压器输出的电压范围，选用的PWR_REGULATOR_VOLTAGE_SCALE1 */
   	__HAL_RCC_PWR_CLK_ENABLE();
   	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
   
   	/* 使能HSE，并选择HSE作为PLL时钟源 */
   	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   	RCC_OscInitStruct.PLL.PLLM = 25;
   	RCC_OscInitStruct.PLL.PLLN = 336;
   	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   	RCC_OscInitStruct.PLL.PLLQ = 4;q1
   	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   	{
           //Error_Handler(__FILE__, __LINE__);
   	}
   
   	/* 
          选择PLL的输出作为系统时钟
   		HCLK = SYSCLK / 1  (AHB1Periph)
   		PCLK2 = HCLK / 2   (APB2Periph)
   		PCLK1 = HCLK / 4   (APB1Periph)
       */
   	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
   								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
   	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
   
   	/* 此函数会更新SystemCoreClock，并重新配置HAL_InitTick */
   	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
   	{
           //Error_Handler(__FILE__, __LINE__);
   	}
   
       /* 使能SYS时钟和IO补偿 */
   	__HAL_RCC_SYSCFG_CLK_ENABLE() ;
   
   	HAL_EnableCompensationCell();
   }
   
   ```

   

# 6.GPIO基础

## 6.1 GPIO功能简介

- 输出状态：开漏/推挽 + 上拉/下拉电阻
- 输出数据状态寄存器(GPIOx_ODR)或外设(复用模式)输出数据
- GPIO速度等级 IO补偿
- 输入状态：浮空 上拉/下拉 模拟
- 输入寄存器(GPIOx_IDR)或者外设输入数据
- 寄存器GPIOx_BSRR实现对寄存器GPIOx_ODR的为操作
- 寄存器GPIOx_LCKR的锁机制，实现冻结IO口配置
- 每两个时钟周期可以翻转一次IO
- 引脚复用功能

## 6.2 GPIO模式分析

- 输入浮空
- 输入上拉
- 输入下拉
- 模拟功能
- 具有上拉或下拉功能的开漏输出
- 具有上拉或下拉功能的推挽输出
- 具有上拉或下拉功能的复用功能推挽
- 具有上拉或下拉功能的复用功能开漏

**由于上拉和下拉是可选配置，对应的 HAL 库配置使用下面 6 种就可以表示：**

- GPIO_MODE_INPUT 输入模式
- GPIO_MODE_OUTPUT_PP 推挽输出
- GPIO_MODE_OUTPUT_OD 开漏输出
- GPIO_MODE_AF_PP 复用推挽
- GPIO_MODE_AF_OD 复用开漏
- GPIO_MODE_ANALOG 模拟模式

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-165054.png)

### 6.2.1 推挽输出

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230207-104328.png)

[Output driver]

推挽电路是两个参数相同的三极管或 MOSFET，以推挽方式存在于电路中。 电路工作时，两只对称的开关管每次只有一个导通，导通损耗小、效率高。输出既可以向负载灌电流，也可以从负载抽取电流。推拉式输出级提高电路的负载能力。 相对于开漏输出模式，推挽输出最大优势是输出高电平时，上升时间快，电压驱动能力强。

### 6.2.2 开漏输出

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230207-104349.png) 

开漏端相当于 MOS 管的漏极（三极管的集电极），要得到高电平状态必须外接上拉电阻才行，因此输出高电平的驱动能力完全由外接上拉电阻决定，但是其输出低电平的驱动能力很强。开漏形式的电路有以下几个特点：

1. 输出高电平时利用外部电路的驱动能力，减少 IC 内部的驱动。
2. 开漏是用来连接不同电平的器件，匹配电平用的，因为开漏引脚不连接外部的上拉电阻时，只能输出低电平。如果需要同时具备输出高电平的功能，则需要接上拉电阻，很好的一个优点是通过改变上拉电源的电压，便可以改变传输电平。上拉电阻的阻值决定了逻辑电平转换的速度。阻值越大，速度越低，功耗越小。
3. 开漏输出提供了灵活的输出方式，但是也有其弱点，就是带来上升沿的延时。因为上升沿是通过外接上拉无源电阻对负载充电，所以当电阻选择小时延时就小，但功耗大；反之延时大功耗小。所以如果对延时有要求，则建议用下降沿输出。
4. 可以将多个开漏输出连接到一条线上。通过一只上拉电阻，在不增加任何器件的情况下，形成“与逻辑”关系，即“线与”。可以简单的理解为：在所有引脚连在一起时，外接一上拉电阻，如果有一个引脚输出为逻辑 0，相当于接地，与之并联的回路“相当于被一根导线短路”，所以外电路逻辑电平便为 0，只有都为高电平时，与的结果才为逻辑 1。

### 6.2.3 复用推挽和开漏

复用指GPIO切换到CPU内部设备，由内部设备直接驱动。特质一样

### 6.2.4 输入模式

通过上面的引脚结构图可以得到如下三种方式

- 浮空输入：CPU 内部的上拉电阻、下拉电阻均断开的输入模式。

- 下拉输入：CPU 内部的下拉电阻使能、上拉电阻断开的输入模式。

- 上拉输入：CPU 内部的上拉电阻使能、下拉电阻断开的输入模式。
- 模拟输入：ADC使用

## 6.3 GPIO的拉电流和灌电流负载能力

- **拉电流负载：**一种负载电流从驱动门流向外电路，称为拉电流负载。
- **灌电流负载：**负载电流从外电路流入驱动门，称为灌电流负载。

STM32F407 的 IO 驱动能力 数据手册

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230207-111316.png) 

## 6.4 IO补偿单元，用于高速

IO 补偿单元用于控制 I/O 通信压摆率（tfall / trise）以此来降低 I/O 噪声。当前 STM32F4 的速度等级可以配置为以下四种：

```
/** @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
#define  GPIO_SPEED_FREQ_LOW         0x00000000U  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM      0x00000001U  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH        0x00000002U  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define  GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */
/**
  * @}
  */
```

> 不同的速度等级支持的最大时钟速度可以看此贴：
>
> http://www.armbbs.cn/forum.php?mod=viewthread&tid=94429 。

## 6.5 GPIO兼容CMOS和TTL电平

> http://www.armbbs.cn/forum.php?mod=viewthread&tid=87676 。

## 6.6 不使用的引脚推进设置外模拟模式

主要从功耗和防干扰考虑。

- 所有用作带上拉电阻输入的 I/O 都会在引脚外部保持为低时产生电流消耗。此电流消耗的值可通过使用的静态特性中给出的上拉 / 下拉电阻值简单算出。

- 对于输出引脚，还必须考虑任何外部下拉电阻或外部负载以估计电流消耗。

- 若外部施加了中间电平，则额外的 I/O 电流消耗是因为配置为输入的 I/O。此电流消耗是由用于区分输入值的输入施密特触发器电路导致。除非应用需要此特定配置，否则可通过将这些 I/O 配置为模拟模式以避免此供电电流消耗。 ADC 输入引脚应配置为模拟输入就是这种情况。

- 任何浮空的输入引脚都可能由于外部电磁噪声，成为中间电平或意外切换。为防止浮空引脚相关的电

  流消耗，它们必须配置为模拟模式，或内部强制为确定的数字值。这可通过使用上拉 / 下拉电阻或

  将引脚配置为输出模式做到。

# 7.HAL库API

1. 系统上电复位,进入启动文件startup_stm32f407xx.s->执行中断复位程序
   1. 在复位中断程序中执行SystemInit-> system_stm32f4xx.c
   2. 调用编译器封装好的函数,比如MDK的启动文件是调用__main,最后调用main函数
2. main()
   1. HAL库初始化函数HAL_Init-> stm32f4xx_hal.c
   2. 系统时钟初始化->stm32f4xx_hal_rcc.c

**依次使用的库文件**

- startup_stm32f407xx.s
- system_stm32f4xx.c
- stm32f4xx_hal.c
- stm32f4xx_hal_cortex.c
- stm32f4xx_hal_rcc.c
- core_cm4.h

## 7.1 stm32f4xx_hal.c

- HAL 库中各个外设驱动里面的延迟实现是基于此文件提供的时间基准，而这个时间基准既可以使用滴答定时器实现也可以使用通用的定时器实现，默认情况下是用的滴答定时器。
- 函数 HAL_Init 里面会调用时间基准初始化函数 HAL_InitTick，而调用函数 HAL_RCC_ClockConfig也会调用时间基准初始化函数 HAL_InitTick。
- 如果在中断服务程序里面调用延迟函数 HAL_Delay 要特别注意，因为这个函数的时间基准是基于滴答定时器或者其他通用定时器实现，实现方式是滴答定时器或者其他通用定时器里面做了个变量计数。如此一来，结果是显而易见的，如果其他中断服务程序调用了此函数，且中断优先级高于滴答定时器，会导致滴答定时器中断服务程序一直得不到执行，从而卡死在里面。所以滴答定时器的中断优先级一定要比它们高。

### 7.2.1 HAL_Init()

```c
/**
  * @brief  This function is used to initialize the HAL Library; it must be the first 
  *         instruction to be executed in the main program (before to call any other
  *         HAL function), it performs the following:
  *           Configure the Flash prefetch, instruction and Data caches.
  *           Configures the SysTick to generate an interrupt each 1 millisecond,
  *           which is clocked by the HSI (at this stage, the clock is not yet
  *           configured and thus the system is running from the internal HSI at 16 MHz).
  *           Set NVIC Group Priority to 4.
  *           Calls the HAL_MspInit() callback function defined in user file 
  *           "stm32f4xx_hal_msp.c" to do the global low level hardware initialization 
  *            
  * @note   SysTick is used as time base for the HAL_Delay() function, the application
  *         need to ensure that the SysTick time base is always set to 1 millisecond
  *         to have correct HAL operation.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_Init(void)
{
  /* Configure Flash prefetch, Instruction cache, Data cache */ 
#if (INSTRUCTION_CACHE_ENABLE != 0U)
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif /* INSTRUCTION_CACHE_ENABLE */

#if (DATA_CACHE_ENABLE != 0U)
  __HAL_FLASH_DATA_CACHE_ENABLE();
#endif /* DATA_CACHE_ENABLE */

#if (PREFETCH_ENABLE != 0U)
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  HAL_MspInit();

  /* Return function status */
  return HAL_OK;
}
```

- __HAL_FLASH_INSTRUCTION_CACHE_ENABLE
- __HAL_FLASH_DATA_CACHE_ENABLE
- __HAL_FLASH_PREFETCH_BUFFER_ENABLE
- NVIC_PRIORITYGROUP_4
- 滴答定时器的每 1ms 中断一次
- HAL 库不像之前的标准库，在系统启动函数 SystemInit 里面做了 RCC 初始化，HAL 库是没有做的，所以进入到 main 函数后，系统还在用内部高速时钟 HSI，对于 F4 来说，HSI 主频是 16MHz。
- 函数 HAL_Init 里面调用的 HAL_MspInit 一般在文件 stm32f4xx_hal_msp.c 里面做具体实现，主要用于底层初始化。当前此函数也在文件 stm32f4xx_hal.c 里面，只是做了弱定义。
- 返回值，返回 HAL_ERROR 表示参数错误，HAL_OK 表示发送成功，HAL_BUSY 表示忙，正在使用中。

**注意事项:**

- 必须在 main 函数里面优先调用此函数。
- 用户务必保证每 1ms 一次滴答中断。

### 7.2.2 HAL_DeInit()

```c
/**
  * @brief  This function de-Initializes common part of the HAL and stops the systick.
  *         This function is optional.   
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DeInit(void)
{
  /* Reset of all peripherals */
  __HAL_RCC_APB1_FORCE_RESET();
  __HAL_RCC_APB1_RELEASE_RESET();

  __HAL_RCC_APB2_FORCE_RESET();
  __HAL_RCC_APB2_RELEASE_RESET();

  __HAL_RCC_AHB1_FORCE_RESET();
  __HAL_RCC_AHB1_RELEASE_RESET();

  __HAL_RCC_AHB2_FORCE_RESET();
  __HAL_RCC_AHB2_RELEASE_RESET();

  __HAL_RCC_AHB3_FORCE_RESET();
  __HAL_RCC_AHB3_RELEASE_RESET();

  /* De-Init the low level hardware */
  HAL_MspDeInit();
    
  /* Return function status */
  return HAL_OK;
}
```

此函数用于复位 HAL 库和滴答时钟

- 复位了 APB1,2 的时钟以及 AHB1,2,3 的时钟
- 函数 HAL_DeInit 里面调用的 HAL_MspDeInit 一般在文件 stm32f4xx_hal_msp.c 里面做具体实现，主要用于底层初始化，跟函数 HAL_Init 里面调用的 HAL_MspInit 是一对。当前此函数也在文件stm32f4xx_hal.c 里面，只是做了弱定义。

### 7.2.3 HAL_InitTick()

```c
/**
  * @brief This function configures the source of the time base.
  *        The time source is configured  to have 1ms time base with a dedicated 
  *        Tick interrupt priority.
  * @note This function is called  automatically at the beginning of program after
  *       reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig().
  * @note In the default implementation, SysTick timer is the source of time base. 
  *       It is used to generate interrupts at regular time intervals. 
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process, 
  *       The SysTick interrupt must have higher priority (numerically lower)
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority Tick interrupt priority.
  * @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Return function status */
  return HAL_OK;
}
```

此函数用于初始化滴答时钟

- 此函数有个前缀__weak ，表示弱定义，用户可以重定义
- 此函数用于初始化滴答时钟 1ms 中断一次，并且为滴答中断配置一个用户指定的优先级
- 此函数由 HAL_Init 调用，或者任何其它地方调用函数 HAL_RCC_ClockConfig 配置 RCC 的时候也会调用 HAL_InitTick
- 调用基于此函数实现的 HAL_Delay 要特别注意，因为这个函数的时间基准是基于滴答定时器或者其他通用定时器实现，实现方式是滴答定时器或者其他通用定时器里面做了个变量计数。如此一来，结果是显而易见的，如果其他中断服务程序调用了此函数，且中断优先级高于滴答定时器，会导致滴答定时器中断服务程序一直得不到执行，从而卡死在里面。所以滴答定时器的中断优先级一定要比它们高。
- TickPriority 用于设置滴答定时器优先级

### 7.2.4 Systick相关函数

```
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
```

### 7.2.5 低功耗状态下继续使用调试功能

如果希望在睡眠，停机和待机的低功耗模式下继续使用调试功能，调用下面的函数

```c
/**
  * @brief  Enable the Debug Module during SLEEP mode
  * @retval None
  */
void HAL_DBGMCU_EnableDBGSleepMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Disable the Debug Module during SLEEP mode
  * @retval None
  */
void HAL_DBGMCU_DisableDBGSleepMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Enable the Debug Module during STOP mode
  * @retval None
  */
void HAL_DBGMCU_EnableDBGStopMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Disable the Debug Module during STOP mode
  * @retval None
  */
void HAL_DBGMCU_DisableDBGStopMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Enable the Debug Module during STANDBY mode
  * @retval None
  */
void HAL_DBGMCU_EnableDBGStandbyMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
}

/**
  * @brief  Disable the Debug Module during STANDBY mode
  * @retval None
  */
void HAL_DBGMCU_DisableDBGStandbyMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
}

```

## 7.2 stm32f4xx_hal_rcc.c

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230206-155951.png) 

实现内部和外部时钟（HSE、HSI、LSE、LSI、PLL、CSS、MCO）以及总线时钟（SYSCLK、AHB1、AHB2、AHB3、APB1）的配置

- 系统上电复位后，通过内部高速时钟 HSI 运行（主频 16MHz），Flash 工作在 0 等待周期，所有外设除了 SRAM、Flash、JTAG 和 PWR，时钟都是关闭的。
  - AHB 和 APB 总线无分频，所有挂载这两类总线上的外设都是以 HSI 频率运行。
  - 所有的 GPIO 都是模拟模式，除了 JTAG 相关的几个引脚。
- 系统上电复位后，用户需要完成以下工作
  - 选择用于驱动系统时钟的时钟源
  - 配置系统时钟频率和 Flash 设置
  - 配置分频器
  - 使能外设时钟
  - 配置外设时钟源，部分外设的时钟可以不来自系统时钟（I2S, RTC, ADC, USB OTGFS/SDIO/RNG）。

**RCC局限性**

使能了外设时钟后，不能立即操作对应的寄存器，要加延迟。不同外设延迟不同：

- 如果是 AHB 的外设，使能了时钟后，需要等待 2 个 AHB 时钟周期才可以操作这个外设的寄存器。
- 如果是 APB 的外设，使能了时钟后，需要等待 2 个 APB 时钟周期才可以操作这个外设的寄存器。

当前 HAL 库的解决方案是在使能了外设时钟后，再搞一个读操作，算是当做延迟用。

```c
#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)
```

**时钟知识补充**

1. PCLK1,2和HCLK对应哪些时钟

   1. PCLK1,PCLK2对应APB总线APB1,APB2时钟

   2. HCLK对应AHB总线

      ![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/screenshot-20230207-141619.png) 

2. 内部和外部时钟配置

   1. HSI(high-speed internal)

      高速内部 RC 振荡器，可以直接或者通过 PLL 倍频后做系统时钟源。缺点是精度差些，即使经过校准。

   2. LSI (low-speed internal)

      低速内部时钟，主要用于独立看门狗和 RTC 的时钟源。

   3. HSE (high-speed external)

      高速外部晶振，可接 4 - 26M 的晶振，可以直接或者通过 PLL 倍频后做系统时钟源，也可以做 RTC的是时钟源。

   4. LSE (low-speed external)

      低速外部晶振，主要用于 RTC。

   5. CSS (Clock security system)

      时钟安全系统，一旦使能后，如果 HSE 启动失败（不管是直接作为系统时钟源还是通过 PLL 输出后做系统时钟源），系统时钟将切换到 HSI。如果使能了中断的话，将进入不可屏蔽中断 NMI。

   6. MCO1 (micro controller clock output)

      可以在 PA8 引脚输出 SYSCLK、PLLI2SCLK、HSE 和 PLLCLK

   7. MCO2 (micro controller clock output)

      可以在 PC9 引脚输出 LSE、HSE、HSI 和 PLLCLK。

   8. PLL 锁相环，时钟输入来自 HSI , HSE 或者 CSI

### 7.2.1 HAL_RCC_DeInit()

```c
/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE and PLL OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  * @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_RCC_DeInit(void)
{
  return HAL_OK;
}
```

文件 stm32f4xx_hal_rcc.c 里面的此函数是空的,具体实现放在了 stm32f4xx_hal_rcc_ex.c 里面的此函数里面

此函数用于 RCC 复位函数，主要实现如下功能

- HSI 打开作为系统时钟
- HSE 和 PLL 关闭
- AHB, APB1 和 APB2 总线无分频 
- CSS, MCO1 和 MCO2 关闭
- 所有中断关闭

此函数不会修改外设时钟，LSI、LSE 和 RTC 时钟。

### 7.2.2 HAL_RCC_OscConfig()

```c
/**
  * @brief  Initializes the RCC Oscillators according to the specified parameters in the
  *         RCC_OscInitTypeDef.
  * @param  RCC_OscInitStruct pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
  *         supported by this API. User should request a transition to LSE Off
  *         first and then LSE On or LSE Bypass.
  * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this API. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
  uint32_t tickstart, pll_config;

  /* Check Null pointer */
  if(RCC_OscInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RCC_OSCILLATORTYPE(RCC_OscInitStruct->OscillatorType));
  /*------------------------------- HSE Configuration ------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSE(RCC_OscInitStruct->HSEState));
    /* When the HSE is used as system clock or clock source for PLL in these cases HSE will not disabled */
    if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSE) ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)))
    {
      if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Set the new HSE configuration ---------------------------------------*/
      __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

      /* Check the HSE State */
      if((RCC_OscInitStruct->HSEState) != RCC_HSE_OFF)
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is bypassed or disabled */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- HSI Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSI(RCC_OscInitStruct->HSIState));
    assert_param(IS_RCC_CALIBRATION_VALUE(RCC_OscInitStruct->HSICalibrationValue));

    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI) ||\
      ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)))
    {
      /* When HSI is used as system clock it will not disabled */
      if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) && (RCC_OscInitStruct->HSIState != RCC_HSI_ON))
      {
        return HAL_ERROR;
      }
      /* Otherwise, just the calibration is allowed */
      else
      {
        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
    }
    else
    {
      /* Check the HSI State */
      if((RCC_OscInitStruct->HSIState)!= RCC_HSI_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI). */
        __HAL_RCC_HSI_ENABLE();

        /* Get Start Tick*/
        tickstart = HAL_GetTick();

        /* Wait till HSI is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Adjusts the Internal High Speed oscillator (HSI) calibration value. */
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI). */
        __HAL_RCC_HSI_DISABLE();

        /* Get Start Tick*/
        tickstart = HAL_GetTick();

        /* Wait till HSI is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*------------------------------ LSI Configuration -------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
  {
    /* Check the parameters */
    assert_param(IS_RCC_LSI(RCC_OscInitStruct->LSIState));

    /* Check the LSI State */
    if((RCC_OscInitStruct->LSIState)!= RCC_LSI_OFF)
    {
      /* Enable the Internal Low Speed oscillator (LSI). */
      __HAL_RCC_LSI_ENABLE();

      /* Get Start Tick*/
      tickstart = HAL_GetTick();

      /* Wait till LSI is ready */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Disable the Internal Low Speed oscillator (LSI). */
      __HAL_RCC_LSI_DISABLE();

      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSI is ready */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /*------------------------------ LSE Configuration -------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
  {
    FlagStatus       pwrclkchanged = RESET;

    /* Check the parameters */
    assert_param(IS_RCC_LSE(RCC_OscInitStruct->LSEState));

    /* Update LSE configuration in Backup Domain control register    */
    /* Requires to enable write access to Backup Domain of necessary */
    if(__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = HAL_GetTick();

      while(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Set the new LSE configuration -----------------------------------------*/
    __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
    /* Check the LSE State */
    if((RCC_OscInitStruct->LSEState) != RCC_LSE_OFF)
    {
      /* Get Start Tick*/
      tickstart = HAL_GetTick();

      /* Wait till LSE is ready */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSE is ready */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Restore clock configuration if changed */
    if(pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }
  /*-------------------------------- PLL Configuration -----------------------*/
  /* Check the parameters */
  assert_param(IS_RCC_PLL(RCC_OscInitStruct->PLL.PLLState));
  if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
  {
    /* Check if the PLL is used as system clock or not */
    if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
    {
      if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
      {
        /* Check the parameters */
        assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
        assert_param(IS_RCC_PLLM_VALUE(RCC_OscInitStruct->PLL.PLLM));
        assert_param(IS_RCC_PLLN_VALUE(RCC_OscInitStruct->PLL.PLLN));
        assert_param(IS_RCC_PLLP_VALUE(RCC_OscInitStruct->PLL.PLLP));
        assert_param(IS_RCC_PLLQ_VALUE(RCC_OscInitStruct->PLL.PLLQ));

        /* Disable the main PLL. */
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Configure the main PLL clock source, multiplication and division factors. */
        WRITE_REG(RCC->PLLCFGR, (RCC_OscInitStruct->PLL.PLLSource                                            | \
                                 RCC_OscInitStruct->PLL.PLLM                                                 | \
                                 (RCC_OscInitStruct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)             | \
                                 (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) | \
                                 (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)));
        /* Enable the main PLL. */
        __HAL_RCC_PLL_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the main PLL. */
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
      /* Check if there is a request to disable the PLL used as System clock source */
      if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF)
      {
        return HAL_ERROR;
      }
      else
      {
        /* Do not return HAL_ERROR if request repeats the current configuration */
        pll_config = RCC->PLLCFGR;
#if defined (RCC_PLLCFGR_PLLR)
        if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (RCC_OscInitStruct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (RCC_OscInitStruct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLR) != (RCC_OscInitStruct->PLL.PLLR << RCC_PLLCFGR_PLLR_Pos)))
#else
        if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (RCC_OscInitStruct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (RCC_OscInitStruct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos) ||
            (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)))
#endif
        {
          return HAL_ERROR;
        }
      }
    }
  }
  return HAL_OK;
}
```

**配置HSE,HSI,LSI,LSE,PLL**

函数的形参是 RCC_OscInitTypeDef 类型结构体变量

```c
/**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */

  RCC_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
}RCC_OscInitTypeDef;

```

**注意事项：**

1. LSE Bypass 切换到 LSE On 或者 LSE On 切换到 LSE Bypass 都不支持，用需要先关闭 LSE，然后才可以切换到 LSE Bypass 或者 LSE On
2. HSE Bypass 切换到 HSE On 或者 LSE On 切换到 LSE Bypass 都不支持，用需要先关闭 HSE，然后才可以切换到 HSE Bypass 或者 HSE On

```
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* 使能HSE，并选择HSE作为PLL时钟源 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
        //Error_Handler(__FILE__, __LINE__);
	}

```

### 7.2.3 HAL_RCC_ClockConfig()

```c
/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCC_ClkInitStruct pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC peripheral.
  * @param  FLatency FLASH Latency, this parameter depend on device selected
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCC_GetHCLKFreq() function called within this function
  *
  * @note   The HSI is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @note   Depending on the device voltage range, the software has to set correctly
  *         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
  *         (for more details refer to section above "Initialization/de-initialization functions")
  * @retval None
  */
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;

  /* Check Null pointer */
  if(RCC_ClkInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RCC_CLOCKTYPE(RCC_ClkInitStruct->ClockType));
  assert_param(IS_FLASH_LATENCY(FLatency));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
    must be correctly programmed according to the frequency of the CPU clock
    (HCLK) and the supply voltage of the device. */

  /* Increasing the number of wait states because of higher CPU frequency */
  if(FLatency > __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if(__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      return HAL_ERROR;
    }
  }

  /*-------------------------- HCLK Configuration --------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
  {
    /* Set the highest APBx dividers in order to ensure that we do not go through
       a non-spec phase whatever we decrease or increase HCLK. */
    if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
    }

    if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
    }

    assert_param(IS_RCC_HCLK(RCC_ClkInitStruct->AHBCLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
  {
    assert_param(IS_RCC_SYSCLKSOURCE(RCC_ClkInitStruct->SYSCLKSource));

    /* HSE is selected as System Clock Source */
    if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    else if((RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)   ||
            (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLRCLK))
    {
      /* Check the PLL ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else
    {
      /* Check the HSI ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }

    __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick */
    tickstart = HAL_GetTick();

    while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos))
    {
      if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  /* Decreasing the number of wait states because of lower CPU frequency */
  if(FLatency < __HAL_FLASH_GET_LATENCY())
  {
     /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if(__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      return HAL_ERROR;
    }
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB1CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB2CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3U));
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_Pos];

  /* Configure the source of time base considering new system clocks settings */
  HAL_InitTick (uwTickPrio);

  return HAL_OK;
}
```

**配置了 HCLK、SYSCLK、PLCK1 和 PLCK2**

1. RCC_ClkInitTypeDef 类型的结构体变量
2. Flash 的延迟设置

**注意事项**

1. 此函数会更新全局变量 SystemCoreClock 的主频值，并且会再次调用函数 HAL_InitTick 更新系统滴答时钟，这点要特别注意
2. 系统上电复位或者从停机、待机模式唤醒后，使用的是 HSI 作为系统时钟。以防使用 HSE 直接或者通过 PLL 输出后做系统时钟时失败（如果使能了 CSS）
3. 目标时钟就绪后才可以从当前的时钟源往这个目标时钟源切换，如果目标时钟源没有就绪，就会等待直到时钟源就绪才可以切换
4. 根据设备的供电范围，必须正确设置 D1CPRE[3:0]位的范围，防止超过允许的最大频率

```c
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/* 
       选择PLL的输出作为系统时钟
		HCLK = SYSCLK / 1  (AHB1Periph)
		PCLK2 = HCLK / 2   (APB2Periph)
		PCLK1 = HCLK / 4   (APB1Periph)
    */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	/* 此函数会更新SystemCoreClock，并重新配置HAL_InitTick */
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
        Error_Handler(__FILE__, __LINE__);
	}

```

### 7.2.4 HAL_RCC_MCOConfig()

```c
/**
  * @brief  Selects the clock source to output on MCO1 pin(PA8) or on MCO2 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCC_MCOx specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1: Clock source to output on MCO1 pin(PA8).
  *            @arg RCC_MCO2: Clock source to output on MCO2 pin(PC9).
  * @param  RCC_MCOSource specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  *            @arg RCC_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source, available for all STM32F4 devices except STM32F410xx
  *            @arg RCC_MCO2SOURCE_I2SCLK: I2SCLK clock selected as MCO2 source, available only for STM32F410Rx devices
  *            @arg RCC_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  RCC_MCODiv specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  * @note  For STM32F410Rx devices to output I2SCLK clock on MCO2 you should have
  *        at last one of the SPI clocks enabled (SPI1, SPI2 or SPI5).
  * @retval None
  */
void HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Check the parameters */
  assert_param(IS_RCC_MCO(RCC_MCOx));
  assert_param(IS_RCC_MCODIV(RCC_MCODiv));
  /* RCC_MCO1 */
  if(RCC_MCOx == RCC_MCO1)
  {
    assert_param(IS_RCC_MCO1SOURCE(RCC_MCOSource));

    /* MCO1 Clock Enable */
    __MCO1_CLK_ENABLE();

    /* Configure the MCO1 pin in alternate function mode */
    GPIO_InitStruct.Pin = MCO1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

    /* Mask MCO1 and MCO1PRE[2:0] bits then Select MCO1 clock source and prescaler */
    MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE), (RCC_MCOSource | RCC_MCODiv));

   /* This RCC MCO1 enable feature is available only on STM32F410xx devices */
#if defined(RCC_CFGR_MCO1EN)
    __HAL_RCC_MCO1_ENABLE();
#endif /* RCC_CFGR_MCO1EN */
  }
#if defined(RCC_CFGR_MCO2)
  else
  {
    assert_param(IS_RCC_MCO2SOURCE(RCC_MCOSource));

    /* MCO2 Clock Enable */
    __MCO2_CLK_ENABLE();

    /* Configure the MCO2 pin in alternate function mode */
    GPIO_InitStruct.Pin = MCO2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(MCO2_GPIO_PORT, &GPIO_InitStruct);

    /* Mask MCO2 and MCO2PRE[2:0] bits then Select MCO2 clock source and prescaler */
    MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE), (RCC_MCOSource | (RCC_MCODiv << 3U)));

   /* This RCC MCO2 enable feature is available only on STM32F410Rx devices */
#if defined(RCC_CFGR_MCO2EN)
    __HAL_RCC_MCO2_ENABLE();
#endif /* RCC_CFGR_MCO2EN */
  }
#endif /* RCC_CFGR_MCO2 */
}
```

此函数的作用是配置 MCO1（PA8 引脚）和 MCO2（PC9 引脚）的时钟输出以及选择的时钟源:

- 参数1:选择输出的引脚，可选择 RCC_MCO1（PA8 引脚）或者 RCC_MCO2（PC9 引脚）。

- 参数2:选择输出的时钟源，MCO1 可选择的时钟源如下

  - RCC_MCO1SOURCE_HSI
  - RCC_MCO1SOURCE_LSE
  - RCC_MCO1SOURCE_HSE
  - RCC_MCO1SOURCE_PLLCLK

  MCO2:

  - RCC_MCO2SOURCE_SYSCLK
  - RCC_MCO2SOURCE_PLLI2SCLK
  - RCC_MCO2SOURCE_I2SCLK
  - RCC_MCO2SOURCE_HSE
  - RCC_MCO2SOURCE_PLLCLK

- 参数3:设置输出分频，范围从 RCC_MCODIV_1 到 RCC_MCODIV_4。

### 7.2.5 RCC相关函数

```c
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);
```

## 7.3 stm32f4xx_hal_cortex.c

这个库文件主要功能是 NVIC，MPU 和 Systick 的配置。此文件有个臃肿的地方，里面的 API 其实就是将 ARM 的 CMSIS 库各种 API 重新封装了一遍。这么做的好处是保证了 HAL 的 API 都是以字母 HAL开头。

```c
/* Initialization and de-initialization functions *****************************/
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);

uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);

void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);
```

# 8. GPIO HAL库API

> 很多时候，我们会直接调用 GPIO 的寄存器进行配置，而不使用 HAL 进行调用，以提高执行效率，特别是中断里面执行时。

## 8.1 stm32f4xx_hal_gpio.c

- 系统上电后，引脚默认状态是模拟模式。
- 所有的引脚有弱上拉和弱下拉电阻，阻值范围 30-50KΩ。其中配置为模拟模式时，上拉和下拉被硬件禁止，其它的输入、输出和复用都可以配置上拉和下拉。
- 在输出或者复用模式，每个引脚可以配置成推挽或者开漏，且有 GPIO 速度等级可配置。另外注意，不同的供电范围，实际速度等级是有些区别的
- 每个 GPIO 都可以配置成外部中断/事件模式，但要特别注意，引脚要配置成输入模式，在芯片的内部有个多路选择器，选择引脚与 16 个外部中断/事件 EXTI0 - EXTI15 中的那个导通。这就决定了，每个外部中断/事件只能与一个引脚导通，如果用户配置了多个引脚 PA0，PB0，PC0 等，那么只有一个能够与 EXTI0 导通。

### 8.1.1 HAL_GPIO_Init()

```c
/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position;
  uint32_t ioposition = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));

  /* Configure the port pins */
  for(position = 0U; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = 0x01U << position;
    /* Get the current IO position */
    iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Output or Alternate function mode selection */
      if(((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) || \
          (GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {
        /* Check the Speed parameter */
        assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR; 
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
        temp |= (GPIO_Init->Speed << (position * 2U));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OTYPER = temp;
       }

      if((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
      {
        /* Check the parameters */
        assert_param(IS_GPIO_PULL(GPIO_Init->Pull));
        
        /* Activate the Pull-up or Pull down resistor for the current IO */
        temp = GPIOx->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
        temp |= ((GPIO_Init->Pull) << (position * 2U));
        GPIOx->PUPDR = temp;
      }

      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {
        /* Check the Alternate function parameter */
        assert_param(IS_GPIO_AF(GPIO_Init->Alternate));
        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3U];
        temp &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & 0x07U) * 4U));
        GPIOx->AFR[position >> 3U] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODER = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if((GPIO_Init->Mode & EXTI_MODE) != 0x00U)
      {
        /* Enable SYSCFG Clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[position >> 2U];
        temp &= ~(0x0FU << (4U * (position & 0x03U)));
        temp |= ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U)));
        SYSCFG->EXTICR[position >> 2U] = temp;

        /* Clear Rising Falling edge configuration */
        temp = EXTI->RTSR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & TRIGGER_RISING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->RTSR = temp;

        temp = EXTI->FTSR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & TRIGGER_FALLING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->FTSR = temp;

        temp = EXTI->EMR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & EXTI_EVT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->EMR = temp;

        /* Clear EXTI line configuration */
        temp = EXTI->IMR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & EXTI_IT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->IMR = temp;
      }
    }
  }
}
```

- GPIO 功能配置
- 设置 EXTI 功能

```
/** 
  * @brief GPIO Init structure definition  
  */ 
typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins. 
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDef;
```

**注意事项:**

- HAL_GPIO_Init 对引脚的初始化是把同组 16 个引脚 for 循环检测了一遍，效率稍低

  程序运行期间的引脚状态切换，最好采用下面的方式或者直接寄存器操作

  ```c
  GPIO_InitStruct.Pin = GPIO_PIN_0 |GPIO_PIN_1 | GPIO_PIN_2 ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); //这里会执行 16 次 for 查询
  ```

### 8.1.2 HAL_GPIO_DeInit()

```c
/**
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
  uint32_t position;
  uint32_t ioposition = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t tmp = 0x00U;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  
  /* Configure the port pins */
  for(position = 0U; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = 0x01U << position;
    /* Get the current IO position */
    iocurrent = (GPIO_Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*------------------------- EXTI Mode Configuration --------------------*/
      tmp = SYSCFG->EXTICR[position >> 2U];
      tmp &= (0x0FU << (4U * (position & 0x03U)));
      if(tmp == ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U))))
      {
        /* Clear EXTI line configuration */
        EXTI->IMR &= ~((uint32_t)iocurrent);
        EXTI->EMR &= ~((uint32_t)iocurrent);
        
        /* Clear Rising Falling edge configuration */
        EXTI->FTSR &= ~((uint32_t)iocurrent);
        EXTI->RTSR &= ~((uint32_t)iocurrent);

        /* Configure the External Interrupt or event for the current IO */
        tmp = 0x0FU << (4U * (position & 0x03U));
        SYSCFG->EXTICR[position >> 2U] &= ~tmp;
      }

      /*------------------------- GPIO Mode Configuration --------------------*/
      /* Configure IO Direction in Input Floating Mode */
      GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (position * 2U));

      /* Configure the default Alternate Function in current IO */
      GPIOx->AFR[position >> 3U] &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;

      /* Deactivate the Pull-up and Pull-down resistor for the current IO */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));

      /* Configure the default value IO Output Type */
      GPIOx->OTYPER  &= ~(GPIO_OTYPER_OT_0 << position) ;

      /* Configure the default value for IO Speed */
      GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
    }
  }
}
```

### 8.1.3 HAL_GPIO_ReadPin()

```c
/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to read.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The input port pin value.
  */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}

```

### 8.1.4 HAL_GPIO_WritePin()

```c
/**
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if(PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
  }
}
```

### 8.1.5 HAL_GPIO_TogglePin()

```c
/**
  * @brief  Toggles the specified GPIO pins.
  * @param  GPIOx Where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
  *                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
  * @param  GPIO_Pin Specifies the pins to be toggled.
  * @retval None
  */
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint32_t odr;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  /* get current Output Data Register value */
  odr = GPIOx->ODR;

  /* Set selected pins that were at low level, and reset ones that were high */
  GPIOx->BSRR = ((odr & GPIO_Pin) << GPIO_NUMBER) | (~odr & GPIO_Pin);
}
```

### 8.1.6 HAL_GPIO_LockPin()

```c
/**
  * @brief  Locks GPIO Pins configuration registers.
  * @note   The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
  * @note   The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for STM32F4 family
  * @param  GPIO_Pin specifies the port bit to be locked.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15).
  * @retval None
  */
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  __IO uint32_t tmp = GPIO_LCKR_LCKK;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  /* Apply lock key write sequence */
  tmp |= GPIO_Pin;
  /* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
  GPIOx->LCKR = tmp;
  /* Reset LCKx bit(s): LCKK='0' + LCK[15-0] */
  GPIOx->LCKR = GPIO_Pin;
  /* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
  GPIOx->LCKR = tmp;
  /* Read LCKR register. This read is mandatory to complete key lock sequence */
  tmp = GPIOx->LCKR;

  /* Read again in order to confirm lock is active */
 if((GPIOx->LCKR & GPIO_LCKR_LCKK) != RESET)
  {
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}
```

此函数用于锁住 GPIO 引脚所涉及到的寄存器，这些寄存器包括 GPIOx_MODER，GPIOx_OTYPER，GPIOx_OSPEEDR，GPIOx_PUPDR，GPIOx_AFRL 和 GPIOx_AFRH。

### 8.1.7 HAL_GPIO_EXTI_IRQHandler

```c
/**
  * @brief  This function handles EXTI interrupt request.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    HAL_GPIO_EXTI_Callback(GPIO_Pin);
  }
}
```

### 8.1.8 HAL_GPIO_EXTI_Callback()

```c
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
```

## 8.2 使用步骤

1. 使能 GPIO 所在总线的 AHB 时钟，__HAL_RCC_GPIOx_CLK_ENABLE()
2. 通过函数 HAL_GPIO_Init()配置 GPIO
   1. 结构体 GPIO_InitTypeDef 的成员 Mode 配置输入、输出、模拟等模式
   2. 结构体 GPIO_InitTypeDef 的成员 Pull 配置上拉、下拉电阻
   3. 结构体 GPIO_InitTypeDef 的成员 Speed 配置 GPIO 速度等级
   4. 选择了复用模式，那么就需要配置结构体 GPIO_InitTypeDef 的成员 Alternate
   5. 引脚功能用于 ADC、DAC 的话，需要配置引脚为模拟模式
   6. 用于外部中断/事件，结构体 GPIO_InitTypeDef 的成员 Mode 可以配置相应模式，相应的上升沿、下降沿或者双沿触发也可以选择
3. 配置了外部中断/事件，可以通过函数 HAL_NVIC_SetPriority 设置优先级，然后调用函数 HAL_NVIC_EnableIRQ 使能此中断
4. 输入模式读取引脚状态可以使用函数 HAL_GPIO_ReadPin
5. 输出模式设置引脚状态可以调用函数 HAL_GPIO_WritePin()和 HAL_GPIO_TogglePin

**注意事项**

- 系统上电复位后，GPIO 默认是模拟模式，除了 JTAG 相关引脚
- 关闭 LSE 的话，用到的两个引脚 OSC32_IN 和 OSC32_OUT（分别是 PC14，PC15）可以用在通用IO，如果开启了，就不能再做 GPIO
- 关闭 HSE 的话，用到的两个引脚 OSC_IN 和 OSC_OUT（分别是 PH0，PH1）可以用在通用 IO，如果开启了，就不能再做 GPIO

# 9.驱动LED

## 9.1 灌电流驱动方式

拉电流和灌电流时，STM32F407 单个引脚最大可可达 25mA

## 9.2 LED的压降和驱动电流

- 直插超亮发光二极管压降

  - 红色发光二极管的压降为 2.0V-2.2V
  -  黄色发光二极管的压降为 1.8V-2.0V
  - 绿色发光二极管的压降为 3.0V-3.2V
  - 正常发光时的额定电流约为 20mA

- 贴片 LED 压降

  - 红色的压降为 1.82-1.88V，电流 5-8mA
  - 绿色的压降为 1.75-1.82V，电流 3-5mA
  - 橙色的压降为 1.7-1.8V，电流 3-5mA
  - 蓝色的压降为 3.1-3.3V，电流 8-10mA
  - 白色的压降为 3-3.2V，电流 10-15mA

  正负极区分:贴片 LED，会发现一端有绿点，有绿点的这端是负极，而另一端就是正极

  

