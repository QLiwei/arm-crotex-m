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