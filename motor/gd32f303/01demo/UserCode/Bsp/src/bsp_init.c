/**
 ******************************************************************************
 * @file           : bsp_init.c
 * @brief          : bsp layer initialize program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022
 * All rights reserved.
 *
 ******************************************************************************
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-08     vector       the first version
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_init.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/**
 * @brief bsp layer initialize
 *
 * @param[in] none
 * @param[out] none
 * @return none
 */
void bsp_init(void) {
    /*systick config */
    systick_config();
    /* debug uart init */
    board_console_init();
    /* dma adc init */
    bsp_adc_init();
    /* timer pwm init */
    bsp_timer_init();
    /* timer encoder init */
    bsp_encoder_init();
    /* basic timer init */
    bsp_basic_timer_init();
}
