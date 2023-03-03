/**
 ******************************************************************************
 * @file           : bsp_init.h
 * @brief          : Header for bsp_init.c file.
 *                   This file contains the common defines of the bsp layer
 *                   initialize.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_INIT_H__
#define __BSP_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "systick.h"
#include "debug_usart.h"
#include "bsp_adc.h"
#include "bsp_timer.h"
#include "bsp_encoder.h"
#include "bsp_basic_timer.h"

/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void bsp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_INIT_H__ */
