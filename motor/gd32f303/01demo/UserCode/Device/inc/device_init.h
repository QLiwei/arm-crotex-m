/**
 ******************************************************************************
 * @file           : device_init.h
 * @brief          : Header for device_init.c file.
 *                   This file contains the common defines of the device layer
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
#ifndef __DEVICE_INIT_H__
#define __DEVICE_INIT_H__

#include "device_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void device_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_INIT_H__ */
