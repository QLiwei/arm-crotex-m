/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_init.h"
#include "device_init.h"
#include "protocol.h"

/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define LOG_TAG "MAIN"
#define DBG_ENABLE
#define DBG_SECTION_NAME LOG_TAG
#define DBG_LEVEL        DBG_LOG
// #define DBG_COLOR
#include "utils_dbg.h"

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
