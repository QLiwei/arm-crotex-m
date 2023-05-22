/**
 * @file base.h
 * @brief base definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __BASE_H__
#define __BASE_H__

#include "stm32f4xx_hal.h"
#include "comm_def.h"

#ifdef __cplusplus
extern "C" {
#endif


void system_clock_config(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_H__ */
