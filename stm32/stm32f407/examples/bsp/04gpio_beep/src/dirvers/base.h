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

#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

void system_clock_config(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_H__ */
