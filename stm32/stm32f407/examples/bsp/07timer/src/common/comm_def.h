/**
 * @file comm_def.h
 * @brief Generic common definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __COMM_DEF_H__
#define __COMM_DEF_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "comm_errorno.h"
#include "comm_init.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GET_BIT
    #define GET_BIT(value, bit) ((value) & (1<<(bit)))
#endif /* GET_BIT */

#ifndef SET_BIT
    #define SET_BIT(value, bit) ((value) |= (1<<(bit)))
#endif /* SET_BIT */

#ifndef RESET_BIT
    #define RESET_BIT(value, bit) ((value) &= ~(1<<(bit)))
#endif /* RESET_BIT */

#ifndef XOR_BIT
    #define XOR_BIT(value, bit) ((value) ^= (1<<(bit)))
#endif /* XOR_BIT */

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(_x) (sizeof(_x) / sizeof(_x[0]))
#endif /* ARRAY_SIZE */

#ifndef MAX
    #define MAX(_x, _y) (((_x) > (_y)) ? (_x) : (_y))
#endif /* MAX */

#ifndef MIN
    #define MIN(_x, _y) (((_x) < (_y)) ? (_x) : (_y))
#endif /* MIN */

#ifdef __cplusplus
}
#endif

#endif /* __COMM_DEF_H__ */
