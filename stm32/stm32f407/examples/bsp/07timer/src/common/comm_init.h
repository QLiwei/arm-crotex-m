/**
 * @file comm_init.h
 * @brief Generic automatically initialization definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __COMM_INIT_H__
#define __COMM_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__ARMCC_VERSION)           /* ARM Compiler */
    #include <stdarg.h>
    #define COMM_SECTION(x)               __attribute__((section(x)))
    #define COMM_USED                     __attribute__((used))
    #define COMM_ALIGN(n)                 __attribute__((aligned(n)))
    #define COMM_WEAK                     __attribute__((weak))
    #define comm_inline                   static __inline
#elif defined(__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    #include <stdarg.h>
    #define COMM_SECTION(x)               @ x
    #define COMM_USED                     __root
    #define COMM_PRAGMA(x)                _Pragma(#x)
    #define COMM_ALIGN(n)                 PRAGMA(data_alignment=n)
    #define COMM_WEAK                     __weak
    #define comm_inline                   static inline
#elif defined(__GNUC__)                /* GNU GCC Compiler */
    #include <stdarg.h>
    #define COMM_SECTION(x)               __attribute__((section(x)))
    #define COMM_USED                     __attribute__((used))
    #define COMM_ALIGN(n)                 __attribute__((aligned(n)))
    #define COMM_WEAK                     __attribute__((weak))
    #define comm_inline                   static __inline
#else
    #error not supported tool chain
#endif

typedef int (*init_fn_t)(void);
#define INIT_EXPORT(fn, level)                                                          \
            COMM_USED const init_fn_t __comm_init_##fn COMM_SECTION(".comm_i_fn." level) = fn

/* board init routines will be called in board_init() function */
#define INIT_BOARD_EXPORT(fn)           INIT_EXPORT(fn, "1")

/* pre/device/component/env/app init routines will be called in init_thread */
/* components pre-initialization (pure software initialization) */
#define INIT_PREV_EXPORT(fn)            INIT_EXPORT(fn, "2")
/* device initialization */
#define INIT_DEVICE_EXPORT(fn)          INIT_EXPORT(fn, "3")
/* components initialization (dfs, lwip, ...) */
#define INIT_COMPONENT_EXPORT(fn)       INIT_EXPORT(fn, "4")
/* environment initialization (mount disk, ...) */
#define INIT_ENV_EXPORT(fn)             INIT_EXPORT(fn, "5")
/* application initialization (rtgui application etc ...) */
#define INIT_APP_EXPORT(fn)             INIT_EXPORT(fn, "6")

int system_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_INIT_H__ */

