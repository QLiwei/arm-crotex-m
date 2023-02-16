/**
 * @file soft_timer.h
 * @brief soft timer component definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-14       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __SOFT_TIMER_H__
#define __SOFT_TIMER_H__

#include "comm_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST_NONE_REPEAT    0
#define ST_INFINIT_REPEAT   0xFFFF

#define ST_CYCLE_1MS	1
#define ST_CYCLE_10MS	10
#define ST_CYCLE_100MS	100
#define ST_CYCLE_1000MS	1000

typedef enum {
    ST_DEFAULT_INDEX = 0,
	ST_KEY_SCAN_TASK_INDEX,
	ST_BEEP_TASK_INDEX,
    ST_KEY_HANDLE_TASK_INDEX,
    ST_INDEX_MAX,
} ST_INDEX_E;

typedef void (*soft_timer_task_cb)(void);

typedef struct {
    volatile bool enable;           /* Whether the task is enabled or not true:enable false:disable*/
    volatile bool event_occur;      /* Task scheduling event true:execute task callback function */
    volatile uint16_t repeat;       /* The number of task repetitions */
    volatile uint32_t counter;      /* Task counter */
    volatile uint32_t cycle;        /* Task cycle */
    soft_timer_task_cb cb_func;     /* Task callback function */
} soft_timer_t;

void soft_timer_update(void);
error_t soft_timer_set(uint16_t _index, uint32_t _cycle, uint16_t _repeat, soft_timer_task_cb _cb_func);
error_t soft_timer_disable(uint16_t _index);
error_t soft_timer_reset(uint16_t _index);
bool soft_timer_state(uint16_t _index);
void soft_timer_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_TIMER_H__ */
