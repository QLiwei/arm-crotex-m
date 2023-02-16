/**
 * @file soft_timer.c
 * @brief soft timer component
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-14       vector(vector_qiu@163.com)      first version
 *
 */

#include "soft_timer.h"

static soft_timer_t s_task_list_t[ST_INDEX_MAX];

/**
 * @brief soft timer count update,Called during a timer interrupt
 *
 * @note The counting period of soft timer is generally 1ms
 */
void soft_timer_update(void) {
    int i;
    for (i = 0; i < ST_INDEX_MAX; i++) {
        if (s_task_list_t[i].enable != false) {
            s_task_list_t[i].counter++;
            if (s_task_list_t[i].counter >= s_task_list_t[i].cycle) {
                s_task_list_t[i].counter = 0;
                s_task_list_t[i].event_occur = true;

                if (s_task_list_t[i].repeat > 0) {
                    if (s_task_list_t[i].repeat != ST_INFINIT_REPEAT) {
                        s_task_list_t[i].repeat--;
                    }
                } else {
                    s_task_list_t[i].enable = false;
                }
            }
        }
    }
}

/**
 * @brief Setting task parameters
 *
 * @param _index Task index
 * @param _cycle Task cycle
 * @param _repeat The number of task repetitions
 * @param _cb_func Task function
 * @return error_t EOK: successfully set -EINVAL: Task index inexistence
 */
error_t soft_timer_set(uint16_t _index, uint32_t _cycle, uint16_t _repeat, soft_timer_task_cb _cb_func) {
    if (_index >= ST_INDEX_MAX) {
        return -EINVAL;
    }

    s_task_list_t[_index].enable = false;
    s_task_list_t[_index].event_occur = false;
    s_task_list_t[_index].counter = 0;
    s_task_list_t[_index].cycle = _cycle;
    s_task_list_t[_index].repeat = _repeat;
    s_task_list_t[_index].cb_func = _cb_func;

    s_task_list_t[_index].enable = true;

    return EOK;
}

/**
 * @brief Disabled task
 *
 * @param _index Task index
 * @return error_t EOK: successfully disable -EINVAL: Task index inexistence
 */
error_t soft_timer_disable(uint16_t _index) {
    if (_index >= ST_INDEX_MAX) {
        return -EINVAL;
    }

    s_task_list_t[_index].enable = false;
    s_task_list_t[_index].event_occur = false;
    s_task_list_t[_index].counter = 0;
    s_task_list_t[_index].repeat = 0;
    s_task_list_t[_index].cb_func = NULL;

    return EOK;
}

/**
 * @brief Reset Task
 *
 * @param _index Task index
 * @return error_t EOK: successfully reset -EINVAL: Task index inexistence
 */
error_t soft_timer_reset(uint16_t _index) {
    if (_index >= ST_INDEX_MAX) {
        return -EINVAL;
    }

    s_task_list_t[_index].counter = 0;
    return EOK;
}

/**
 * @brief Get task state
 *
 * @param _index Task index
 * @return true enabled
 * @return false disable
 */
bool soft_timer_state(uint16_t _index) {
    if (_index >= ST_INDEX_MAX) {
        return -EINVAL;
    }

    return s_task_list_t[_index].enable;
}

/**
 * @brief task scheduling
 *
 */
void soft_timer_run(void) {
    uint16_t i;

    for (i = 0; i < ST_INDEX_MAX; i++) {
        if (s_task_list_t[i].event_occur != false) {
            s_task_list_t[i].event_occur = false;

            if (s_task_list_t[i].cb_func != NULL) {
                s_task_list_t[i].cb_func();
            }
        }
    }
}

