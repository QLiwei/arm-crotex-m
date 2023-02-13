/**
 * @file comm_init.c
 * @brief Generic automatically initialization
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */

#include "comm_init.h"

static int i_start(void)
{
    return 0;
}
INIT_EXPORT(i_start, "0");

static int i_board_start(void)
{
    return 0;
}
INIT_EXPORT(i_board_start, "0.end");

static int i_board_end(void)
{
    return 0;
}
INIT_EXPORT(i_board_end, "1.end");

static int i_end(void)
{
    return 0;
}
INIT_EXPORT(i_end, "6.end");


void comm_board_init(void) {
    volatile const init_fn_t *fn_ptr;
    for (fn_ptr = &__comm_init_i_board_start; fn_ptr < &__comm_init_i_board_end; fn_ptr++)
    {
        (*fn_ptr)();
    }
}

void comm_components_init(void) {
    volatile const init_fn_t *fn_ptr;
    for (fn_ptr = &__comm_init_i_board_end; fn_ptr < &__comm_init_i_end; fn_ptr++)
    {
        (*fn_ptr)();
    }
}

int system_init(void)
{
    comm_board_init();
    comm_components_init();
    return 0;
}
