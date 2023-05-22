#ifndef __DRIVER_GPIO_H__
#define __DRIVER_GPIO_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gpio_read_t gpio_read_t;
struct gpio_read_t {
    uint8_t (*read)(gpio_read_t *self);
};

typedef struct gpio_write_t gpio_write_t;
struct gpio_write_t {
    void (*write)(gpio_write_t *self, uint8_t state);
    void (*toggle)(gpio_write_t *self);
};

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_GPIO_H__ */
