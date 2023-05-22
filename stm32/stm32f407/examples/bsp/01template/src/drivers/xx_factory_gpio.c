#include "device.h"
#include "xx_driver_gpio.h"


static void *gd_gpio_new(const gpio_gd32_cfg_t *config);
static void *gd_gpio_init(const struct device *dev);

