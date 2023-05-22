#include "driver_gpio.h"


typedef struct xx_gpio_read_t {
    gpio_read_t intf;
    uint32_t port;
    uint16_t pin;
}xx_gpio_read_t;

typedef struct xx_gpio_write_t {
    gpio_write_t intf;
    uint32_t port;
    uint16_t pin;
}xx_gpio_write_t;

static uint8_t _read(gpio_read_t *_self) {
    xx_gpio_read_t *self = _self;
    return gpio_input_bit_get(self->port, self->pin);
}

static void _write(gpio_write_t *_self, uint8_t state) {
    xx_gpio_write_t *self = _self;
    gpio_bit_write(self->port, self->pin, state);
}

static void _toggle(gpio_write_t *_self) {
    xx_gpio_write_t *self = _self; // ，根据偏移，在结构体中查找指定成员的地址
    gpio_bit_toggle(self->port, self->pin);
}

gpio_read_t *xx_gpio_read_new(const xx_gpio_config_t *config) {

    xx_gpio_read_t *self = (xx_gpio_read_t *)malloc(sizeof(xx_gpio_read_t));
    if (!self) {
        return NULL;
    }
    self->port = xxx;
    self->pin = xxx;
    self->intf.read = _read;
    return &self->intf;
}

gpio_write_t *xx_gpio_write_new(const xx_gpio_config_t *config) {
    xx_gpio_write_t *self = (xx_gpio_write_t *)malloc(sizeof(xx_gpio_write_t));
    if (!self) {
        return NULL;
    }
    self->port = xxx;
    self->pin = xxx;
    self->intf.write = _write;
    return &self->intf;
}