#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct device device_t;

struct device_state {
    uint32_t init_res : 8;
    bool initialized : 1;
};

struct device {
    const char *name;
    const void *config;
    struct device_state *state;
    void *(*device_init)(const device_t *dev);
};

#define DEVICE_REGISTER(type, index, name, config, state, init)         \
    __attribute__((used)) const device_t __device_##type##_##index      \
    __attribute__((section(".device"))) = {                             \
        .name = name,                                                   \
        .config = config,                                               \
        .state = state,                                                 \
        .device_init = init,                                            \
    }

const device_t *device_find(const char *name);
void *device_init(const device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_H__ */