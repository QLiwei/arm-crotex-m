#include "device.h"

#include <stdio.h>
#include <string.h>

#if defined(__ARMCC_VERSION)           /* ARM Compiler */
extern const device_t Image$$device_region$$Base;
extern const device_t Image$$device_region$$Limit;
    #define DEVICE_START ((const device_t *)&Image$$device_region$$Base)
    #define DEVICE_END   ((const device_t *)&Image$$device_region$$Limit)
#elif defined(__GNUC__)                /* GNU GCC Compiler */
extern const device_t __device_start;
extern const device_t __device_end;
    #define DEVICE_START ((const device_t *)&__device_start)
    #define DEVICE_END   ((const device_t *)&__device_end)
#else
    #error not supported tool chain
#endif

void *device_init(const device_t *dev) {
    // 断言检查
    void *p = dev->device_init(dev);
    // 断言检查
    return p;
}

const device_t *device_find(const char *name) {
    const device_t *dev;
    for (dev = DEVICE_START; dev < DEVICE_END; dev++) {
        if (strcmp(dev->name, name) == 0 && dev->state->initialized == false) {
            return dev;
        }
    }
    return NULL;
}
