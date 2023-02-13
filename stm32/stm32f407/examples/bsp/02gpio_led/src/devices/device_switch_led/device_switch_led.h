/**
 * @file device_switch_led.h
 * @brief switch led device definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __DEVICE_SWITCH_LED_H__
#define __DEVICE_SWITCH_LED_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_LED1
#define DEVICE_LED2
#define DEVICE_LED3

#define LED1_CONFIG     {                           \
                            GPIOE,                  \
                            GPIO_PIN_3,              \
                            0,                      \
                        },                          \

#define LED2_CONFIG     {                           \
                            GPIOE,                  \
                            GPIO_PIN_4,              \
                            0,                      \
                        },                          \

#define LED3_CONFIG     {                           \
                            GPIOG,                  \
                            GPIO_PIN_9,              \
                            0,                      \
                        },                          \


// GPIOx PINx en
typedef struct switch_led_device
{
    GPIO_TypeDef *gpiox;
    uint16_t gpio_pin;
    uint8_t state;
} switch_led_device_t;
extern switch_led_device_t g_switch_led_devs[];

void switch_led_on(switch_led_device_t *dev);
void switch_led_off(switch_led_device_t *dev);
void switch_led_toggle(switch_led_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __SWITCH_LED_DEVICE_H__ */
