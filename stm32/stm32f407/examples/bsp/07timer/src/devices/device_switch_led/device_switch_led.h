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

typedef enum {
    DEVICE_LED01 = 0,
    DEVICE_LED02,
    DEVICE_LED03,
    DEVICE_LED_ID_MAX,
}device_led_id_e;

typedef enum {
    DEVICE_LED_OFF,
    DEVICE_LED_ON,
}device_led_state_e;

device_led_state_e switch_led_on(device_led_id_e _id);
device_led_state_e switch_led_off(device_led_id_e _id);
void switch_led_toggle(device_led_id_e _id);

#ifdef __cplusplus
}
#endif

#endif /* __SWITCH_LED_DEVICE_H__ */
