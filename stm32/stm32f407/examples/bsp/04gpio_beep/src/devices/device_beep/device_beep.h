/**
 * @file device_beep.h
 * @brief beep device definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-14       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __DEVICE_BEEP_H__
#define __DEVICE_BEEP_H__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BEEP_ID_0 = 0,
    BEEP_ID_MAX,
}BEEP_ID_E;

typedef struct device_beep
{
    uint8_t enable;         /* 0:disable 1:enable */
    uint8_t state;          /* 0:beep 1:stop */
    uint16_t beep_time;     /* One cycle buzzer time */
    uint16_t stop_time;     /* A period of time when the buzzer does not sound */
    uint16_t count;         /* time count */
    uint16_t cycle;         /* The number of periods the buzzer sounds */
    uint16_t cycle_count;   /* cycle count */
    uint8_t mute;           /* 1: mute*/
}device_beep_t;

void device_beep_start(uint8_t _id, uint16_t _beep_time, uint16_t _stop_time, uint16_t _cycle);
void device_beep_stop(uint8_t _id);
void device_beep_pause(uint8_t _id);
void device_beep_resume(uint8_t _id);
void device_beep_key_tone(uint8_t _id);
void device_beep_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_BEEP_H__ */
