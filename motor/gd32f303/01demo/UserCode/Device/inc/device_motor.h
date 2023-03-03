#ifndef __DEVICE_MOTOR_H__
#define __DEVICE_MOTOR_H__

#include "gd32f30x.h"
#include "bsp_timer.h"
#include "bsp_encoder.h"
#include "utils_pid.h"

#define SHUTDOWM_GPIO_CLK   RCU_GPIOC
#define SHUTDOWM_GPIO_PORT  GPIOC
#define SHUTDOWM_GPIO_PIN   GPIO_PIN_0

#define SET_FWD_COMPAER(ChannelPulse)     timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,ChannelPulse)
#define SET_REV_COMPAER(ChannelPulse)     timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,ChannelPulse)
#define COMPAER_MAX     (TIMER_PERIOD - 100)
#define VELOCITY_MEASURING_PERIOD              10         // 测速周期：ms

#define PID_ASSISTANT_EN

typedef enum {
    MOTOR_FWD = 0,
    MOTOR_REV,
}motor_dir_t;

extern utils_pid_t current_pid;
extern utils_pid_t speed_pid;

void device_motor_set_speed(uint16_t v);
int32_t device_motor_get_speed(void); // 单位 r
void device_motor_set_direction(motor_dir_t dir);
motor_dir_t device_motor_get_direction(void);
void device_motor_enable(void);
void device_motor_disable(void);
void device_motor_pid_control(void);
void device_motor_init(void);

#endif /* __DEVICE_MOTOR_H__ */

