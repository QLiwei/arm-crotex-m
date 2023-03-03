/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022
 * All rights reserved.
 *
 ******************************************************************************
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-08     vector       the first version
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
extern volatile uint32_t control_period;
/**
 * @brief The application entry point.
 *
 * @return int
 */
int main(void) {
    utils_pid_t pid_config;
    /* bsp layer initialize */
    bsp_init();
    device_init();
    protocol_init();
    /* 电流PID初始化 mA */
    pid_config.target_value = 0.0f;
    pid_config.kp = 0.0f;
    pid_config.ki = 3.5f;
    pid_config.kd = 0.0f;
    utils_pid_param_init(&current_pid, pid_config);
#if defined(PID_ASSISTANT_EN)
    float utils_pid_temp[3] = {0};
    utils_pid_temp[0] = current_pid.kp;
    utils_pid_temp[1] = current_pid.ki;
    utils_pid_temp[2] = current_pid.kd;
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, utils_pid_temp, 3);     // 给通道1发送current P I D 值
#endif
    /* 速度PID初始化 r/min */
    pid_config.target_value = 0.0f;
    pid_config.kp = 5.0f;
    pid_config.ki = 2.0f;
    pid_config.kd = 0.0f;
    utils_pid_param_init(&speed_pid, pid_config);
#if defined(PID_ASSISTANT_EN)
    utils_pid_temp[0] = speed_pid.kp;
    utils_pid_temp[1] = speed_pid.ki;
    utils_pid_temp[2] = speed_pid.kd;
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH2, utils_pid_temp, 3);     // 给通道2发送speed P I D 值
#endif
    LOG_I("Firmware compile time:%s %s", __DATE__, __TIME__);
#if defined(PID_ASSISTANT_EN)
    set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);    // 同步上位机的启动按钮状态
    set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &current_pid.target_value, 1);     // 给通道 1 发送目标值
    set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &speed_pid.target_value, 1);     // 给通道 2 发送目标值
#endif
    while (1) {
       receiving_process();
#if 0
        delay_1ms(50);
        LOG_D("current_value:%f mA", bsp_adc_get_current_value());
        LOG_D("motor_speed:%d r/min", device_motor_get_speed());
        LOG_D("power_supply_value:%f V", bsp_adc_get_power_supply_value());
#endif
    }
}
