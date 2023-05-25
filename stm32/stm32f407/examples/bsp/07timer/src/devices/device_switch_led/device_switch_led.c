/**
 * @file device_switch_led.c
 * @brief switch led device
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 * 2023-05-18       vecror(vector.qiu@gmail.com)    Optimize LED device interface
 */
#include "device_switch_led.h"

typedef struct switch_led_device
{
    GPIO_TypeDef *gpiox;
    uint16_t gpio_pin;
} switch_led_device_t;
static switch_led_device_t s_switch_led_devs[DEVICE_LED_ID_MAX] = {
    {GPIOE, GPIO_PIN_3},
    {GPIOE, GPIO_PIN_4},
    {GPIOG, GPIO_PIN_9},
};

device_led_state_e switch_led_on(device_led_id_e _id)
{
    HAL_GPIO_WritePin(s_switch_led_devs[_id].gpiox, s_switch_led_devs[_id].gpio_pin, GPIO_PIN_RESET);
    return DEVICE_LED_ON;
}

device_led_state_e switch_led_off(device_led_id_e _id)
{
    HAL_GPIO_WritePin(s_switch_led_devs[_id].gpiox, s_switch_led_devs[_id].gpio_pin, GPIO_PIN_SET);
    return DEVICE_LED_OFF;
}

void switch_led_toggle(device_led_id_e _id) {
    HAL_GPIO_TogglePin(s_switch_led_devs[_id].gpiox, s_switch_led_devs[_id].gpio_pin);
}

void switch_led_gpio_init(device_led_id_e _id)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (s_switch_led_devs[_id].gpiox == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    else if (s_switch_led_devs[_id].gpiox == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }

    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;   /* 设置推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;           /* 上下拉电阻不使能 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;  /* GPIO速度等级 */
    gpio_init_struct.Pin = s_switch_led_devs[_id].gpio_pin;
    HAL_GPIO_Init(s_switch_led_devs[_id].gpiox, &gpio_init_struct);

    switch_led_off(_id);
}

int switch_led_device_init(void)
{
    for (int i = 0; i < DEVICE_LED_ID_MAX; i++)
    {
        switch_led_gpio_init((device_led_id_e)i);
    }
    return 0;
}
INIT_BOARD_EXPORT(switch_led_device_init);
