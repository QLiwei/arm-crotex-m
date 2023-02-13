/**
 * @file device_switch_led.c
 * @brief switch led device
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#include "device_switch_led.h"

switch_led_device_t g_switch_led_devs[] = {
#ifdef DEVICE_LED1
    LED1_CONFIG
#endif

#ifdef DEVICE_LED2
        LED2_CONFIG
#endif

#ifdef DEVICE_LED3
            LED3_CONFIG
#endif
};

void switch_led_on(switch_led_device_t *dev)
{
    HAL_GPIO_WritePin(dev->gpiox, dev->gpio_pin, GPIO_PIN_RESET);
    dev->state = 1;
}

void switch_led_off(switch_led_device_t *dev)
{
    HAL_GPIO_WritePin(dev->gpiox, dev->gpio_pin, GPIO_PIN_SET);
    dev->state = 0;
}

void switch_led_toggle(switch_led_device_t *dev) {
    HAL_GPIO_TogglePin(dev->gpiox, dev->gpio_pin);
}

void switch_led_gpio_init(switch_led_device_t *dev)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (dev->gpiox == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    else if (dev->gpiox == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }

    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;   /* 设置推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;           /* 上下拉电阻不使能 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* GPIO速度等级 */
    gpio_init_struct.Pin = dev->gpio_pin;
    HAL_GPIO_Init(dev->gpiox, &gpio_init_struct);

    switch_led_off(dev);
}

int switch_led_device_init(void)
{
    for (int i = 0; i < ARRAY_SIZE(g_switch_led_devs); i++)
    {
        switch_led_gpio_init(&g_switch_led_devs[i]);
    }
    return 0;
}
INIT_BOARD_EXPORT(switch_led_device_init);
