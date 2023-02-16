/**
 * @file device_beep.c
 * @brief beep device
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-14       vector(vector_qiu@163.com)      first version
 *
 */

#include "device_beep.h"

device_beep_t g_device_beep_list[BEEP_ID_MAX] = {0};

typedef struct {
    GPIO_TypeDef *gpiox;
    uint16_t gpio_pin;
}x_gpio_t;

static const x_gpio_t s_gpio_list[] = {
    {GPIOG, GPIO_PIN_7},
};

static void device_beep_hard_init(const x_gpio_t *dev);
static void device_beep_on(const x_gpio_t *dev);
static void device_beep_off(const x_gpio_t *dev);

/**
 * @brief beep device hardware initialization
 *
 * @param dev Buzzer pin information
 */
static void device_beep_hard_init(const x_gpio_t *dev) {
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

    device_beep_off(dev);

    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 设置推挽输出模式 */
    gpio_init_struct.Pull = GPIO_NOPULL;                    /* 上拉电阻使能 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* GPIO速度等级 */
    gpio_init_struct.Pin = dev->gpio_pin;
    HAL_GPIO_Init(dev->gpiox, &gpio_init_struct);
}

/**
 * @brief The buzzer beeped
 *
 * @param dev Buzzer pin information
 */
static void device_beep_on(const x_gpio_t *dev) {
    HAL_GPIO_WritePin(dev->gpiox, dev->gpio_pin, GPIO_PIN_SET);
}

/**
 * @brief The buzzer doesn't chime
 *
 * @param dev Buzzer pin information
 */
static void device_beep_off(const x_gpio_t *dev) {
    HAL_GPIO_WritePin(dev->gpiox, dev->gpio_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set the beep device parameters to enable the buzzer
 *
 * @param _id beep device id
 * @param _beep_time One cycle buzzer time
 * @param _stop_time A period of time when the buzzer does not sound
 * @param _cycle The number of periods the buzzer sounds
 */
void device_beep_start(uint8_t _id, uint16_t _beep_time, uint16_t _stop_time, uint16_t _cycle) {
    if (_id >= BEEP_ID_MAX || _beep_time == 0 || _stop_time == 0) {
        return;
    }
    g_device_beep_list[_id].beep_time = _beep_time;
    g_device_beep_list[_id].stop_time = _stop_time;
    g_device_beep_list[_id].count = 0;
    g_device_beep_list[_id].cycle = _cycle;
    g_device_beep_list[_id].cycle_count = 0;
    g_device_beep_list[_id].state = 0;
    g_device_beep_list[_id].enable = 1;

    device_beep_on(&s_gpio_list[_id]);
}

/**
 * @brief Turn off the buzzer
 *
 * @param _id beep device id
 */
void device_beep_stop(uint8_t _id) {
    if (_id >= BEEP_ID_MAX) {
        return;
    }
    g_device_beep_list[_id].enable = 0;

    if (g_device_beep_list[_id].stop_time == 0 || g_device_beep_list[_id].cycle == 0) {
        device_beep_off(&s_gpio_list[_id]); /* 必须在清控制标志后再停止发声，避免停止后在中断中又开启 */
    }
}

/**
 * @brief The buzzer is silent
 *
 * @param _id beep device id
 */
void device_beep_pause(uint8_t _id) {
    if (_id >= BEEP_ID_MAX) {
        return;
    }
    device_beep_off(&s_gpio_list[_id]);
    g_device_beep_list[_id].mute = 1;
}

/**
 * @brief The buzzer is resume
 *
 * @param _id beep device id
 */
void device_beep_resume(uint8_t _id) {
    if (_id >= BEEP_ID_MAX) {
        return;
    }
    device_beep_off(&s_gpio_list[_id]);
    g_device_beep_list[_id].mute = 0;
}

/**
 * @brief keypad tone
 *
 * @param _id beep device id
 */
void device_beep_key_tone(uint8_t _id) {
    if (_id >= BEEP_ID_MAX) {
        return;
    }
    device_beep_start(_id, 5, 1, 1);    /*beep:50ms stop:10ms, once */
}

/**
 * @brief beep equipment sound processing
 *
 */
void device_beep_handle(void) {
    uint8_t i;
    for (i = 0; i < ARRAY_SIZE(g_device_beep_list); i++) {
        if ((g_device_beep_list[i].enable == 0) || (g_device_beep_list[i].stop_time == 0) || (g_device_beep_list[i].mute == 1))
        {
            continue;
        }
        if (g_device_beep_list[i].state == 0) {
            if (g_device_beep_list[i].stop_time > 0) {
                if (++g_device_beep_list[i].count >= g_device_beep_list[i].beep_time) {
                    device_beep_off(&s_gpio_list[i]);
                    g_device_beep_list[i].count = 0;
                    g_device_beep_list[i].state = 1;
                }
            } else {
                ; /* Continuous chirping */
            }
        } else if (g_device_beep_list[i].state == 1) {
            if (++g_device_beep_list[i].count >= g_device_beep_list[i].stop_time) {
                if (g_device_beep_list[i].cycle > 0) {
                    if (++g_device_beep_list[i].cycle_count >= g_device_beep_list[i].cycle) {
                        g_device_beep_list[i].enable = 0;
                    }

                    if (g_device_beep_list[i].enable == 0) {
                        g_device_beep_list[i].stop_time = 0;
                        continue;
                    }
                }

                g_device_beep_list[i].count = 0;
                g_device_beep_list[i].state = 0;

                device_beep_on(&s_gpio_list[i]);
            }
        }
    }
}


/**
 * @brief The beep device is initialized automatically
 *
 * @return int
 */
int beep_device_init(void)
{
	uint8_t i;
	for (i = 0; i <  ARRAY_SIZE(s_gpio_list); i++) {
		device_beep_hard_init(&s_gpio_list[i]);
	}
    return 0;
}
INIT_DEVICE_EXPORT(beep_device_init);
