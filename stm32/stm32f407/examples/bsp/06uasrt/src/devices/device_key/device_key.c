/**
 * @file device_key.c
 * @brief key device
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-13       vector(vector_qiu@163.com)      first version
 *
 */
#include "device_key.h"

#define HARD_KEY_NUM    KID_MAX
#define KEY_COUNT       HARD_KEY_NUM

typedef struct {
    GPIO_TypeDef *gpiox;
    uint16_t gpio_pin;
    uint8_t active_level;   /* 激活电平 */
}x_gpio_t;


static const x_gpio_t s_gpio_list_t[HARD_KEY_NUM] = {
    {GPIOF, GPIO_PIN_6, 0},
    {GPIOF, GPIO_PIN_7, 0},
    {GPIOF, GPIO_PIN_8, 0},
    {GPIOF, GPIO_PIN_9, 0},
};

static key_device_t s_btn[KEY_COUNT] = {0};
static key_fifo_t s_fifo_key;

static uint8_t is_key_down_func(uint8_t _id);
static void device_key_hard_init(const x_gpio_t *dev);
static void device_key_value_init(void);
static void device_detect_key(uint8_t i);
static void device_detect_fastio(uint8_t i);

/**
 * @brief Determine whether the key is pressed
 *
 * @param _id Key ID
 * @return uint8_t 1 means pressed (conduction), 0 means not pressed (release)
 */
static uint8_t key_pin_active(uint8_t _id) {
    uint8_t level;

    if ((s_gpio_list_t[_id].gpiox->IDR & s_gpio_list_t[_id].gpio_pin) == 0) {
        level = 0;
    } else {
        level = 1;
    }

    if (level == s_gpio_list_t[_id].active_level) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Determine whether the key is pressed. Distinguish between single keys and composite keys.
 *  Single-key events do not allow other keys to be pressed.
 *
 * @param _id Key ID
 * @return uint8_t 1 means pressed (conduction), 0 means not pressed (release)
 */
static uint8_t is_key_down_func(uint8_t _id) {
    if (_id < HARD_KEY_NUM) {
        uint8_t i;
        uint8_t count = 0;
        uint8_t save = 255;

        /* single keys */
        for (i = 0; i < HARD_KEY_NUM; i++) {
            if (key_pin_active(i)) {
                count++;
                save = i;
            }
        }

        if (count == 1 && save == _id) {
            return 1;
        }
        return 0;
    }
    /* composite keys */
    // if (_id == HARD_KEY_NUM + 0) {
    //     if (key_pin_active(0) && key_pin_active(1)) {
    //         return 1;
    //     } else {
    //         return 0;
    //     }
    // }
    return 0;
}

/**
 * @brief Initial GPIO clock to configure the GPIO for the key
 *
 * @param dev key device
 */
static void device_key_hard_init(const x_gpio_t *dev) {
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

    gpio_init_struct.Mode = GPIO_MODE_INPUT;                /* 设置输入模式 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉电阻使能 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* GPIO速度等级 */
    gpio_init_struct.Pin = dev->gpio_pin;
    HAL_GPIO_Init(dev->gpiox, &gpio_init_struct);
}

/**
 * @brief Initialize the key variable
 *
 */
static void device_key_value_init(void) {
    uint8_t i;

    s_fifo_key.read = 0;
    s_fifo_key.read2 = 0;
    s_fifo_key.write = 0;

    /* Keypress default parameter assignment */
    for (i = 0; i < KEY_COUNT; i++) {
        s_btn[i].long_time = KEY_LONG_TIME;
        s_btn[i].count = KEY_FILTER_TIME / 2;
        s_btn[i].state =0;
        s_btn[i].repeat_speed = 0;
        s_btn[i].repeat_count = 0;
    }

    /* Special assignment for key parameters */

}

/**
 * @brief Push 1 key value into the key FIFO buffer. Can be used to simulate a key press.
 *
 * @param _key_code The key press code
 */
void device_put_key(uint8_t _key_code) {
    s_fifo_key.buf[s_fifo_key.write] = _key_code;

    if (++s_fifo_key.write >= KEY_FIFO_SIZE) {
        s_fifo_key.write = 0;
    }
}

/**
 * @brief Read a key value from the key FIFO buffer.
 *
 * @return uint8_t Key press code
 */
uint8_t device_get_key(void) {
    uint8_t ret;

    if (s_fifo_key.read == s_fifo_key.write) {
        return KEY_NONE;
    } else {
        ret = s_fifo_key.buf[s_fifo_key.read];

        if (++s_fifo_key.read >= KEY_FIFO_SIZE) {
            s_fifo_key.read = 0;
        }
        return ret;
    }
}

/**
 * @brief Read a key value from the key FIFO buffer. Separate read Pointers.
 *
 * @return uint8_t Key press code
 */
uint8_t device_get_key2(void) {
    uint8_t ret;

    if (s_fifo_key.read2 == s_fifo_key.write) {
        return KEY_NONE;
    } else {
        ret = s_fifo_key.buf[s_fifo_key.read2];

        if (++s_fifo_key.read2 >= KEY_FIFO_SIZE) {
            s_fifo_key.read2 = 0;
        }
        return ret;
    }
}

/**
 * @brief Read the state of the key press
 *
 * @param _key_id Key ID, starting at 0
 * @return uint8_t 1 means pressed and 0 means not pressed
 */
uint8_t device_get_key_state(KEY_ID_E _key_id) {
    return s_btn[_key_id].state;
}

/**
 * @brief Setting key parameters
 *
 * @param _key_id Key ID, starting at 0
 * @param _long_time Long press the event time
 * @param _repeat_speed Speed of succession
 */
void device_set_key_param(uint8_t _key_id, uint16_t _long_time, uint8_t _repeat_speed) {
    s_btn[_key_id].long_time = _long_time;
    s_btn[_key_id].repeat_speed = _repeat_speed;
    s_btn[_key_id].repeat_count = 0;
}

/**
 * @brief Clear the key FIFO buffer
 *
 */
void device_clear_key(void) {
    s_fifo_key.read = s_fifo_key.write;
}

/**
 * @brief Detecting a key press. Non-blocking state, which must be invoked periodically.
 *
 * @param i The id of the Key, encoded starting from 0
 */
static void device_detect_key(uint8_t i) {
    key_device_t *pbtn;

    pbtn = &s_btn[i];
    if (is_key_down_func(i)) {
        if (pbtn->count < KEY_FILTER_TIME) {
            pbtn->count = KEY_FILTER_TIME;
        } else if (pbtn->count < 2 * KEY_FILTER_TIME) {
            pbtn->count++;
        } else {
            if (pbtn->state == 0) {
                pbtn->state = 1;
                device_put_key((uint8_t)(3 * i + 1));
            }

            if (pbtn->long_time > 0) {
                if (pbtn->long_count < pbtn->long_time) {
                    if (++pbtn->long_count == pbtn->long_time) {
                        device_put_key((uint8_t)(3 * i + 3));
                    }
                } else {
                    if (pbtn->repeat_speed > 0) {
                        if (++pbtn->repeat_count >= pbtn->repeat_speed) {
                            pbtn->repeat_count = 0;
                            device_put_key((uint8_t)(3 * i +1));
                        }
                    }
                }
            }
        }
    } else {
        if (pbtn->count > KEY_FILTER_TIME) {
            pbtn->count = KEY_FILTER_TIME;
        } else if (pbtn->count != 0) {
            pbtn->count--;
        } else {
            if (pbtn->state == 1) {
                pbtn->state = 0;
                device_put_key((uint8_t)(3 * i + 2));
            }
        }
        pbtn->long_count = 0;
        pbtn->repeat_count = 0;
    }
}

/**
 * @brief Check the high speed input IO.1ms refresh
 *
 * @param i The id of the Key, encoded starting from 0
 */
static void device_detect_fastio(uint8_t i) {
    key_device_t *pbtn;

    pbtn = &s_btn[i];
    if (is_key_down_func(i)) {
        if (pbtn->state == 0) {
            pbtn->state = 1;
            device_put_key((uint8_t)(3 * i + 1));
        }

        if (pbtn->long_time > 0) {
            if (pbtn->long_count < pbtn->long_time) {
                if (++pbtn->long_count == pbtn->long_time) {
                    device_put_key((uint8_t)(3 * i + 3));
                }
            } else {
                if (pbtn->repeat_speed > 0) {
                    if (++pbtn->repeat_count >= pbtn->repeat_speed) {
                        pbtn->repeat_count = 0;
                        device_put_key((uint8_t)(3 * i +1));
                    }
                }
            }
        }
    } else {
        if (pbtn->state == 1) {
            pbtn->state = 0;
            device_put_key((uint8_t)(3 * i + 2));
        }
        pbtn->long_count = 0;
        pbtn->repeat_count = 0;
    }
}

/**
 * @brief Scan all keystrokes. Non-blocking, periodic calls interrupted by systick, 10ms at a time
 *
 */
void device_key_scan_10ms(void) {
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        device_detect_key(i);
    }
}

/**
 * @brief Scan all keystrokes. Non-blocking, periodic calls interrupted by systick, 1ms at a time.
 *
 */
void device_key_scan_1ms(void) {
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        device_detect_fastio(i);
    }
}

/**
 * @brief The key device is initialized automatically
 *
 * @return int
 */
int key_device_init(void)
{
    device_key_value_init();

    for (int i = 0; i < ARRAY_SIZE(s_gpio_list_t); i++)
    {
        device_key_hard_init(&s_gpio_list_t[i]);
    }
    return 0;
}
INIT_DEVICE_EXPORT(key_device_init);
