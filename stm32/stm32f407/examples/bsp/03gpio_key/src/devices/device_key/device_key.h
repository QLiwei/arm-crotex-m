/**
 * @file device_key.h
 * @brief key device definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-13       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __DEVICE_KEY__
#define __DEVICE_KEY__

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 按键参数宏定义 单位为按键扫描的周期 一般为10ms */
#define KEY_FILTER_TIME   5
#define KEY_LONG_TIME     100			/* 单位10ms， 持续1秒，认为长按事件 */
#define KEY_FIFO_SIZE     10

/* 按键ID编号 */
typedef enum {
	KID_K1 = 0,
	KID_K2,
	KID_K3,
	KID_K4,
}KEY_ID_E;

/* 按键事件 */
typedef enum {
	KEY_NONE = 0,			/* 0 表示按键事件 */

	KEY_1_DOWN,
	KEY_1_UP,
	KEY_1_LONG,

	KEY_2_DOWN,
	KEY_2_UP,
	KEY_2_LONG,

	KEY_3_DOWN,
	KEY_3_UP,
	KEY_3_LONG,

	KEY_4_DOWN,
	KEY_4_UP,
	KEY_4_LONG,
}KEY_ENUM;

typedef struct key_device{
    uint8_t (*is_key_down_func)(void); 	/* 按键按下的判断函数,1表示按下 */
	/* 单位: 按键扫描的周期 */
    uint8_t  count;			/* 滤波器计数器 */
	uint16_t long_count;	/* 长按计数器 */
	uint16_t long_time;		/* 按键按下持续时间, 0表示不检测长按 */
	uint8_t  state;			/* 按键当前状态（按下还是弹起） */
	uint8_t  repeat_speed;	/* 连续按键周期 */
	uint8_t  repeat_count;	/* 连续按键计数器 */
}key_device_t;

typedef struct
{
	uint8_t buf[KEY_FIFO_SIZE];		/* 键值缓冲区 */
	uint8_t read;					/* 缓冲区读指针1 */
	uint8_t write;					/* 缓冲区写指针 */
	uint8_t read2;					/* 缓冲区读指针2 用于截屏处理（按照顺序触发按键） */
}key_fifo_t;

void device_put_key(uint8_t _key_code);
uint8_t device_get_key(void);
uint8_t device_get_key2(void);
uint8_t device_get_key_state(KEY_ID_E _key_id);
void device_set_key_param(uint8_t _key_id, uint16_t _long_time, uint8_t _repeat_speed);
void device_key_scan_10ms(void);
void device_key_scan_1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_KEY__ */
