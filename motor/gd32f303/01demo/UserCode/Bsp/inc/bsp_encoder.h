#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "gd32f30x.h"

#define ENCODER_TIM_PRESCALER   0
#define ENCODER_TIM_PERIOD      65535

#define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 2倍频后的总分辨率 */
/* 编码器物理分辨率 */
#define ENCODER_RESOLUTION                     16
/* 减速电机减速比 */
#define REDUCTION_RATIO                        30

int32_t bsp_encoder_get_current_count(void);
void bsp_encoder_init(void);

#endif /* __BSP_ENCODER_H__ */
