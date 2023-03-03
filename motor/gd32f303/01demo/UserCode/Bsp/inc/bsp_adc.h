#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "gd32f30x.h"
#include "systick.h"

// 多次记录偏置电压，待系统稳定偏置电压才为有效值 1.5s
#define OFFSET_COUNT    300
#define ADC_NUM_MAX     64
#define VREF            3.3f
#define GET_ADC_VOLTAGE_VALUE(value)    ((float)value/4096.0f*VREF)
// 分辨率为5mA
#define GET_CURRENT_VALUE(value)        (((float)value) / 8.0f/ 0.02f * 2000.0f)         // 得到电流值，电压放大8倍，0.02是采样电阻，单位mA。

#define GET_POWER_SUPPLY_VALUE(value)   (((float)value - 1.24f) * 37.0f)      // 电压最大值（测量电压是电源电压的1/37）

float bsp_adc_get_current_value(void);
float bsp_adc_get_power_supply_value(void);
uint16_t bsp_adc_get_offset_flag(void);
void bsp_adc_init(void);

#endif /* __BSP_ADC_H__ */
