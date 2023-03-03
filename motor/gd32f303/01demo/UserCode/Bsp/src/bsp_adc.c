#include "bsp_adc.h"

volatile uint16_t adc_value[ADC_NUM_MAX];
volatile uint32_t adc_current_sum = 0;
volatile uint32_t adc_current_count = 0;
volatile uint32_t adc_power_supply_average = 0;
uint16_t flag = 0;
/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void rcu_config(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4); //120Mhz / div4 = 30Mhz
}

/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void gpio_config(void)
{
    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_5);
    // gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_6);
    // gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ, GPIO_PIN_7);
}

/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&adc_value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = ADC_NUM_MAX;                  // 8.4us * 32 = 0.2688ms 64:0.5376
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_FTF);
    dma_interrupt_enable(DMA0, DMA_CH0, DMA_INT_FTF);
    nvic_irq_enable(DMA0_Channel0_IRQn, 0, 0);//使能中断并设置优先级
    dma_circulation_enable(DMA0, DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void adc_config(void)
{
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC continuous function enable */
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    /* ADC scan function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 2);

    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5); // (239.5 sampling cycles + 12.5) * ADCCLK = 8.4us
    // adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5);
    // adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

void DMA0_Channel0_IRQHandler(void) {
    if (dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF) != RESET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_FTF);
        uint32_t adc_mean;

        dma_channel_disable(DMA0, DMA_CH0);
        adc_mean = 0;
        /* PA4 ADC_CHANNEL_4 Current average value */
        for(uint32_t count = 0; count < ADC_NUM_MAX; count+=2)
        {
            adc_mean += (uint32_t)adc_value[count];
        }
        adc_current_sum += adc_mean / (ADC_NUM_MAX / 2);
        adc_current_count++;

        adc_mean = 0;
        for(uint32_t count = 1; count < ADC_NUM_MAX; count+=2)
        {
            adc_mean += (uint32_t)adc_value[count];
        }
        adc_power_supply_average = adc_mean / (ADC_NUM_MAX / 2);
        dma_channel_enable(DMA0, DMA_CH0);
    }
}
int32_t adc_offset = 0;    // 偏置电压
float bsp_adc_get_current_value(void) { // 5ms
    int16_t adc_current_average_value = 0;

    adc_current_average_value = adc_current_sum / adc_current_count;
    adc_current_count = 0;
    adc_current_sum = 0;

    if (flag < OFFSET_COUNT)
    {
        adc_offset = adc_current_average_value;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
        flag += 1;
    }
    if (adc_current_average_value >= adc_offset) {
        adc_current_average_value -= adc_offset;
    } else {
        adc_current_average_value = 0;
    }
    float voltage = GET_ADC_VOLTAGE_VALUE(adc_current_average_value);

    return GET_CURRENT_VALUE(voltage);
}

uint16_t bsp_adc_get_offset_flag(void) {
    return flag;
}

float bsp_adc_get_power_supply_value(void) {
    float voltage = GET_ADC_VOLTAGE_VALUE(adc_power_supply_average);
    return GET_POWER_SUPPLY_VALUE(voltage) > 0 ? GET_POWER_SUPPLY_VALUE(voltage) : 0;
}

void bsp_adc_init(void) {
    rcu_config();
    gpio_config();
    dma_config();
    adc_config();
}
