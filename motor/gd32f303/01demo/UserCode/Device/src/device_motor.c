#include "device_motor.h"
#include "bsp_adc.h"
#include "protocol.h"

#define LOG_TAG "DEVICE_MOTOR"
#define DBG_ENABLE
#define DBG_SECTION_NAME LOG_TAG
#define DBG_LEVEL        DBG_LOG
// #define DBG_COLOR
#include "utils_dbg.h"

#define TARGET_CURRENT_MAX    350    // 目标电流的最大值 mA
static motor_dir_t direction  = MOTOR_FWD;     // 记录方向
static uint16_t dutyfactor = 0;             // 记录占空比
static uint8_t is_motor_en = 0;
utils_pid_t current_pid;
utils_pid_t speed_pid;

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void gpio_config(void)
{
    rcu_periph_clock_enable(SHUTDOWM_GPIO_CLK);

    /*Configure SHUTDOWM_GPIO output with push-pull */
    gpio_init(SHUTDOWM_GPIO_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,SHUTDOWM_GPIO_PIN);

    gpio_bit_reset(SHUTDOWM_GPIO_PORT, SHUTDOWM_GPIO_PIN);
}

void device_motor_set_speed(uint16_t v) {
    if (v > COMPAER_MAX)
    {
        v = COMPAER_MAX;
    }

    dutyfactor = v;
    if (direction == MOTOR_FWD)
    {
        SET_FWD_COMPAER(dutyfactor);     // 设置速度
    }
    else
    {
        SET_REV_COMPAER(dutyfactor);     // 设置速度
    }
}

int32_t device_motor_get_speed(void) {
    static volatile int32_t cuurent_count = 0;
    static volatile int32_t last_count = 0;
    int32_t actual_speed = 0;

    cuurent_count = bsp_encoder_get_current_count();
    actual_speed = ((float)(cuurent_count - last_count) / ENCODER_TOTAL_RESOLUTION / REDUCTION_RATIO) / (VELOCITY_MEASURING_PERIOD / 1000.0f / 60.0f);
    last_count = cuurent_count;

	return actual_speed;
}

void device_motor_set_direction(motor_dir_t dir) {
    direction = dir;

    if (direction == MOTOR_FWD)
    {
        SET_FWD_COMPAER(dutyfactor);     // 设置速度
        SET_REV_COMPAER(0);              // 设置速度
    }
    else
    {
        SET_FWD_COMPAER(0);              // 设置速度
        SET_REV_COMPAER(dutyfactor);     // 设置速度
    }
}

motor_dir_t device_motor_get_direction(void) {
    return direction;
}

void device_motor_enable(void) {
    is_motor_en = 1;
    gpio_bit_set(SHUTDOWM_GPIO_PORT, SHUTDOWM_GPIO_PIN);
}

void device_motor_disable(void) {
    is_motor_en = 0;
    gpio_bit_reset(SHUTDOWM_GPIO_PORT, SHUTDOWM_GPIO_PIN);
    SET_FWD_COMPAER(0);
    SET_REV_COMPAER(0);
}
int32_t actual_speed = 0;
float actual_current = 0;
void device_motor_pid_control(void) {
    static uint8_t flag_target = 0;		//提供10ms基准
    static uint32_t time_count = 0;
    float current_loop_output;
    float speed_loop_output;

    flag_target = !flag_target;
    if (flag_target == 0)
        return ;
    actual_current = bsp_adc_get_current_value();       // 10ms 更新电流信息
    actual_speed = device_motor_get_speed();           // 10ms 更新速度信息

    time_count++;
    /* 待系统稳定偏置电压才为有效值 && 电机使能 进行闭环控制 */
    if (bsp_adc_get_offset_flag() >= OFFSET_COUNT && is_motor_en == 1) {
        /* PID1 : P 9 D 2 */
        /* PID2 : P 5 D 2 */
        speed_loop_output = utils_pid_realize(&speed_pid, actual_speed);
        if (speed_loop_output > 0) {
            device_motor_set_direction(MOTOR_REV);
        } else {
            device_motor_set_direction(MOTOR_FWD);
            speed_loop_output = -speed_loop_output;
        }
        speed_loop_output = speed_loop_output > COMPAER_MAX ? COMPAER_MAX : speed_loop_output;
        device_motor_set_speed(speed_loop_output);
#if 0
        /* 电流环控制周期 10ms */
        current_loop_output =  utils_pid_realize(&current_pid, actual_current);
        current_loop_output = current_loop_output < 0 ? 0 : current_loop_output;
        current_loop_output = current_loop_output > COMPAER_MAX ? COMPAER_MAX : current_loop_output;
        device_motor_set_speed(current_loop_output);
#endif

    }

    if (time_count % 5 == 0) { // 50ms
#if defined(PID_ASSISTANT_EN)
    int32_t temp = actual_current;
    // set_computer_value(SEND_FACT_CMD, CURVES_CH1, &temp, 1);      // 给通道 1 发送实际值
    temp = actual_speed;
    set_computer_value(SEND_FACT_CMD, CURVES_CH2, &temp, 1);      // 给通道 2 发送实际值
    temp = current_loop_output;
    // set_computer_value(SEND_FACT_CMD, CURVES_CH3, &temp, 1);      // 给通道3 发送实际值
#else
    LOG_D("current_value:%f mA", actual_current);
    LOG_D("motor_speed:%d r/min", actual_speed);
    LOG_D("power_supply_value:%f V", bsp_adc_get_power_supply_value());
#endif
    }
}

void device_motor_init(void) {
    gpio_config();
    device_motor_disable();
    device_motor_set_speed(0);
    device_motor_set_direction(MOTOR_FWD);
    device_motor_enable();
}

