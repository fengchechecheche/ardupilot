/*
  simple test of RC output interface
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
 */

#include "AP_HAL/AP_HAL.h"

// we need a boardconfig created so that the io processor's enable
// parameter is available
// 用的不是CHIBIOS的内核，所以下面的头文件不被包含
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();   // 使能全部14个输出口
void loop();    // 让14个输出口的PWM占空比在1000到2000之间往复增减

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOutput test\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }
}

static uint16_t pwm = 1500;
static int8_t delta = 1;

void loop (void)
{
    for (uint8_t i=0; i < 14; i++) {
        // 这里的pwm值在1000到2000之间，对应的是PWM波的占空比
        // 频率默认是50Hz
        hal.rcout->write(i, pwm);
        pwm += delta;
        if (delta > 0 && pwm >= 2000) {
            delta = -1;
            hal.console->printf("decreasing\n");
        } else if (delta < 0 && pwm <= 1000) {
            delta = 1;
            hal.console->printf("increasing\n");
        }
    }
    hal.scheduler->delay(5);
}

AP_HAL_MAIN();
