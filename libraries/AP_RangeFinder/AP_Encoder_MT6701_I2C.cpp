#include <GCS_MAVLink/GCS.h>
#include "AP_Encoder_MT6701_I2C.h"
// #include "ssd1306.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder_MT6701_I2c.h 和 AP_Encoder_MT6701_I2c.cpp 负责“具体干活”
 * 驱动程序相关的API用C语言实现，在视频教程中没有被提到，后续再想办法
 */

AP_Encoder_MT6701_I2C::AP_Encoder_MT6701_I2C(AP_Encoder& encoder)
: AP_Encoder_Backend(encoder)
{
    // 此处放硬件初始化的C代码
    // uint8_t check = SSD1306_Init();
    // while(check == 0){
    //     HAL_Delay(1);
    // }
    // SSD1306_Fill(SSD1306_COLOR_BLACK);
    // SSD1306_UpdateScreen();

    // 10,000,000  ---->  10秒
    //  5,000,000  ---->   5秒
    _dev->register_periodic_callback(5000000,
                                     FUNCTOR_BIND_MEMBER(&AP_Encoder_MT6701_I2C::encoder_timer, void));
}

double
AP_Encoder_MT6701_I2C::read(void)
{
    double fangle = 0;


    return fangle;
}

void AP_Encoder_MT6701_I2C::encoder_timer(void)
{
    get_reading();
    // if (legacy_get_reading(state.distance_m)) {
    //     // update range_valid state based on distance measured
    //     update_status();
    // } else {
    //     set_status(RangeFinder::Status::NoData);
    // }
}

void AP_Encoder_MT6701_I2C::get_reading(void)
{
    // be16_t val;

//     const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

//     // read the high and low byte distance registers
//     if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
//         int16_t signed_val = int16_t(be16toh(val));
//         if (signed_val < 0) {
//             // some lidar firmwares will return 65436 for out of range
//             reading_m = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM) * 0.01f;
//         } else {
//             reading_m = uint16_t(signed_val) * 0.01f;
//         }
//         return true;
//     }
//     return false;

    gcs().send_text(MAV_SEVERITY_CRITICAL, "run get_reading().");
}
