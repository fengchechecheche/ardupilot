#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_Encoder_MT6701_I2C.h"
// #include "ssd1306.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder_MT6701_I2c.h 和 AP_Encoder_MT6701_I2c.cpp 负责“具体干活”
 * 驱动程序相关的API用C语言实现，在视频教程中没有被提到，后续再想办法
 */

extern const AP_HAL::HAL &hal;
#define ReadAddress1 0X03 // 数据高位寄存器地址
#define ReadAddress2 0X04 // 数据低位寄存器地址

/* ---------------------------------------------- suwp方案代码 ------------------------------------------------------ */
// AP_Encoder_MT6701_I2C::AP_Encoder_MT6701_I2C(AP_Encoder& encoder)
// : AP_Encoder_Backend(encoder)
// {
//     // 此处放硬件初始化的C代码
//     // uint8_t check = SSD1306_Init();
//     // while(check == 0){
//     //     HAL_Delay(1);
//     // }
//     // SSD1306_Fill(SSD1306_COLOR_BLACK);
//     // SSD1306_UpdateScreen();

//     // 10,000,000  ---->  10秒
//     //  5,000,000  ---->   5秒
//     _dev->register_periodic_callback(5000000,
//                                      FUNCTOR_BIND_MEMBER(&AP_Encoder_MT6701_I2C::encoder_timer, void));
// }
/* ---------------------------------------------- suwp方案代码 ------------------------------------------------------ */

AP_Encoder_MT6701_I2C::AP_Encoder_MT6701_I2C(AP_Encoder &encoder, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_Encoder_Backend(encoder), _dev(std::move(dev)) {}

AP_Encoder_Backend *AP_Encoder_MT6701_I2C::detect(AP_Encoder &encoder, AP_HAL::OwnPtr<AP_HAL::I2CDevice> i2c_dev)
{
    if (!i2c_dev)
    {
        return nullptr;
    }

    AP_Encoder_MT6701_I2C *sensor = new AP_Encoder_MT6701_I2C(encoder, std::move(i2c_dev));
    if (!sensor)
    {
        return nullptr;
    }

    // if (!sensor->init()) {
    //     delete sensor;
    //     return nullptr;
    // }
    sensor->init();

    return sensor;
}

bool AP_Encoder_MT6701_I2C::init()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[2] run AP_Encoder_MT6701_I2C::init() start.\n");
    if (encoder_init())
    {
        hal.console->printf("Found MT6701 Encoder.\n");
        return true;
    }
    hal.console->printf("Encoder not found.\n");
    return false;
}

bool AP_Encoder_MT6701_I2C::encoder_init()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[3] run AP_Encoder_MT6701_I2C::encoder_init() start.\n");
    union
    {
        be16_t be16_val;
        uint8_t bytes[2];
    } timeout;

    // Retrieve lost signal timeout register
    const uint8_t read_reg = ReadAddress1;
    if (!_dev->transfer(&read_reg, 1, timeout.bytes, 2))
    {
        return false;
    }

    // call timer() at 20Hz
    _dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&AP_Encoder_MT6701_I2C::encoder_timer, void));

    return true;
}

double
AP_Encoder_MT6701_I2C::read(void)
{
    double fangle = 0;

    return fangle;
}

void AP_Encoder_MT6701_I2C::encoder_timer(void)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "run AP_Encoder_MT6701_I2C::encoder_timer() start.");

    float reading_m;

    get_reading(reading_m);
    // if (legacy_get_reading(state.distance_m)) {
    //     // update range_valid state based on distance measured
    //     update_status();
    // } else {
    //     set_status(RangeFinder::Status::NoData);
    // }
}

void AP_Encoder_MT6701_I2C::get_reading(float &reading_m)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "run get_reading().");

    be16_t val;

    const uint8_t read_reg = ReadAddress1;

    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val)))
    {
        int16_t signed_val = int16_t(be16toh(val));
        if (signed_val < 0)
        {
            // some lidar firmwares will return 65436 for out of range
            // reading_m = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM) * 0.01f;
            reading_m = signed_val;
        }
        else
        {
            reading_m = uint16_t(signed_val) * 0.01f;
        }
        // return true;
    }
    // return false;
}
