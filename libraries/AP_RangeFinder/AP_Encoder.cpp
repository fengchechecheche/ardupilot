#include <AP_HAL/AP_HAL.h>
#include "AP_Encoder.h"
#include "AP_Encoder_MT6701_I2C.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder.h 和 AP_Encoder.cpp 负责前台类的实现
 */

extern const AP_HAL::HAL &hal;
#define SlaveAddress    0X0C        //MT6701 地址

AP_Encoder::AP_Encoder()
{

}

void 
AP_Encoder::init(void)
{
    // 此处就将前台类后后台“具体干活”的类通过后台类联系起来了
    _driver = new AP_Encoder_MT6701_I2C(*this);

    // if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
    //                                                                   hal.i2c_mgr->get_device(i, params[instance].address)),
    //                              instance)) {
    //                 break;
    //             }
    
    hal.i2c_mgr->get_device(0, SlaveAddress);
}

double
AP_Encoder::read(void)
{
    double fangle = 0;

    fangle = _driver->read();

    return fangle;
}


void AP_Encoder::update(void)
{
//     for (uint8_t i=0; i<num_instances; i++) {
//         if (drivers[i] != nullptr) {
//             if ((Type)params[i].type.get() == Type::NONE) {
//                 // allow user to disable a rangefinder at runtime
//                 state[i].status = Status::NotConnected;
//                 state[i].range_valid_count = 0;
//                 continue;
//             }
//             drivers[i]->update();
//         }
//     }
// #if HAL_LOGGING_ENABLED
//     Log_RFND();
// #endif

    _driver->read();
    
}
