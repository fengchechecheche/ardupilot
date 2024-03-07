#include <AP_HAL/AP_HAL.h>
#include <AP_MATH/AP_Math.h>
#include <GCS_Mavlink/GCS.h>
#include "AP_Encoder.h"
#include "AP_Encoder_MT6701_I2C.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder.h 和 AP_Encoder.cpp 负责前台类的实现
 */

// AP_Encoder& encoder: 这是一个对 AP_Encoder 类型的对象的引用。
// 你需要创建一个 AP_Encoder 对象或者已经有一个现有的对象，并将其作为引用传递给 detect 方法。
// 创建AP_Encoder对象 
AP_Encoder encoder; 

AP_Encoder::AP_Encoder()
{

}

void AP_Encoder::init(void)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[1] run AP_Encoder::init() start\n.");

    if (_add_backend(AP_Encoder_MT6701_I2C::detect(encoder, hal.i2c_mgr->get_device(0, SlaveAddress)), 0, 0))
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-1] run detect success.\n");
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-2] run detect failed.\n");
    }
}

bool AP_Encoder::_add_backend(AP_Encoder_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= ENCODER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

void AP_Encoder::update(void)
{
    drivers[0]->update();
}
