#ifndef __AP_ENCODER_BACKEND_H__
#define __AP_ENCODER_BACKEND_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Encoder.h"

extern const AP_HAL::HAL& hal;

class AP_Encoder_Backend{
public:
    // constructor. This incorporates initialisation as well.
	AP_Encoder_Backend(AP_Encoder& encoder);

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};
private:
    // 在后台类的私有成员中，会用到前台类
    // 在后台类中，如果想要通过对前台类的引用，调用到前台类和它的私有成员的话
    // 需要在前台类中声明后台类和后台类的子类为友元（friend class）类
    // 这样的话，后台类就可以通过_frontend调用到前台类中的私有成员
    AP_Encoder& _frontend;
};

#endif
